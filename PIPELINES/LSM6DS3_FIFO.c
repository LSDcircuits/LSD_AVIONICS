#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/time.h"

// RP2040-Zero <-> LSM6DS3 (4-wire SPI)
// GP2 -> SCL/SPC (SCK)
// GP3 -> SDA/SDI (MOSI)
// GP0 -> SDO/SA0 (MISO)
// GP1 -> CS  (active low)

#define IMU_SPI   spi0
#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  0
#define PIN_CS    1

// Scale factors — [DS: Sec 4.1, Table 3] Mechanical characteristics
#define GYRO_2000DPS_SENS   0.07f        // mdps/LSB -> dps/LSB  (FS=±2000 dps)
#define ACC_16G_SENS_G      0.000488f    // mg/LSB -> g/LSB      (FS=±16 g)
#define G_TO_MS2            9.80665f

// Register map
#define WHO_AM_I        0x0F
#define FIFO_CTRL1      0x06
#define FIFO_CTRL2      0x07
#define FIFO_CTRL3      0x08
#define FIFO_CTRL4      0x09
#define FIFO_CTRL5      0x0A
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define CTRL3_C         0x12
#define CTRL10_C        0x19
#define TAP_CFG         0x58
#define WAKE_UP_DUR     0x5C
#define FIFO_STATUS1    0x3A
#define FIFO_STATUS2    0x3B
#define FIFO_STATUS3    0x3C
#define FIFO_STATUS4    0x3D
#define FIFO_DATA_OUT_L 0x3E
#define TIMESTAMP0_REG  0x40
#define TIMESTAMP1_REG  0x41
#define TIMESTAMP2_REG  0x42

// FIFO packet: Gyro (3 words) + Accel (3 words) = 6 words = 12 bytes.
// Timestamp is NOT stored in the FIFO with this config; read from dedicated registers.
#define NAV_PACKET_WORDS  6
#define NAV_PACKET_BYTES  12

typedef struct {
    int16_t  gx, gy, gz;
    int16_t  ax, ay, az;
    uint32_t sensor_ts;      // LSM6DS3 24-bit timestamp (25 µs/LSB, wraps at 2^24)
    uint64_t host_ts_us;     // RP2040 time_us_64() at read time
} imu_nav_sample;

typedef struct {
    float gx_off, gy_off, gz_off;
    float ax_off, ay_off, az_off;
} imu_cal_t;

static imu_cal_t cal = {0};

static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }

static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    cs_low();
    spi_write_blocking(IMU_SPI, tx, 2);
    cs_high();
}

static void reg_read(uint8_t reg, uint8_t *dst, size_t n) {
    uint8_t hdr = (uint8_t)(reg | 0x80);   // bit7 = 1 => read
    cs_low();
    spi_write_blocking(IMU_SPI, &hdr, 1);
    spi_read_blocking(IMU_SPI, 0x00, dst, n);
    cs_high();
}

static void fifo_reset(void) {
    // Bypass mode immediately clears the FIFO.
    reg_write(FIFO_CTRL5, 0x00);
    sleep_us(50);
    // Restore continuous mode at 1.66 kHz.
    reg_write(FIFO_CTRL5, 0x46);
    sleep_us(50);
}

static bool nav_sample_available(void) {
    uint8_t st[2];
    reg_read(FIFO_STATUS1, st, 2);
    // OVER_RUN: at least one sample was overwritten -> stream can no longer be
    // trusted to be contiguous. Flush and wait for fresh aligned data.
    if (st[1] & 0x40) {
        fifo_reset();
        return false;
    }
    uint16_t words = st[0] | ((uint16_t)(st[1] & 0x0F) << 8);
    return words >= NAV_PACKET_WORDS;
}

static bool fifo_packet_aligned(void) {
    uint8_t pat[2];
    reg_read(FIFO_STATUS3, pat, 2);
    uint16_t pattern = pat[0] | ((uint16_t)(pat[1] & 0x03) << 8);
    // Pattern 0 means the next word is gyro X (start of a 6-word group).
    return (pattern % NAV_PACKET_WORDS) == 0;
}

bool imu_read_burst(imu_nav_sample *s) {
    if (!nav_sample_available())
        return false;

    if (!fifo_packet_aligned()) {
        fifo_reset();
        return false;
    }

    // Host read timestamp — metadata about WHEN YOU READ, not sample time.
    s->host_ts_us = time_us_64();

    // 12-byte burst from FIFO_DATA_OUT_L; IF_INC=1 (CTRL3_C) auto-increments address.
    uint8_t raw[NAV_PACKET_BYTES];
    reg_read(FIFO_DATA_OUT_L, raw, NAV_PACKET_BYTES);

    s->gx = (int16_t)((raw[1]  << 8) | raw[0]);
    s->gy = (int16_t)((raw[3]  << 8) | raw[2]);
    s->gz = (int16_t)((raw[5]  << 8) | raw[4]);
    s->ax = (int16_t)((raw[7]  << 8) | raw[6]);
    s->ay = (int16_t)((raw[9]  << 8) | raw[8]);
    s->az = (int16_t)((raw[11] << 8) | raw[10]);

    // Read 24-bit timestamp from dedicated registers (NOT the FIFO).
    // TIMESTAMP0_REG = 0x40; auto-increment (IF_INC=1) reads 0x40, 0x41, 0x42.
    uint8_t ts[3];
    reg_read(TIMESTAMP0_REG, ts, 3);
    s->sensor_ts = ((uint32_t)ts[2] << 16) | ((uint32_t)ts[1] << 8) | (uint32_t)ts[0];

    return true;
}

static bool imu_init(void) {
    // [DS: Sec 4.4.1, Table 6] SPI @ 10 MHz, Mode 3 (CPOL=1, CPHA=1)
    spi_init(IMU_SPI, 10 * 1000 * 1000);
    spi_set_format(IMU_SPI, 8, true, true, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_high();

    sleep_ms(5);

    // Identify BEFORE configuring: if this fails, every write below goes nowhere.
    uint8_t who = 0;
    reg_read(WHO_AM_I, &who, 1);
    printf("WHO_AM_I=0x%02X (expect 0x69)\n", who);
    if (who != 0x69) {
        printf("IMU not found / wrong part — check wiring. Aborting init.\n");
        return false;
    }

    // Core settings
    reg_write(CTRL3_C, 0x44);      // IF_INC=1, BDU=1
    reg_write(CTRL1_XL, 0x84);     // XL 1.66 kHz, ±16 g
    reg_write(CTRL2_G, 0x8C);      // Gyro 1.66 kHz, ±2000 dps
    reg_write(CTRL10_C, 0x3C);
    reg_write(TAP_CFG, 0xC0);
    reg_write(WAKE_UP_DUR, 0x10);  // Timer enabled (bit 4 = TIMER_EN)

    // --- FIFO config: gyro + accel ONLY, no timestamp inside FIFO ---
    // FIFO_CTRL3 layout: [0][0][DEC_GYRO2:0][DEC_XL2:0]
    // 0x09 = 0b00001001 -> both gyro & XL at 001 = no decimation.
    // DO NOT set bit 7 or 6 to 1; the datasheet requires them to be 0.
    reg_write(FIFO_CTRL1, 0x06);   // Watermark = 6 words (1 packet)
    reg_write(FIFO_CTRL2, 0x00);   // No timer/step/temp in FIFO
    reg_write(FIFO_CTRL3, 0x09);   // Gyro + XL, both no decimation
    reg_write(FIFO_CTRL4, 0x00);   // No 3rd/4th datasets
    reg_write(FIFO_CTRL5, 0x00);   // Bypass first to clear any old/stale data
    sleep_us(50);
    reg_write(FIFO_CTRL5, 0x46);   // Continuous mode, 1.66 kHz FIFO ODR
    sleep_us(50);

    return true;
}

static bool calibrate_gyro(uint16_t samples) {
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t collected = 0;
    uint16_t discard = 50;   // discard first 50 packets to let sensor settle

    // Drain stale FIFO data first
    fifo_reset();

    while (collected < samples + discard) {
        if (!nav_sample_available()) {
            tight_loop_contents();   // non-blocking wait
            continue;
        }

        imu_nav_sample s;
        if (!imu_read_burst(&s)) continue;

        if (collected >= discard) {
            sum_gx += s.gx;
            sum_gy += s.gy;
            sum_gz += s.gz;
        }
        collected++;
    }

    cal.gx_off = (float)sum_gx / (float)samples;
    cal.gy_off = (float)sum_gy / (float)samples;
    cal.gz_off = (float)sum_gz / (float)samples;

    printf("Gyro offsets: X=%.2f Y=%.2f Z=%.2f LSB\n",
           cal.gx_off, cal.gy_off, cal.gz_off);
    return true;
}

int main(void) {
    stdio_init_all();
    sleep_ms(200);
    if (!imu_init()) {
        while (true) { tight_loop_contents(); }
    }

    printf("Keep IMU stationary for calibration...\n");
    sleep_ms(500);
    calibrate_gyro(512);

    imu_nav_sample s;
    uint32_t print_decim = 0;
    uint32_t prev_sensor_ts = 0;
    bool have_prev = false;

    while (true) {
        if (imu_read_burst(&s)) {
            // Scale to physical units (keep raw ints in struct for your KF)
            float gx_dps = (s.gx - cal.gx_off) * GYRO_2000DPS_SENS;
            float gy_dps = (s.gy - cal.gy_off) * GYRO_2000DPS_SENS;
            float gz_dps = (s.gz - cal.gz_off) * GYRO_2000DPS_SENS;

            float ax_g   = s.ax * ACC_16G_SENS_G;
            float ay_g   = s.ay * ACC_16G_SENS_G;
            float az_g   = s.az * ACC_16G_SENS_G;

            float ax_ms2 = ax_g * G_TO_MS2;
            float ay_ms2 = ay_g * G_TO_MS2;
            float az_ms2 = az_g * G_TO_MS2;

            // Sample-to-sample sensor-time delta, wrap-safe over the 24-bit counter.
            uint32_t dt_ticks = have_prev ? ((s.sensor_ts - prev_sensor_ts) & 0xFFFFFFu) : 0;
            prev_sensor_ts = s.sensor_ts;
            have_prev = true;

            // USB stdio cannot sustain 1.66 kHz prints; decimate for human viewing.
            if (++print_decim >= 100) {
                print_decim = 0;
                printf("G:%7.2f %7.2f %7.2f dps | "
                       "A:%6.3f %6.3f %6.3f g (%6.2f %6.2f %6.2f m/s2) | "
                       "SensTS:%8lu | dTS:%3lu ticks | HostUS:%llu\n",
                       gx_dps, gy_dps, gz_dps,
                       ax_g, ay_g, az_g,
                       ax_ms2, ay_ms2, az_ms2,
                       (unsigned long)s.sensor_ts, (unsigned long)dt_ticks,
                       s.host_ts_us);
                sleep_ms(1);
            }
        }
    }
    return 0;
}
