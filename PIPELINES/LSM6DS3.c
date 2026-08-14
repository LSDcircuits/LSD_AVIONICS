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
// [DS: Sec 8, Table 16] Register address map
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

// One FIFO packet with our config = 3 datasets x 3 words = 9 words = 18 bytes
#define NAV_PACKET_WORDS  9
#define NAV_PACKET_BYTES  18

// Navigation sample: raw int16 + sensor timestamp + host timestamp.
typedef struct {
    int16_t  gx, gy, gz;
    int16_t  ax, ay, az;
    uint32_t sensor_ts;      // LSM6DS3 24-bit timestamp (25 µs/LSB, wraps at 2^24)
    uint64_t host_ts_us;     // RP2040 time_us_64() at read time (READ time, not sample time)
} imu_nav_sample;

// chip select
static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }

// Low-level SPI helpers
// [DS: Sec 6.2, Figure 8-13] SPI read/write protocols (Mode 3)
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

// Reset/flush the FIFO: drop to Bypass mode (clears content) then back to
// Continuous mode. [DS: Sec 5.4.1/5.4.2] "Bypass mode is also used to reset
// the FIFO". Keeps ODR_FIFO = 1.66 kHz in both writes.
static void fifo_reset(void) {
    reg_write(FIFO_CTRL5, 0x40);   // ODR_FIFO=1000, FIFO_MODE=000 (Bypass) -> flush
    reg_write(FIFO_CTRL5, 0x46);   // ODR_FIFO=1000, FIFO_MODE=110 (Continuous)
}

// Check whether at least one complete packet is stored.
// A packet = Gyro set (3 words) + Accel set (3 words) + TS/Step set (3 words) = 9 words.
// [DS: Sec 9.52/9.53] DIFF_FIFO is 12 bits: FIFO_STATUS1 = low 8 bits,
// FIFO_STATUS2[3:0] = high 4 bits  -> mask MUST be 0x0F (your 0x03 truncated it).
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

// Sync guard: FIFO_PATTERN[9:0] (FIFO_STATUS3/4) is the index of the NEXT word
// to be read inside the repeating dataset pattern. With our 9-word pattern it
// cycles 0..8, and 0 means "next word = gyro X of a new packet". If we ever
// lose alignment (e.g. after an overrun), reset instead of decoding garbage.
static bool fifo_packet_aligned(void) {
    uint8_t pat[2];
    reg_read(FIFO_STATUS3, pat, 2);
    uint16_t pattern = pat[0] | ((uint16_t)(pat[1] & 0x03) << 8);
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

    // 18-byte burst from FIFO_DATA_OUT_L; IF_INC=1 (CTRL3_C) makes the chip
    // walk the FIFO word by word. Packet layout with our config:
    //   raw[ 0.. 5] = gyro  X,Y,Z  (little-endian int16)
    //   raw[ 6..11] = accel X,Y,Z  (little-endian int16)
    //   raw[12..17] = 4th dataset (AN4650 Table 75, NOT plain little-endian):
    //       raw[12] = TIMESTAMP[15:8]
    //       raw[13] = TIMESTAMP[23:16]
    //       raw[14] = unused
    //       raw[15] = TIMESTAMP[7:0]
    //       raw[16] = STEPS[7:0]   (pedometer, ignore if unwanted)
    //       raw[17] = STEPS[15:8]
    uint8_t raw[NAV_PACKET_BYTES];
    reg_read(FIFO_DATA_OUT_L, raw, NAV_PACKET_BYTES);

    s->gx = (int16_t)((raw[1]  << 8) | raw[0]);
    s->gy = (int16_t)((raw[3]  << 8) | raw[2]);
    s->gz = (int16_t)((raw[5]  << 8) | raw[4]);
    s->ax = (int16_t)((raw[7]  << 8) | raw[6]);
    s->ay = (int16_t)((raw[9]  << 8) | raw[8]);
    s->az = (int16_t)((raw[11] << 8) | raw[10]);

    s->sensor_ts = ((uint32_t)raw[13] << 16) |   // TS[23:16]
                   ((uint32_t)raw[12] << 8)  |   // TS[15:8]
                   raw[15];                      // TS[7:0]
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

   // [DS: Sec 9.14, Table 52-53] CTRL3_C: BDU=1, IF_INC=1
    reg_write(CTRL3_C, 0x44);

   // [DS: Sec 9.12, Table 45-47] CTRL1_XL: ODR 1.66 kHz, FS ±16 g, BW 400 Hz
    reg_write(CTRL1_XL, 0x84);

   // [DS: Sec 9.13, Table 49-51] CTRL2_G: ODR 1.66 kHz, FS ±2000 dps
    reg_write(CTRL2_G, 0x8C);

   // [DS: Sec 9.21] CTRL10_C: FUNC_EN=1 (needed for timestamp/pedo dataset).
   // Default 0x38 keeps gyro X/Y/Z enabled — don't write a bare 0x04.
    reg_write(CTRL10_C, 0x3C);

   // [DS: Sec 9.85] TAP_CFG: TIMER_EN=1 (bit7) + PEDO_EN=1 (bit6)
   // per AN4650 Sec 8.8 procedure for timestamp+step in FIFO.
    reg_write(TAP_CFG, 0xC0);

   // [DS: Sec 9.76, Table 184-185] TIMER_HR=1 -> timestamp resolution 25 µs/LSB
    reg_write(WAKE_UP_DUR, 0x10);

    // [DS: Sec 9.3/9.4] FTH = 9 words = exactly one packet
    reg_write(FIFO_CTRL1, 0x09);

    // [DS: Sec 9.4] TIMER_PEDO_FIFO_EN=1 -> 4th dataset = step counter + timestamp;
    // TIMER_PEDO_FIFO_DRDY=0 -> FIFO writes triggered by XL/Gyro data-ready
    reg_write(FIFO_CTRL2, 0x80);

    // [DS: Sec 9.5] gyro & accel: no decimation (both at full FIFO ODR)
    reg_write(FIFO_CTRL3, 0x09);

    // [DS: Sec 9.6, Table 29-32]
    //  DEC_DS4_FIFO[5:3] = 001 -> 4th dataset NO decimation (timestamp every packet)
    //  DEC_DS3_FIFO[2:0] = 000 -> 3rd dataset not in FIFO
    //  (0x10 was DEC_DS4=010 = ÷2 -> alternating 9-word/6-word packets: the bug)
    reg_write(FIFO_CTRL4, 0x08);

    // Flush anything collected while configuring, then start Continuous mode.
    fifo_reset();

    // Reset the 24-bit timestamp counter so t=0 is a known epoch.
    // [DS: Sec 9.60] writing 0xAA to TIMESTAMP2_REG resets it.
    reg_write(TIMESTAMP2_REG, 0xAA);

    // [DS: Sec 5.4] first sample after FIFO mode switch must be discarded —
    // the flush above empties the FIFO, and we simply let fresh data refill it.
    sleep_ms(5);
    return true;
}

int main(void) {
    stdio_init_all();
    sleep_ms(200);
    if (!imu_init()) {
        while (true) { tight_loop_contents(); }
    }

    imu_nav_sample s;
    uint32_t print_decim = 0;
    uint32_t prev_sensor_ts = 0;
    bool have_prev = false;

    while (true) {
        if (imu_read_burst(&s)) {
            /* ---- Scale to physical units (keep raw ints in struct for your KF) ---- */
            float gx_dps = s.gx * GYRO_2000DPS_SENS;
            float gy_dps = s.gy * GYRO_2000DPS_SENS;
            float gz_dps = s.gz * GYRO_2000DPS_SENS;

            float ax_g   = s.ax * ACC_16G_SENS_G;
            float ay_g   = s.ay * ACC_16G_SENS_G;
            float az_g   = s.az * ACC_16G_SENS_G;

            float ax_ms2 = ax_g * G_TO_MS2;
            float ay_ms2 = ay_g * G_TO_MS2;
            float az_ms2 = az_g * G_TO_MS2;

            // Sample-to-sample sensor-time delta, wrap-safe over the 24-bit counter.
            // Expect ~24 ticks (24 x 25 µs = 600 µs = 1/1.66 kHz).
            uint32_t dt_ticks = have_prev ? ((s.sensor_ts - prev_sensor_ts) & 0xFFFFFFu) : 0;
            prev_sensor_ts = s.sensor_ts;
            have_prev = true;

            /* USB stdio cannot sustain 1.66 kHz prints; decimate for human viewing.
               Your quaternion/Kalman code will consume *every* sample, not print them. */
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
            }
        }
    }
    return 0;
}
