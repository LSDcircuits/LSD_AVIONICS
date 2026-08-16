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
#define STATUS_REG      0x1E
#define OUTX_L_G        0x22
#define TIMESTAMP0_REG  0x40
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define CTRL10_C        0x19
#define CTRL3_C         0x12

#define WAKE_UP_DUR     0x5C
#define FIFO_CTRL5      0x0A

typedef struct {
    int16_t  gx, gy, gz;
    int16_t  ax, ay, az;
    uint32_t sensor_ts;      // LSM6DS3 24-bit timestamp (25 µs/LSB, wraps at 2^24)
    uint64_t host_ts_us;     // RP2040 time_us_64() at read time
} imu_nav_sample;

typedef struct {
    float gx_off, gy_off, gz_off;
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

// -----------------------------------------------------------------------------
// Polled read: check STATUS_REG, then burst gyro+accel from output registers.
// -----------------------------------------------------------------------------
static bool imu_read_polled(imu_nav_sample *s) {
    uint8_t status;
    reg_read(STATUS_REG, &status, 1);

    // Bit 0 = XL_DA (accel data available), Bit 1 = G_DA (gyro data available)
    if ((status & 0x03) != 0x03)
        return false;

    // Burst read 12 bytes starting at OUTX_L_G (0x22).
    // With IF_INC=1, this auto-increments through gyro (0x22-0x27) then
    // accel (0x28-0x2D) with no gaps.
    uint8_t buf[12];
    reg_read(OUTX_L_G, buf, 12);

    s->gx = (int16_t)(((uint16_t)buf[1]  << 8) | (uint16_t)buf[0]);
    s->gy = (int16_t)(((uint16_t)buf[3]  << 8) | (uint16_t)buf[2]);
    s->gz = (int16_t)(((uint16_t)buf[5]  << 8) | (uint16_t)buf[4]);
    s->ax = (int16_t)(((uint16_t)buf[7]  << 8) | (uint16_t)buf[6]);
    s->ay = (int16_t)(((uint16_t)buf[9]  << 8) | (uint16_t)buf[8]);
    s->az = (int16_t)(((uint16_t)buf[11] << 8) | (uint16_t)buf[10]);

    // Timestamp from dedicated registers (NOT the FIFO).
    uint8_t ts[3];
    reg_read(TIMESTAMP0_REG, ts, 3);
    s->sensor_ts = ((uint32_t)ts[2] << 16) | ((uint32_t)ts[1] << 8) | (uint32_t)ts[0];

    s->host_ts_us = time_us_64();
    return true;
}

// Initialization: SPI + sensor core settings. FIFO is left in Bypass mode.
static bool imu_init(void) {
    // SPI @ 10 MHz, Mode 3 (CPOL=1, CPHA=1)
    spi_init(IMU_SPI, 10 * 1000 * 1000);
    spi_set_format(IMU_SPI, 8, true, true, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_high();

    sleep_ms(5);

    // Identify
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
    reg_write(CTRL10_C, 0x20)
    reg_write(WAKE_UP_DUR, 0x00);  // TIMER_EN = 1 (free-running timestamp)
    // Put FIFO in Bypass mode so it doesn't fill/overflow in the background.
    reg_write(FIFO_CTRL5, 0x00);
    sleep_ms(5);
    return true;
}


// Gyro zero-rate offset calibration at boot.
static bool calibrate_gyro(uint16_t samples) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t discard = 100;

    for (uint16_t i = 0; i < samples + discard; ) {
        imu_nav_sample s;
        if (imu_read_polled(&s)) {
            if (i >= discard) {
                sum_x += s.gx;
                sum_y += s.gy;
                sum_z += s.gz;
            }
            i++;
        }
    }

    cal.gx_off = (float)sum_x / (float)samples;
    cal.gy_off = (float)sum_y / (float)samples;
    cal.gz_off = (float)sum_z / (float)samples;

    printf("Gyro offsets: X=%.2f Y=%.2f Z=%.2f LSB\n",
           cal.gx_off, cal.gy_off, cal.gz_off);
    return true;
}

// Main
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
        if (imu_read_polled(&s)) {
            // Scale to physical units
            float gx_dps = (s.gx - cal.gx_off) * GYRO_2000DPS_SENS;
            float gy_dps = (s.gy - cal.gy_off) * GYRO_2000DPS_SENS;
            float gz_dps = (s.gz - cal.gz_off) * GYRO_2000DPS_SENS;

            float ax_g   = s.ax * ACC_16G_SENS_G;
            float ay_g   = s.ay * ACC_16G_SENS_G;
            float az_g   = s.az * ACC_16G_SENS_G;

            float ax_ms2 = ax_g * G_TO_MS2;
            float ay_ms2 = ay_g * G_TO_MS2;
            float az_ms2 = az_g * G_TO_MS2;

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
                sleep_ms(10);
            }
        }
    }
    return 0;
}
