#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/time.h"

// RP2040 <-> ICM-42688-P (4-wire SPI), pin map from example
#define IMU_SPI   spi0
#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  0
#define PIN_CS    1

//ICM-42688-P registers (bank 0)
#define REG_DEVICE_CONFIG       0x11  // bit0: soft reset
#define REG_FIFO_CONFIG         0x16  // 0x40 = stream mode
#define REG_INT_STATUS          0x2D  // bit2: FIFO threshold, bit1: FIFO full
#define REG_FIFO_COUNTH         0x2E  // read high first to latch
#define REG_FIFO_COUNTL         0x2F
#define REG_FIFO_DATA           0x30  // burst read port
#define REG_SIGNAL_PATH_RESET   0x4B  // bit1: FIFO flush
#define REG_PWR_MGMT0           0x4E  // 0x0F = gyro LN + accel LN, temp on
#define REG_GYRO_CONFIG0        0x4F  // 0x06 = +-2000 dps, 1 kHz ODR
#define REG_ACCEL_CONFIG0       0x50  // 0x06 = +-16 g, 1 kHz ODR
#define REG_GYRO_ACCEL_CONFIG0  0x52  // 0x11 = UI filter BW max(400,ODR)/4
#define REG_FIFO_CONFIG1        0x5F  // 0x47 = resume partial + temp+gyro+accel
#define REG_FIFO_CONFIG2        0x60  // watermark low byte
#define REG_FIFO_CONFIG3        0x61  // watermark high nibble
#define REG_INT_CONFIG1         0x64  // 0x00: INT_ASYNC_RESET=0, 100us pulse
#define REG_INT_SOURCE0         0x65  // 0x04: route FIFO threshold to INT1
#define REG_WHO_AM_I            0x75  // expect 0x47
#define REG_BANK_SEL            0x76

#define WHO_AM_I_EXPECTED       0x47
#define FIFO_WATERMARK_BYTES    64u   // 4 complete 16-byte packets
#define FIFO_PACKET_SIZE        16u

// Fixed ODR assumption (SPEC): 1 kHz
#define IMU_ODR_HZ              1000.0f
#define IMU_DT_S                (1.0f / IMU_ODR_HZ)

// Scaling (SPEC)
#define GYRO_LSB_PER_DPS        16.4f     // +-2000 dps
#define ACCEL_LSB_PER_G         2048.0f   // +-16 g
#define G_TO_MS2                9.80665f
#define FIFO_TEMP_LSB_PER_C     2.07f

// Max samples handled per FIFO burst (static buffers, no malloc)
#define FIFO_MAX_BURST_SAMPLES  32

//Data model (per SPEC) 
typedef struct {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int8_t   temp;
    uint16_t sensor_ts;   // FIFO ODR timestamp, microseconds, wraps at 65536
    uint64_t host_ts_us;  // burst timestamp + packet index * 1000 us
} imu_nav_sample;

typedef struct {
    float gx_off, gy_off, gz_off;   // raw LSB offsets
} imu_cal_t;

static imu_cal_t cal = {0};

// SPI low level
static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }

static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    cs_low();
    spi_write_blocking(IMU_SPI, tx, 2);
    cs_high();
}

static void reg_read(uint8_t reg, uint8_t *dst, size_t n) {
    uint8_t hdr = (uint8_t)(reg | 0x80);
    cs_low();
    spi_write_blocking(IMU_SPI, &hdr, 1);
    spi_read_blocking(IMU_SPI, 0x00, dst, n);
    cs_high();
}

// Scaling helpers
static inline float gyro_dps(int16_t raw, float off) {
    return ((float)raw - off) / GYRO_LSB_PER_DPS;
}
static inline float accel_g(int16_t raw) {
    return (float)raw / ACCEL_LSB_PER_G;
}
static inline float accel_ms2(int16_t raw) {
    return accel_g(raw) * G_TO_MS2;
}
static inline float fifo_temp_c(int8_t t) {
    return 25.0f + (float)t / FIFO_TEMP_LSB_PER_C;
}

// (Init / reset / WHOAMI)
static bool imu_init(void) {
    spi_init(IMU_SPI, 10 * 1000 * 1000);
    spi_set_format(IMU_SPI, 8, true, true, SPI_MSB_FIRST);  // SPI mode 3
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_high();

    sleep_ms(5);

    // Soft reset, then wait >= 1 ms
    reg_write(REG_DEVICE_CONFIG, 0x01);
    sleep_ms(10);

    // Stay in bank 0 (default after reset, but be explicit)
    reg_write(REG_BANK_SEL, 0x00);

    uint8_t who = 0;
    reg_read(REG_WHO_AM_I, &who, 1);
    printf("WHO_AM_I=0x%02X (expect 0x%02X)\n", who, WHO_AM_I_EXPECTED);
    if (who != WHO_AM_I_EXPECTED) {
        printf("IMU not found. Aborting.\n");
        return false;
    }

    // Configure everything while sensors are off.
    reg_write(REG_FIFO_CONFIG, 0x40);          // FIFO stream mode
    reg_write(REG_GYRO_CONFIG0, 0x06);         // +-2000 dps, 1 kHz
    reg_write(REG_ACCEL_CONFIG0, 0x06);        // +-16 g, 1 kHz
    reg_write(REG_GYRO_ACCEL_CONFIG0, 0x11);   // UI filter BW = 250 Hz @ 1 kHz
    reg_write(REG_FIFO_CONFIG1, 0x47);         // temp+gyro+accel into FIFO
    reg_write(REG_FIFO_CONFIG2, (uint8_t)(FIFO_WATERMARK_BYTES & 0xFF));
    reg_write(REG_FIFO_CONFIG3, 0x00);         // watermark high nibble = 0
    reg_write(REG_INT_CONFIG1, 0x00);          // 100 us pulse OK at 1 kHz
    reg_write(REG_INT_SOURCE0, 0x04);          // FIFO threshold -> INT1
    // INTF_CONFIG0 left at reset default 0x30 (big-endian, byte-count FIFO).
    // TMST_CONFIG left at reset default 0x23 (timestamp on, 1 us resolution).

    // Turn on gyro + accel (low noise), temp enabled.
    // After this change from off: no register writes for >= 200 us,
    // then wait for sensor startup.
    reg_write(REG_PWR_MGMT0, 0x0F);
    sleep_us(200);
    sleep_ms(50);

    // Flush FIFO so we start from a clean state.
    reg_write(REG_SIGNAL_PATH_RESET, 0x02);
    sleep_ms(1);

    // Read-to-clear any pending interrupt status.
    uint8_t dummy;
    reg_read(REG_INT_STATUS, &dummy, 1);

    return true;
}

// FIFO burst read
// Returns number of complete parsed samples (0..max_samples).
// Only complete 16-byte packets are consumed; burst size is capped.
static int imu_fifo_read(imu_nav_sample *out, int max_samples) {
    if (!out || max_samples <= 0)
        return 0;

    static uint8_t buf[FIFO_MAX_BURST_SAMPLES * FIFO_PACKET_SIZE];

    // Latch FIFO count: read high byte first.
    uint8_t cnt[2];
    reg_read(REG_FIFO_COUNTH, cnt, 2);
    uint16_t fifo_bytes = (uint16_t)(((uint16_t)cnt[0] << 8) | cnt[1]);

    int packets = (int)(fifo_bytes / FIFO_PACKET_SIZE);
    if (packets > max_samples)
        packets = max_samples;
    if (packets > FIFO_MAX_BURST_SAMPLES)
        packets = FIFO_MAX_BURST_SAMPLES;
    if (packets == 0)
        return 0;

    uint64_t burst_ts = time_us_64();
    reg_read(REG_FIFO_DATA, buf, (size_t)packets * FIFO_PACKET_SIZE);

    int n = 0;
    for (int i = 0; i < packets; i++) {
        const uint8_t *p = &buf[i * FIFO_PACKET_SIZE];
        uint8_t header = p[0];

        // Valid simple sample: HEADER_MSG=0, HEADER_ACCEL=1, HEADER_GYRO=1.
        // Hi-res/FSYNC/ODR-change flags ignored in this simple version.
        if ((header & 0x80) != 0)
            continue;
        if ((header & 0x60) != 0x60)
            continue;

        imu_nav_sample *s = &out[n];
        s->ax = (int16_t)(((uint16_t)p[1]  << 8) | p[2]);
        s->ay = (int16_t)(((uint16_t)p[3]  << 8) | p[4]);
        s->az = (int16_t)(((uint16_t)p[5]  << 8) | p[6]);
        s->gx = (int16_t)(((uint16_t)p[7]  << 8) | p[8]);
        s->gy = (int16_t)(((uint16_t)p[9]  << 8) | p[10]);
        s->gz = (int16_t)(((uint16_t)p[11] << 8) | p[12]);
        s->temp = (int8_t)p[13];
        s->sensor_ts = (uint16_t)(((uint16_t)p[14] << 8) | p[15]);
        s->host_ts_us = burst_ts + (uint64_t)i * 1000u;  // preserve 1 kHz packet spacing
        n++;
    }
    return n;
}

// Gyro calibration from FIFO samples
static bool calibrate_gyro(uint16_t samples) {
    if (samples == 0)
        return false;

    const uint16_t discard = 100;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t got = 0, skipped = 0;

    printf("CAL: keep board perfectly still...\n");
    sleep_ms(500);

    // Flush stale FIFO data before collecting.
    reg_write(REG_SIGNAL_PATH_RESET, 0x02);
    sleep_ms(2);

    imu_nav_sample batch[FIFO_MAX_BURST_SAMPLES];

    while (got < samples) {
        int n = imu_fifo_read(batch, FIFO_MAX_BURST_SAMPLES);
        if (n <= 0) {
            sleep_ms(1);
            continue;
        }
        for (int i = 0; i < n && got < samples; i++) {
            if (skipped < discard) {
                skipped++;
                continue;
            }
            sum_x += batch[i].gx;
            sum_y += batch[i].gy;
            sum_z += batch[i].gz;
            got++;
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
    calibrate_gyro(512);

    imu_nav_sample batch[FIFO_MAX_BURST_SAMPLES];
    uint32_t print_decim = 0;

    while (true) {
        int n = imu_fifo_read(batch, FIFO_MAX_BURST_SAMPLES);
        if (n <= 0) {
            sleep_ms(1);
            continue;
        }

        for (int i = 0; i < n; i++) {
            const imu_nav_sample *s = &batch[i];

            float gx_dps = gyro_dps(s->gx, cal.gx_off);
            float gy_dps = gyro_dps(s->gy, cal.gy_off);
            float gz_dps = gyro_dps(s->gz, cal.gz_off);
            float ax_g   = accel_g(s->ax);
            float ay_g   = accel_g(s->ay);
            float az_g   = accel_g(s->az);
            float temp_c = fifo_temp_c(s->temp);

            // Decimated print: 1 kHz samples -> ~10 Hz output.
            if (++print_decim >= 100) {
                print_decim = 0;
                printf("G:%7.2f %7.2f %7.2f dps | "
                       "A:%6.3f %6.3f %6.3f g | "
                       "T:%5.1f C | "
                       "STS:%u HostUS:%llu\n",
                       gx_dps, gy_dps, gz_dps,
                       ax_g, ay_g, az_g,
                       temp_c,
                       s->sensor_ts,
                       (unsigned long long)s->host_ts_us);
            }
        }
        sleep_ms(2);
    }
    return 0;
}
