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
#define ACC_8G_SENS_G       0.000244
#define ACC_4G_SENS_G       0.000122
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
#define TAP_CFG         0x58
#define WAKE_UP_DUR     0x5C
#define FIFO_STATUS1    0x3A
#define FIFO_STATUS2    0x3B
#define FIFO_DATA_OUT_L 0x3E
#define TIMESTAMP0_REG  0x40
#define TIMESTAMP1_REG  0x41
#define TIMESTAMP2_REG  0x42

// Navigation sample: raw int16 + sensor timestamp + host timestamp.
typedef struct {
    int16_t  gx, gy, gz;
    int16_t  ax, ay, az;
    uint32_t sensor_ts;      // LSM6DS3 24-bit timestamp (25 µs/LSB)
    uint64_t host_ts_us;     // RP2040 time_us_64() at read time
} imu_nav_sample;

// chip select
static inline void cs_low(void)  { gpio_put(PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(PIN_CS, 1); }

// Low-level SPI helpers
// [DS: Sec 6.2, Figure 8-13] SPI read/write protocols (Mode 3)
static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), val };
    cs_low();
    spi_write_blocking(IMU_SPI, tx, 2); //(SPI PORT|transmit|num bytes)
    cs_high();
}

static void reg_read(uint8_t reg, uint8_t *dst, size_t n) { // | register | received dat| size |
    uint8_t hdr = (uint8_t)(reg | 0x80);   // bit0 = 1 => read
    cs_low();
    spi_write_blocking(IMU_SPI, &hdr, 1); // write 
    spi_read_blocking(IMU_SPI, 0x00, dst, n);
    cs_high();
}

// Check whether at least one complete sample pair is stored.
// A pair = Gyro data set (3 words) + Accel data set (3 words) = 6 words.
// [DS: Sec 9.52/9.53, Table 135-138] FIFO_STATUS1 / FIFO_STATUS2
static bool nav_sample_available(void) {
    uint8_t st[2];
    reg_read(FIFO_STATUS1, st, 2);
    uint16_t words = st[0] | ((st[1] & 0x03) << 8);
    return words >= 6;
}

bool imu_read_burst(imu_nav_sample *s) {
       if (!nav_sample_available())
        return false;

    // Read timestamp first (before SPI bus gets busy)
    uint8_t ts[3];
    reg_read(TIMESTAMP0_REG, ts, 3);
    s->sensor_ts = ((uint32_t)ts[2] << 16) | ((uint32_t)ts[1] << 8) | ts[0];
    s->host_ts_us = time_us_64();

    // define raw data
    uint8_t raw[12];
    // 12 (8 bit)bytes (bytes 0-5 gyro, 6-11 acc)
    //or 6 (16 bit)bytes (bytes 0-2 gyro , 3-5 acc)
    reg_read(FIFO_DATA_OUT_L, raw, 12);
    s->gx = (int16_t)((raw[1] << 8 )| raw[0]);
    s->gy = (int16_t)((raw[3] << 8 )| raw[2]);
    s->gz = (int16_t)((raw[5] << 8 )| raw[4]);
    s->ax = (int16_t)((raw[7] << 8 )| raw[6]);
    s->ay = (int16_t)((raw[9] << 8 )| raw[8]);
    s->az = (int16_t)((raw[11] << 8 )| raw[10]);
    return true;
}

static void imu_init(void) {
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

   // [DS: Sec 9.14, Table 52-53] CTRL3_C
   // BDU=1   : Block Data Update — output registers freeze until both L/H read.
   // IF_INC=1: Address auto-increment during multi-byte SPI access.
   // SIM=0   : 4-wire SPI (SDO/SDI separate)
    reg_write(CTRL3_C, 0x44);   // 0x44

   // [DS: Sec 9.12, Table 45-47] CTRL1_XL
   // ODR_XL[3:0] = 1000 → 1.66 kHz (High-Performance mode)
   // FS_XL[1:0]  = 01   → ±16 g
   // BW_XL[1:0]  = 00   → 400 Hz analog anti-aliasing bandwidth
   // NOTE: Matched to gyro ODR so FIFO produces synchronous pairs. */
    reg_write(CTRL1_XL, 0x84);

    /* [DS: Sec 9.13, Table 49-51] CTRL2_G
       ODR_G[3:0] = 1000 → 1.66 kHz (High-Performance mode)
       FS_G[1:0]  = 11   → ±2000 dps
       FS_125     = 0    → 125 dps mode disabled
       (Your original 0xAC used ODR=1010 which is undefined for gyro;
        0x8C selects the documented 1.66 kHz rate.)                  */
    reg_write(CTRL2_G, 0x8C);

    /* [DS: Sec 9.72, Table 175-176] TAP_CFG
       TIMER_EN = 1 → Start the internal timestamp counter.          */
    reg_write(TAP_CFG, 0x80);

    /* [DS: Sec 9.76, Table 184-185] WAKE_UP_DUR
       TIMER_HR = 1 → Timestamp resolution = 25 µs/LSB.
       (Default 0 would give 6.4 ms/LSB, too coarse for navigation.)  */
    reg_write(WAKE_UP_DUR, 0x10);

    /* [DS: Sec 9.3/9.4, Table 21-24] FIFO_CTRL1 / FIFO_CTRL2
       FTH[11:0] = 6 words → interrupt/watermark every 1 sample pair.
       One pair = Gyro(3 words) + Accel(3 words).                    */
    reg_write(FIFO_CTRL1, 0x06);
    reg_write(FIFO_CTRL2, 0x00);

    /* [DS: Sec 9.5, Table 25-28] FIFO_CTRL3
       DEC_FIFO_GYRO[2:0] = 001 → no decimation (1.66 kHz)
       DEC_FIFO_XL[2:0]   = 001 → no decimation (1.66 kHz)          */
    reg_write(FIFO_CTRL3, 0x09);

    /* [DS: Sec 9.6, Table 29-32] FIFO_CTRL4
       3rd and 4th FIFO data sets disabled.                          */
    reg_write(FIFO_CTRL4, 0x00);

    /* [DS: Sec 9.7, Table 33-36] FIFO_CTRL5
       ODR_FIFO[3:0]  = 1000 → FIFO ODR = 1.66 kHz
       FIFO_MODE[2:0] = 110  → Continuous mode (overwrites oldest)   */
    reg_write(FIFO_CTRL5, 0x46);

    /* [DS: Sec 5.4.6] First sample after FIFO enable can be invalid;
       give the anti-alias filters a few ODR periods to settle.       */
    sleep_ms(5);
}

int main(void) {
    stdio_init_all();
    sleep_ms(200);
    imu_init();

    uint8_t who = 0;
    reg_read(WHO_AM_I, &who, 1);
    printf("WHO_AM_I=0x%02X (expect 0x69)\n", who);
    imu_nav_sample s;
    uint32_t print_decim = 0;

    while (true) {
        if (imu_nav_read(&s)) {
            /* ---- Scale to physical units (keep raw ints in struct for your KF) ---- */
            float gx_dps = s.gx * GYRO_2000DPS_SENS;
            float gy_dps = s.gy * GYRO_2000DPS_SENS;
            float gz_dps = s.gz * GYRO_2000DPS_SENS;

            float ax_g   = s.ax * ACC_16G_SENS_G;   // leave in g, or convert:
            float ay_g   = s.ay * ACC_16G_SENS_G;   
            float az_g   = s.az * ACC_16G_SENS_G;   

            float ax_ms2 = ax_g * G_TO_MS2;   
            float ay_ms2 = ay_g * G_TO_MS2;
            float az_ms2 = az_g * G_TO_MS2;

            /* USB stdio cannot sustain 1.66 kHz prints; decimate for human viewing.
               Your quaternion/Kalman code will consume *every* sample, not print them. */
            if (++print_decim >= 100) {
                print_decim = 0;
                printf("G:%7.2f %7.2f %7.2f dps | "
                       "A:%6.3f %6.3f %6.3f g (%6.2f %6.2f %6.2f m/s2) | "
                       "SensTS:%6lu | HostUS:%llu\n",
                       gx_dps, gy_dps, gz_dps,
                       ax_g, ay_g, az_g,
                       ax_ms2, ay_ms2, az_ms2,
                       s.sensor_ts, s.host_ts_us);
            }
        }
    }
    return 0;
}
