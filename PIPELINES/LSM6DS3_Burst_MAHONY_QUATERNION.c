#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/time.h"

// RP2040-Zero <-> LSM6DS3 (4-wire SPI)
#define IMU_SPI   spi0
#define PIN_SCK   2
#define PIN_MOSI  3
#define PIN_MISO  0
#define PIN_CS    1

// Scale factors
#define GYRO_2000DPS_SENS   0.07f
#define ACC_16G_SENS_G      0.000488f
#define G_TO_MS2            9.80665f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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
    uint32_t sensor_ts;
    uint64_t host_ts_us;
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
    uint8_t hdr = (uint8_t)(reg | 0x80);
    cs_low();
    spi_write_blocking(IMU_SPI, &hdr, 1);
    spi_read_blocking(IMU_SPI, 0x00, dst, n);
    cs_high();
}

// Polled read
static bool imu_read_polled(imu_nav_sample *s) {
    uint8_t status;
    reg_read(STATUS_REG, &status, 1);
    if ((status & 0x03) != 0x03)
        return false;

    uint8_t buf[12];
    reg_read(OUTX_L_G, buf, 12);

    s->gx = (int16_t)(((uint16_t)buf[1]  << 8) | (uint16_t)buf[0]);
    s->gy = (int16_t)(((uint16_t)buf[3]  << 8) | (uint16_t)buf[2]);
    s->gz = (int16_t)(((uint16_t)buf[5]  << 8) | (uint16_t)buf[4]);
    s->ax = (int16_t)(((uint16_t)buf[7]  << 8) | (uint16_t)buf[6]);
    s->ay = (int16_t)(((uint16_t)buf[9]  << 8) | (uint16_t)buf[8]);
    s->az = (int16_t)(((uint16_t)buf[11] << 8) | (uint16_t)buf[10]);

    uint8_t ts[3];
    reg_read(TIMESTAMP0_REG, ts, 3);
    s->sensor_ts = ((uint32_t)ts[2] << 16) | ((uint32_t)ts[1] << 8) | (uint32_t)ts[0];

    s->host_ts_us = time_us_64();
    return true;
}

// -----------------------------------------------------------------------------
// Mahony 6DOF attitude estimator
// -----------------------------------------------------------------------------
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z  (body relative to NED)
static float gyro_bias_mahony[3] = {0.0f, 0.0f, 0.0f};
static uint64_t last_attitude_us = 0;

#define MAHONY_KP  0.1f
#define MAHONY_KI  0.05f

static void mahony_update(float gx_rad, float gy_rad, float gz_rad,
                          float ax_g, float ay_g, float az_g, float dt)
{
    float norm;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    // Normalize accelerometer (only correct attitude when not in freefall)
    norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (norm > 0.1f) {
        ax_g /= norm; ay_g /= norm; az_g /= norm;

        // Estimated gravity direction in body frame (3rd column of R(q))
        float vx = 2.0f*(q[1]*q[3] - q[0]*q[2]);
        float vy = 2.0f*(q[0]*q[1] + q[2]*q[3]);
        float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

        // Error = measured gravity x estimated gravity
        ex = (ay_g * vz - az_g * vy);
        ey = (az_g * vx - ax_g * vz);
        ez = (ax_g * vy - ay_g * vx);

        // Integrate error into gyro bias estimate
        gyro_bias_mahony[0] += ex * MAHONY_KI * dt;
        gyro_bias_mahony[1] += ey * MAHONY_KI * dt;
        gyro_bias_mahony[2] += ez * MAHONY_KI * dt;

        // Correct gyro with PI feedback
        gx_rad += MAHONY_KP * ex + gyro_bias_mahony[0];
        gy_rad += MAHONY_KP * ey + gyro_bias_mahony[1];
        gz_rad += MAHONY_KP * ez + gyro_bias_mahony[2];
    }

    // Quaternion derivative: q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float q_dot_w = 0.5f * (-qx*gx_rad - qy*gy_rad - qz*gz_rad);
    float q_dot_x = 0.5f * ( qw*gx_rad + qy*gz_rad - qz*gy_rad);
    float q_dot_y = 0.5f * ( qw*gy_rad - qx*gz_rad + qz*gx_rad);
    float q_dot_z = 0.5f * ( qw*gz_rad + qx*gy_rad - qy*gx_rad);

    // Integrate
    q[0] += q_dot_w * dt;
    q[1] += q_dot_x * dt;
    q[2] += q_dot_y * dt;
    q[3] += q_dot_z * dt;

    // Normalize
    norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

// Convert quaternion to Euler angles (radians) for human debugging
static void quat_to_euler(const float quat[4], float *roll, float *pitch, float *yaw)
{
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    *pitch = asinf(2.0f*(w*y - z*x));
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

// -----------------------------------------------------------------------------
// Init
// -----------------------------------------------------------------------------
static bool imu_init(void) {
    spi_init(IMU_SPI, 10 * 1000 * 1000);
    spi_set_format(IMU_SPI, 8, true, true, SPI_MSB_FIRST);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    cs_high();

    sleep_ms(5);

    uint8_t who = 0;
    reg_read(WHO_AM_I, &who, 1);
    printf("WHO_AM_I=0x%02X (expect 0x69)\n", who);
    if (who != 0x69) {
        printf("IMU not found. Aborting.\n");
        return false;
    }

    reg_write(CTRL3_C, 0x44);
    reg_write(CTRL1_XL, 0x84);
    reg_write(CTRL2_G, 0x8C);
    reg_write(WAKE_UP_DUR, 0x01);
    reg_write(FIFO_CTRL5, 0x00);
    sleep_ms(5);
    return true;
}

// -----------------------------------------------------------------------------
// Calibration
// -----------------------------------------------------------------------------
static bool calibrate_gyro(uint16_t samples) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t discard = 100;

    printf("CAL: keep board perfectly still...\n");
    sleep_ms(500);

    for (uint16_t i = 0; i < samples + discard; ) {
        imu_nav_sample s;
        if (imu_read_polled(&s)) {
            if (i >= discard) {
                sum_x += s.gx; sum_y += s.gy; sum_z += s.gz;
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

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
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
            float gx_dps = (s.gx - cal.gx_off) * GYRO_2000DPS_SENS * -1 ;
            float gy_dps = (s.gy - cal.gy_off) * GYRO_2000DPS_SENS;
            float gz_dps = (s.gz - cal.gz_off) * GYRO_2000DPS_SENS * -1;

            float ax_g = s.ax * ACC_16G_SENS_G;
            float ay_g = s.ay * ACC_16G_SENS_G;
            float az_g = s.az * ACC_16G_SENS_G;

            // --- Attitude update (host time for dt) ---
            float dt;
            if (last_attitude_us == 0) {
                dt = 1.0f / 1660.0f;  // first sample fallback
            } else {
                dt = (float)(s.host_ts_us - last_attitude_us) * 1e-6f;
                if (dt <= 0.0f || dt > 0.01f) dt = 1.0f / 1660.0f;
            }
            last_attitude_us = s.host_ts_us;

            float gx_rad = gx_dps * (M_PI / 180.0f);
            float gy_rad = gy_dps * (M_PI / 180.0f);
            float gz_rad = gz_dps * (M_PI / 180.0f);

            mahony_update(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, dt);

            // --- Debug print ---
            uint32_t dt_ticks = have_prev ? ((s.sensor_ts - prev_sensor_ts) & 0xFFFFFFu) : 0;
            prev_sensor_ts = s.sensor_ts;
            have_prev = true;

            if (++print_decim >= 100) {
                print_decim = 0;
                float roll, pitch, yaw;
                quat_to_euler(q, &roll, &pitch, &yaw);

                printf("G:%6.2f %6.2f %6.2f dps | "
                       "A:%5.3f %5.3f %5.3f g | "
                       "Q:%6.3f %6.3f %6.3f %6.3f | "
                       "RPY:%5.1f %5.1f %5.1f deg | "
                       "HostUS:%llu\n",
                       gx_dps, gy_dps, gz_dps,
                       ax_g, ay_g, az_g,
                       q[0], q[1], q[2], q[3],
                       roll * 180.0f / M_PI,
                       pitch * 180.0f / M_PI,
                       yaw * 180.0f / M_PI,
                       s.host_ts_us);
                sleep_ms(10);
            }
        }
    }
    return 0;
}
