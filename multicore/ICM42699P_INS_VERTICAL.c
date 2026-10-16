#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/time.h"

#define IMU_SPI   spi1
#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_MISO  12
#define PIN_CS    13

#define REG_DEVICE_CONFIG       0x11
#define REG_BANK_SEL            0x76
#define REG_FIFO_CONFIG         0x16
#define REG_FIFO_CONFIG1        0x5F
#define REG_FIFO_CONFIG2        0x60
#define REG_FIFO_CONFIG3        0x61
#define REG_INT_CONFIG1         0x64
#define REG_INT_SOURCE0         0x65
#define REG_GYRO_CONFIG0        0x4F
#define REG_ACCEL_CONFIG0       0x50
#define REG_GYRO_ACCEL_CONFIG0  0x52
#define REG_SIGNAL_PATH_RESET   0x4B
#define REG_PWR_MGMT0           0x4E
#define REG_WHO_AM_I            0x75
#define REG_FIFO_COUNTH         0x2E
#define REG_FIFO_DATA           0x30
#define REG_INT_STATUS          0x2D

#define WHO_AM_I_EXPECTED       0x47
#define FIFO_WATERMARK_BYTES    64u
#define FIFO_PACKET_SIZE        16u
#define FIFO_MAX_BURST_SAMPLES  32

#define IMU_ODR_HZ              1000.0f
#define IMU_DT_S                (1.0f / IMU_ODR_HZ)

#define GYRO_LSB_PER_DPS        16.4f
#define ACCEL_LSB_PER_G         2048.0f
#define G_TO_MS2                9.80665f
#define FIFO_TEMP_LSB_PER_C     2.07f

// (-1) == reverse & 1 == normal
#define M_ROLL                  -1
#define M_PITCH                  1
#define M_YAW                   -1

typedef struct {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int8_t   temp;
    uint16_t sensor_ts;
    uint64_t host_ts_us;
} imu_nav_sample;

typedef struct {
    float gx_off, gy_off, gz_off;   // gyro bias, LSB
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

static inline float gyro_dps(int16_t raw, float off) {
    return ((float)raw - off) / GYRO_LSB_PER_DPS;
}
static inline float accel_g(int16_t raw) {
    return (float)raw / ACCEL_LSB_PER_G;
}
static inline float fifo_temp_c(int8_t t) {
    return 25.0f + (float)t / FIFO_TEMP_LSB_PER_C;
}

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

    reg_write(REG_DEVICE_CONFIG, 0x01);
    sleep_ms(10);

    reg_write(REG_BANK_SEL, 0x00);
    uint8_t who = 0;
    reg_read(REG_WHO_AM_I, &who, 1);
    printf("WHO_AM_I=0x%02X (expect 0x%02X)\n", who, WHO_AM_I_EXPECTED);
    if (who != WHO_AM_I_EXPECTED) {
        printf("IMU not found. Aborting.\n");
        return false;
    }
    reg_write(REG_FIFO_CONFIG, 0x40);
    reg_write(REG_GYRO_CONFIG0, 0x06);
    reg_write(REG_ACCEL_CONFIG0, 0x06);
    reg_write(REG_GYRO_ACCEL_CONFIG0, 0x11);
    reg_write(REG_FIFO_CONFIG1, 0x47);
    reg_write(REG_FIFO_CONFIG2, (uint8_t)(FIFO_WATERMARK_BYTES & 0xFF));
    reg_write(REG_FIFO_CONFIG3, 0x00);
    reg_write(REG_INT_CONFIG1, 0x00);
    reg_write(REG_INT_SOURCE0, 0x04);
    reg_write(REG_PWR_MGMT0, 0x0F);
    sleep_us(200);
    sleep_ms(50);
    reg_write(REG_SIGNAL_PATH_RESET, 0x02);
    sleep_ms(1);
    uint8_t dummy;
    reg_read(REG_INT_STATUS, &dummy, 1);
    return true;
}

static int imu_fifo_read(imu_nav_sample *out, int max_samples) {
    if (out == NULL || max_samples <= 0)
        return 0;

    static uint8_t buf[FIFO_MAX_BURST_SAMPLES * FIFO_PACKET_SIZE];
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
        s->host_ts_us = burst_ts + (uint64_t)i * 1000u;
        n++;
    }
    return n;
}

// ---------------- Mahony 6DOF attitude estimator ----------------

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
static float gyro_bias_mahony[3] = {0.0f, 0.0f, 0.0f};
static float norm;
static float s_lp = 1.0f;
#define MAHONY_KP_0  0.3f
#define MAHONY_KI_0  0.1f

// Estimated gravity/up direction in body frame (Mahony error vector).
// NOTE: whether this points up or down is convention-dependent, so the
// sign is determined empirically during calibration (see up_sign below).
static inline void gravity_body(float *vx, float *vy, float *vz) {
    *vx = 2.0f*(q[1]*q[3] - q[0]*q[2]);
    *vy = 2.0f*(q[0]*q[1] + q[2]*q[3]);
    *vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
}

static void mahony_update(float gx_rad, float gy_rad, float gz_rad,
                          float ax_g, float ay_g, float az_g, float dt) {
    float np = 20.0f, ni = 10.0f, K = 1.0f;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    s_lp += 0.02f * (norm - s_lp);
    float gain_KP = -((np - np*s_lp)*(np - np*s_lp)) + K;
    float gain_KI = -((ni - ni*s_lp)*(ni - ni*s_lp)) + K;
    gain_KP = fmaxf(0.0f, gain_KP);
    gain_KI = fmaxf(0.0f, gain_KI);

    float MAHONY_KP = MAHONY_KP_0 * gain_KP;
    float MAHONY_KI = MAHONY_KI_0 * gain_KI;

    if (norm > 0.1f) {
        ax_g /= norm; ay_g /= norm; az_g /= norm;

        float vx, vy, vz;
        gravity_body(&vx, &vy, &vz);

        ex = (ay_g * vz - az_g * vy);
        ey = (az_g * vx - ax_g * vz);
        ez = (ax_g * vy - ay_g * vx);

        gyro_bias_mahony[0] += ex * MAHONY_KI * dt;
        gyro_bias_mahony[1] += ey * MAHONY_KI * dt;
        gyro_bias_mahony[2] += ez * MAHONY_KI * dt;

        gx_rad += MAHONY_KP * ex + gyro_bias_mahony[0];
        gy_rad += MAHONY_KP * ey + gyro_bias_mahony[1];
        gz_rad += MAHONY_KP * ez + gyro_bias_mahony[2];
    }

    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float q_dot_w = 0.5f * (-qx*gx_rad - qy*gy_rad - qz*gz_rad);
    float q_dot_x = 0.5f * ( qw*gx_rad + qy*gz_rad - qz*gy_rad);
    float q_dot_y = 0.5f * ( qw*gy_rad - qx*gz_rad + qz*gx_rad);
    float q_dot_z = 0.5f * ( qw*gz_rad + qx*gy_rad - qy*gx_rad);

    q[0] += q_dot_w * dt;
    q[1] += q_dot_x * dt;
    q[2] += q_dot_y * dt;
    q[3] += q_dot_z * dt;

    norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
}

// ---------------- Vertical-channel INS ----------------

static float alt_vel   = 0.0f;  // m/s, positive UP
static float alt       = 0.0f;  // m, relative to arming point
static float a_prev_up = 0.0f;
static uint64_t last_ts_us = 0;
static bool ins_started = false;

// Auto-determined during calibration: which way gravity_body() points,
// and the measured stationary projection magnitude (~1.0 g).
static float up_sign  = 1.0f;
static float up_g_ref = 1.0f;

static void ins_reset(void) {
    alt = 0.0f;
    alt_vel = 0.0f;
    a_prev_up = 0.0f;
    ins_started = false;
    last_ts_us = 0;
}

static float a_up_dbg = 0.0f;

static void ins_update_vertical(float ax_g, float ay_g, float az_g, uint64_t ts_us)
{
    if (!ins_started) { last_ts_us = ts_us; ins_started = true; return; }
    uint64_t dts = ts_us - last_ts_us;
    last_ts_us = ts_us;
    if (dts == 0 || dts > 50000) { a_prev_up = 0.0f; return; }  // FIFO gap

    float dt = (float)dts * 1e-6f;

    float vx, vy, vz;
    gravity_body(&vx, &vy, &vz);

    // Project specific force (g) onto the Mahony up/down axis, correct the
    // sign empirically, and subtract the MEASURED stationary projection.
    // This absorbs both the up/down convention AND the accel bias projected
    // on the vertical axis, so no per-axis accel offsets are needed.
    float proj = ax_g*vx + ay_g*vy + az_g*vz;        // in g
    float a_up = (up_sign * proj - up_g_ref) * G_TO_MS2;

    // sanity clamp: keeps shocks/FIFO glitches from integrating to km
    const float A_MAX = 4.0f * G_TO_MS2;
    if (a_up >  A_MAX) a_up =  A_MAX;
    if (a_up < -A_MAX) a_up = -A_MAX;
    a_up_dbg = a_up;

    float a_avg = 0.5f * (a_up + a_prev_up);
    alt     += alt_vel * dt + 0.5f * a_avg * dt * dt;
    alt_vel += a_avg * dt;
    a_prev_up = a_up;
}

// ---------------- Calibration ----------------

static bool calibrate_imu(uint16_t samples) {
    if (samples == 0)
        return false;
    const uint16_t discard = 100;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    float sum_proj = 0.0f;
    uint16_t got = 0, skipped = 0;

    printf("CAL: keep board perfectly still...\n");
    sleep_ms(500);
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
            // feed Mahony so attitude converges while we average
            float gx_rad = gyro_dps(batch[i].gx, 0.0f) * (float)(M_PI / 180.0);
            float gy_rad = gyro_dps(batch[i].gy, 0.0f) * (float)(M_PI / 180.0);
            float gz_rad = gyro_dps(batch[i].gz, 0.0f) * (float)(M_PI / 180.0);
            float axg = accel_g(batch[i].ax);
            float ayg = accel_g(batch[i].ay);
            float azg = accel_g(batch[i].az);
            mahony_update(gx_rad, gy_rad, gz_rad, axg, ayg, azg, IMU_DT_S);

            if (skipped < discard) {
                skipped++;
                continue;
            }
            sum_x += batch[i].gx;
            sum_y += batch[i].gy;
            sum_z += batch[i].gz;

            float vx, vy, vz;
            gravity_body(&vx, &vy, &vz);
            sum_proj += axg*vx + ayg*vy + azg*vz;   // ~ +1g or -1g
            got++;
        }
    }

    cal.gx_off = (float)sum_x / (float)samples;
    cal.gy_off = (float)sum_y / (float)samples;
    cal.gz_off = (float)sum_z / (float)samples;

    // Empirically determine the up/down sign and the stationary projection:
    // whatever sign makes the stationary projection equal to +1 g IS up.
    float rest_proj = sum_proj / (float)samples;
    up_sign  = (rest_proj >= 0.0f) ? 1.0f : -1.0f;
    up_g_ref = fabsf(rest_proj);   // ~1.0; absorbs scale + projected bias

    printf("Gyro offsets: X=%7.2f Y=%7.2f Z=%7.2f LSB\n",
           cal.gx_off, cal.gy_off, cal.gz_off);
    printf("Up-axis: sign=%+.0f  rest projection=%+.3f g\n", up_sign, rest_proj);
    if (up_g_ref < 0.7f || up_g_ref > 1.3f)
        printf("WARNING: stationary projection far from 1g - board not still during cal?\n");
    return true;
}

// ---------------- Helpers ----------------

static float qm[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static void q_mirror(const float quatm[4]){
    qm[0] = quatm[0];
    qm[1] = quatm[1]*(M_ROLL);
    qm[2] = quatm[2]*(M_PITCH);
    qm[3] = quatm[3]*(M_YAW);
}

static void quat_to_euler(const float quat[4], float *roll, float *pitch, float *yaw) {
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    *pitch = asinf(2.0f*(w*y - z*x));
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

// ---------------- Main ----------------

int main(void) {
    stdio_init_all();
    sleep_ms(200);

    if (!imu_init()) {
        while (true) { tight_loop_contents(); }
    }

    calibrate_imu(2000);
    ins_reset();   // altitude reference = here, now

    imu_nav_sample batch[FIFO_MAX_BURST_SAMPLES];
    uint64_t prev_ts = 0;
    uint32_t next_print = time_us_32() + 100000;

    while (true) {
        int n = imu_fifo_read(batch, FIFO_MAX_BURST_SAMPLES);
        if (n <= 0) {
            sleep_ms(1);
            continue;
        }

        for (int i = 0; i < n; i++) {
            const imu_nav_sample *s = &batch[i];

            // per-sample dt from timestamps (fall back to nominal 1 ms)
            uint64_t dts = (prev_ts == 0) ? 1000 : (s->host_ts_us - prev_ts);
            prev_ts = s->host_ts_us;
            float dt = (dts > 0 && dts < 50000) ? (float)dts * 1e-6f : IMU_DT_S;

            float gx_rad = gyro_dps(s->gx, cal.gx_off) * (float)(M_PI / 180.0);
            float gy_rad = gyro_dps(s->gy, cal.gy_off) * (float)(M_PI / 180.0);
            float gz_rad = gyro_dps(s->gz, cal.gz_off) * (float)(M_PI / 180.0);
            float ax_g   = accel_g(s->ax);
            float ay_g   = accel_g(s->ay);
            float az_g   = accel_g(s->az);
            float mag    = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

            mahony_update(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, dt);
            ins_update_vertical(ax_g, ay_g, az_g, s->host_ts_us);

            // ZUPT: stationary -> kill vertical velocity drift
            if (fabsf(mag - 1.0f) < 0.02f &&
                fabsf(gx_rad) + fabsf(gy_rad) + fabsf(gz_rad) < 0.05f) {
                alt_vel = 0.0f;
            }
        }

        // decimated output: 20 Hz, time-based (printing at 1 kHz blocks the loop)
        uint32_t now = time_us_32();
        if ((int32_t)(now - next_print) >= 0) {
            next_print = now + 50000;
            float roll, pitch, yaw;
            quat_to_euler(q, &roll, &pitch, &yaw);
            q_mirror(q);
            printf("RPY:%6.1f %6.1f %6.1f deg | ALT:%8.3f m  V:%7.3f m/s  a_up:%6.2f | Qm:%5.2f %5.2f %5.2f %5.2f | s_lp:%4.2f\n",
                   roll  * 180.0f / (float)M_PI,
                   pitch * 180.0f / (float)M_PI,
                   yaw   * 180.0f / (float)M_PI,
                   alt, alt_vel, a_up_dbg,
                   qm[0], qm[1], qm[2], qm[3], s_lp);
        }

        sleep_ms(1);
    }
    return 0;
}
