#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
#define N_PI                    3.14159265358979323846

// (-1) == reverse & 1 == normal
#define M_ROLL                  -1 
#define M_PITCH                   1 
#define M_YAW                   -1 

typedef struct {
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    int8_t   temp;
    uint16_t sensor_ts;   
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
        s->ax = ((int16_t)(((uint16_t)p[1]  << 8) | p[2]));
        s->ay = (int16_t)(((uint16_t)p[3]  << 8) | p[4]);
        s->az = ((int16_t)(((uint16_t)p[5]  << 8) | p[6]));
        s->gx = ((int16_t)(((uint16_t)p[7]  << 8) | p[8]));
        s->gy = (int16_t)(((uint16_t)p[9]  << 8) | p[10]);
        s->gz = (int16_t)(((uint16_t)p[11] << 8) | p[12]);
        s->temp = (int8_t)p[13];
        s->sensor_ts = (uint16_t)(((uint16_t)p[14] << 8) | p[15]);
        s->host_ts_us = burst_ts + (uint64_t)i * 1000u;  
        n++;
    }
    return n;
}
static bool calibrate_gyro(uint16_t samples) {
    if (samples == 0)
        return false;
    const uint16_t discard = 100;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
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
            if (skipped < discard) { 
                skipped++;
                continue;
            }
            sum_x += (batch[i].gx);
            sum_y += (batch[i].gy);
            sum_z += (batch[i].gz);
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

// Mahony 6DOF attitude estimator

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z  (body relative to NED)
static float gyro_bias_mahony[3] = {0.0f, 0.0f, 0.0f};
static float gain;
static float norm;
#define MAHONY_KP_0  0.3f //0.3
#define MAHONY_KI_0  0.1f //0.1



static void mahony_update(float gx_rad, float gy_rad, float gz_rad, float ax_g, float ay_g, float az_g, float dt) {
    float n = 8;
    float K = 1;

    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    // Normalize accelerometer (only correct attitude when not in freefall)
    norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    gain = -((n*1 - n*norm)*(n*1 - n*norm)) + K;
    gain = fmaxf(0.0f, gain); // clamped

    float MAHONY_KP =  MAHONY_KP_0 * gain;
    float MAHONY_KI =  MAHONY_KI_0 * gain;

    // change 
    if (norm > 0.1f) {
        ax_g /= norm; ay_g /= norm; az_g /= norm;

        // Estimated gravity direction in body frame (3rd column of R(q))
        // computes q = [w,x,y,z] quaternion
        float vx = 2.0f*(q[1]*q[3] - q[0]*q[2]);
        float vy = 2.0f*(q[0]*q[1] + q[2]*q[3]);
        float vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

        // Error = measured gravity x estimated gravity
        // cross product (ax,ay,az)(x)(vx,vy,vz)
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

// mirroring fucntion for quaternion representation.
static float qm[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static void q_mirror(const float quatm[4]){
    qm[0] = quatm[0]; 
    qm[1] = quatm[1]*(M_ROLL);
    qm[2] = quatm[2]*(M_PITCH);
    qm[3] = quatm[3]*(M_YAW);
}

static void var_gain(){
}


static void quat_to_euler(const float quat[4], float *roll, float *pitch, float *yaw) {
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    *pitch = asinf(2.0f*(w*y - z*x));
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

// Main 
int main(void) {
    stdio_init_all();
    sleep_ms(200);

    if (!imu_init()) {
        while (true) { tight_loop_contents(); }
    }

    printf("Keep IMU stationary for calibration...\n");
    calibrate_gyro(2000);

    imu_nav_sample batch[FIFO_MAX_BURST_SAMPLES]; // stack of 32 structs 
    uint32_t print_decim = 0;

    while (true) {
        printf("serial workjing\n");
        int n = imu_fifo_read(batch, FIFO_MAX_BURST_SAMPLES);
        if (n <= 0) {
            sleep_ms(1);
            continue;
        }

        for (int i = 0; i < n; i++) {
            const imu_nav_sample *s = &batch[i];
            float gx_rad = gyro_dps(s->gx, cal.gx_off) * (M_PI/180);
            float gy_rad = gyro_dps(s->gy, cal.gy_off) * (M_PI/180);
            float gz_rad = gyro_dps(s->gz, cal.gz_off) * (M_PI/180);
            float ax_g   = accel_g(s->ax);
            float ay_g   = accel_g(s->ay);
            float az_g   = accel_g(s->az);
            float mag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
            float temp_c = fifo_temp_c(s->temp);
            mahony_update(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, 0.001f);
            if(i == n - 1){
                float roll, pitch, yaw;
                quat_to_euler(q, &roll, &pitch, &yaw);
                q_mirror(q);
                printf("G:%6.2f %6.2f %6.2f rad | "
                       "A:%5.3f %5.3f %5.3f g | "
                       "Q:%6.3f %6.3f %6.3f %6.3f | "
                       "RPY:%5.1f %5.1f %5.1f deg\n"
                       "Norm:%6.3f %6.3f ",
                       gx_rad, gy_rad, gz_rad,
                       ax_g, ay_g, az_g,
                       qm[0], qm[1], qm[2], qm[3],
                       roll * 180.0f / M_PI,
                       pitch * 180.0f / M_PI,
                       yaw * 180.0f / M_PI,
                       mag, gain);
            }
        }
        sleep_ms(2);
    }
    return 0;
}
