#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include "pico/multicore.h"

// ---- attitude codee -----

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
static float acc[3] = {0.0f, 0.0f, 0.0f};
static float gain_KI;
static float gain_KP;
static float norm;
static float s_lp;
#define MAHONY_KP_0  0.3f //0.3
#define MAHONY_KI_0  0.1f //0.1

static void mahony_update(float gx_rad, float gy_rad, float gz_rad, float ax_g, float ay_g, float az_g, float dt) {
    float np = 20;
    float ni = 5;
    float K = 1;

    float ex = 0.0f, ey = 0.0f, ez = 0.0f;

    // Normalize accelerometer (only correct attitude when not in freefall)
    norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    s_lp += 0.02f * (norm - s_lp); 
    gain_KP = -((np*1 - np*s_lp)*(np*1 - np*s_lp)) + K;
    gain_KI = -((ni*1 - ni*s_lp)*(ni*1 - ni*s_lp)) + K;
    gain_KP = fmaxf(0.0f, gain_KP); // clamped
    gain_KI = fmaxf(0.0f, gain_KI); // clamped


    float MAHONY_KP =  MAHONY_KP_0 * gain_KP;
    float MAHONY_KI =  MAHONY_KI_0 * gain_KI;

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

static void quat_to_euler(const float quat[4], float *roll, float *pitch, float *yaw) {
    float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    *roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    *pitch = asinf(2.0f*(w*y - z*x));
    *yaw   = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));
}

void attitude_core(){
    sleep_ms(200);
    if (!imu_init()){
        while(true) {tight_loop_contents();}
    }
    calibrate_gyro(2000);
    imu_nav_sample batch[FIFO_MAX_BURST_SAMPLES]; // stack of 32 structs
    uint32_t print_decim = 0;

    while (true) {
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
        acc[0] = ax_g; acc[2] = ay_g; acc[2] = az_g;
        mahony_update(gx_rad, gy_rad, gz_rad, ax_g, ay_g, az_g, 0.001f);
        q_mirror(q);
        }
        
    }
}


// ---- GPS code ---

// config 
#define GPS_UART        uart1
#define GPS_BAUD        38400
#define GPS_TX_PIN      4          // Pico TX -> GPS RX
#define GPS_RX_PIN      5          // Pico RX <- GPS TX

//  UBX constants
#define SYNC1           0xB5
#define SYNC2           0x62 
#define CLASS_NAV       0x01 // identify nav
#define ID_NAV_PVT      0x07 // used to check for correct message
#define NAV_PVT_PAYLOAD 92

// frame[] layout after sync pair: class(1) id(1) len(2) payload(92) ck_a ck_b
#define FRAME_MAX       (4 + NAV_PVT_PAYLOAD + 2)

// decoded data 
typedef struct {
    uint32_t iTOW;              //  0  ms
    uint16_t year;              //  4
    uint8_t  month, day, hour, min, sec;  // 6..10
    uint8_t  valid;             // 11
    uint32_t tAcc;              // 12  ns
    int32_t  nano;              // 16  ns
    uint8_t  fixType;           // 20
    uint8_t  flags;             // 21  bit0 = gnssFixOK
    uint8_t  flags2;            // 22
    uint8_t  numSV;             // 23
    int32_t  lon;               // 24  deg * 1e-7
    int32_t  lat;               // 28  deg * 1e-7
    int32_t  height;            // 32  mm
    int32_t  hMSL;              // 36  mm
    uint32_t hAcc;              // 40  mm
    uint32_t vAcc;              // 44  mm
    int32_t  velN, velE, velD;  // 48,52,56  mm/s
    int32_t  gSpeed;            // 60  mm/s
    int32_t  headMot;           // 64  deg * 1e-5
    uint32_t sAcc;              // 68  mm/s
    uint32_t headAcc;           // 72  deg * 1e-5
    uint16_t pDOP;              // 76  * 0.01
    int16_t  magDec;            // 88  deg * 1e-2
    uint16_t magAcc;            // 90  deg * 1e-2
} nav_pvt_t;

// receiver state (statics: memory survives between bytes) 
typedef enum { S_SYNC1, S_SYNC2, S_CLS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CK1, S_CK2 } state_t;

static state_t  st = S_SYNC1;
static uint8_t  frame[FRAME_MAX];   // raw frame bytes, class..ck_b
static uint16_t idx;                // next write position in frame[]
static uint16_t plen;               // payload length from the length field
static uint8_t  ck_a, ck_b;         // running Fletcher checksum

// Feed one byte. Returns true only when a complete, checksum-valid NAV-PVT
// frame is sitting in frame[].
static bool ubx_rx_byte(uint8_t b)
{
    switch (st) {
    case S_SYNC1:
        if (b == SYNC1) st = S_SYNC2; // 1 staement needed, hunts sync byte
        break;
    case S_SYNC2:
        // Equivalent {if (b == SYNC2) st = S_CLS; else st = S_SYNC1;}, 2 statements used to move to next or back
        st = (b == SYNC2) ? S_CLS : S_SYNC1; // = S_CLS if Sync2 present otherwise Back hunting
        break;
    case S_CLS:
        frame[0] = b; 
        ck_a = b; ck_b = ck_a;      // checksum starts at class
        st = S_ID;
        break;
    case S_ID:
        frame[1] = b; ck_a += b; ck_b += ck_a;
        st = S_LEN1;
        break;
    case S_LEN1:
        frame[2] = b; ck_a += b; ck_b += ck_a; plen = b;
        st = S_LEN2;
        break;
    case S_LEN2:
        frame[3] = b; ck_a += b; ck_b += ck_a;
        plen |= (uint16_t)b << 8;                 // little-endian length
        idx = 4;
        // we only want NAV-PVT: anything else, abandon and resync
        st = (plen == NAV_PVT_PAYLOAD) ? S_PAYLOAD : S_SYNC1;
        break;
    case S_PAYLOAD:
        frame[idx++] = b; ck_a += b; ck_b += ck_a;
        if (idx == 4 + plen) st = S_CK1;
        break;
    case S_CK1:
        st = (b == ck_a) ? S_CK2 : S_SYNC1;       // first checksum byte must match
        break;
    case S_CK2:
        st = S_SYNC1;
        return (b == ck_b) &&
               frame[0] == CLASS_NAV && frame[1] == ID_NAV_PVT;
    }
    return false;
}


// explicit field-by-field decode
// f points at the first payload byte (iTOW). Offsets match the u-blox spec.
static void decode_nav_pvt(const uint8_t *f, nav_pvt_t *s) {
    s->iTOW  = (uint32_t)f[0] | ((uint32_t)f[1] << 8) | ((uint32_t)f[2] << 16)| ((uint32_t)f[3] << 24);

    s->year  = (uint16_t)f[4] | ((uint16_t)f[5] << 8);

    s->month = f[6];

    s->day   = f[7];

    s->hour  = f[8];

    s->min   = f[9];

    s->sec   = f[10];

    s->valid = f[11];

    s->tAcc  = (uint32_t)f[12] | ((uint32_t)f[13] << 8) | ((uint32_t)f[14] << 16)| ((uint32_t)f[15] << 24);

    s->nano  = (int32_t)((uint32_t)f[16] | ((uint32_t)f[17] << 8) | ((uint32_t)f[18] << 16)| ((uint32_t)f[19] << 24));

    s->fixType = f[20];

    s->flags   = f[21];

    s->flags2  = f[22];

    s->numSV   = f[23];

    s->lon   = ((int32_t)((uint32_t)f[24]   | ((uint32_t)f[25] << 8) | ((uint32_t)f[26] << 16)| ((uint32_t)f[27] << 24)));

    s->lat   = (int32_t)((uint32_t)f[28]   | ((uint32_t)f[29] << 8) | ((uint32_t)f[30] << 16) | ((uint32_t)f[31] << 24));

    s->height = (int32_t)((uint32_t)f[32]  | ((uint32_t)f[33] << 8)| ((uint32_t)f[34] << 16)   | ((uint32_t)f[35] << 24));

    s->hMSL   = (int32_t)((uint32_t)f[36]  | ((uint32_t)f[37] << 8)| ((uint32_t)f[38] << 16)   | ((uint32_t)f[39] << 24));

    s->hAcc   = (uint32_t)f[40] | ((uint32_t)f[41] << 8)| ((uint32_t)f[42] << 16)   | ((uint32_t)f[43] << 24);

    s->vAcc   = (uint32_t)f[44] | ((uint32_t)f[45] << 8)| ((uint32_t)f[46] << 16)   | ((uint32_t)f[47] << 24);

    s->velN   = (int32_t)((uint32_t)f[48]  | ((uint32_t)f[49] << 8) | ((uint32_t)f[50] << 16)   | ((uint32_t)f[51] << 24));

    s->velE   = (int32_t)((uint32_t)f[52]  | ((uint32_t)f[53] << 8) | ((uint32_t)f[54] << 16)   | ((uint32_t)f[55] << 24));

    s->velD   = (int32_t)((uint32_t)f[56]  | ((uint32_t)f[57] << 8) | ((uint32_t)f[58] << 16)   | ((uint32_t)f[59] << 24));

    s->gSpeed = (int32_t)((uint32_t)f[60]  | ((uint32_t)f[61] << 8) | ((uint32_t)f[62] << 16)   | ((uint32_t)f[63] << 24));

    s->headMot= (int32_t)((uint32_t)f[64]  | ((uint32_t)f[65] << 8) | ((uint32_t)f[66] << 16)   | ((uint32_t)f[67] << 24));

    s->sAcc   = (uint32_t)f[68] | ((uint32_t)f[69] << 8) | ((uint32_t)f[70] << 16)   | ((uint32_t)f[71] << 24);

    s->headAcc= (uint32_t)f[72] | ((uint32_t)f[73] << 8) | ((uint32_t)f[74] << 16)   | ((uint32_t)f[75] << 24);

    s->pDOP   = (uint16_t)f[76] | ((uint16_t)f[77] << 8);

    // 78..87 skipped: flags3(2) + reserved0(4) + headVeh(4)
    s->magDec = (int16_t)((uint16_t)f[88] | ((uint16_t)f[89] << 8));

    s->magAcc = (uint16_t)f[90] | ((uint16_t)f[91] << 8);
}

static nav_pvt_t pvt;              // file scope, the one persistent copy

static bool gps_update(void)
{
    bool fresh = false;
    while (uart_is_readable(GPS_UART)) {
        uint8_t b = (uint8_t)uart_getc(GPS_UART);
        if (!ubx_rx_byte(b)) continue;
        decode_nav_pvt(&frame[4], &pvt);
        fresh = true;
    }
    return fresh;
}

void gps_init(void){
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);
}

// helpers for integer-only string output 
static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

int main(){
    stdio_init_all();
    multicore_launch_core1(attitude_core);
    gps_init();
    while (!stdio_usb_connected()) sleep_ms(100);
    printf("UBX NAV-PVT decoder started\r\n");

    nav_pvt_t pvt;
    imu_nav_sample batch[FIFO_MAX_BURST_SAMPLES]; // stack of 32 structs 
    char line[200];

    while (true) {
        while (uart_is_readable(GPS_UART)) {
            uint8_t b = (uint8_t)uart_getc(GPS_UART);
            if (ubx_rx_byte(b) == false) continue;      // keep feeding until a full valid PVT

            // checksum passed, class/id/len confirmed -> decode
            // frame 4 so frame[4] = f[0] in function, this is due to payload being different lenth than message 
            decode_nav_pvt(&frame[4], &pvt); 

            if ((pvt.flags & 0x01) && pvt.fixType >= 2) {
                // integer decomposition, no floats:
                //   lat/lon are deg * 1e7 -> deg and 7-digit fraction
                int32_t lat  = iabs32(pvt.lat);
                int32_t lon  = iabs32(pvt.lon);
                char ns = (pvt.lat < 0) ? 'S' : 'N';
                char ew = (pvt.lon < 0) ? 'W' : 'E';
                //   gSpeed mm/s -> 0.1 km/h
                int32_t kmh10 = (pvt.gSpeed * 36) / 1000;
                //   headMot deg * 1e-5 -> 0.1 deg
                int32_t hdg10 = pvt.headMot / 10000;
                //   hAcc mm -> m, pDOP * 0.01 -> integer part + 2-digit fraction
                uint32_t hacc_m = pvt.hAcc / 1000;

                snprintf(line, sizeof line,
                    "%02u:%02u:%02u  "
                    "Lat %ld.%07ld%c  Lon %ld.%07ld%c  "
                    "Spd %ld.%ld km/h  Hdg %ld.%ld deg  "
                    "fix=%u sats=%u hAcc=%lum pDOP=%u.%02u\r\n",
                    (unsigned)pvt.hour, (unsigned)pvt.min, (unsigned)pvt.sec,
                    (long)(lat / 10000000L), (long)(lat % 10000000L), ns,
                    (long)(lon / 10000000L), (long)(lon % 10000000L), ew,
                    (long)(kmh10 / 10), (long)(kmh10 % 10),
                    (long)(hdg10 / 10), (long)(hdg10 % 10),
                    (unsigned)pvt.fixType, (unsigned)pvt.numSV,
                    (unsigned)hacc_m,
                    (unsigned)(pvt.pDOP / 100), (unsigned)(pvt.pDOP % 100));

                printf("%s", line);
                printf("Q:%6.3f %6.3f %6.3f %6.3f | \n"
                        "A:%5.3f %5.3f %5.3f g | "
                        ,qm[0], qm[1], qm[2], qm[3],
                        acc[0], acc[1], acc[2]);


            } else {
                printf("%02u:%02u:%02u  (acquiring... fixType=%u sats=%u)\r\n",
                       (unsigned)pvt.hour, (unsigned)pvt.min, (unsigned)pvt.sec,
                       (unsigned)pvt.fixType, (unsigned)pvt.numSV);
            }
        }
        tight_loop_contents();
    }
}
