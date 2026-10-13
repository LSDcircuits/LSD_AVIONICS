#ifndef ICM42688P_H
#define ICM42688P_H

typedef struct {
    float q_out[4];
    float qm_out[4];
    float acc_out[3];
} icm_t;

bool imu_init(void);
bool calibrate_gyro(uint16_t samples);
bool test_imu(void);
void get_imu(icm_t *out);

#endif
