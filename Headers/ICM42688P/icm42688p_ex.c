#include <stdio.h>
#include "pico/stdlib.h"
#include "icm42688p.h"

// Main
int main(){
    stdio_init_all();
    sleep_ms(200);
    imu_init();
    printf("Keep IMU stationary for calibration...\n");
    calibrate_gyro(2000);
    icm_t icm;
    while(1){
        get_imu(&icm);
        sleep_ms(10);
        printf("Q:%6.3f %6.3f %6.3f %6.3f\n", icm.qm_out[0], icm.qm_out[1], icm.qm_out[2], icm.qm_out[3]);
        printf("A:%6.3f %6.3f %6.3f %6.3f\n", icm.acc_out[0], icm.acc_out[1], icm.acc_out[2]);
    }
}
