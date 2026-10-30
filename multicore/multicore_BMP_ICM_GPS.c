#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "m9n_ubx.h"
#include "bmp388.h"
#include "icm42688p.h"

//-------- attitude estimate core (high freq) ------------

icm_t icm;
void attitude_core(){
    imu_init();
    printf("Keep IMU stationary for calibration...\n");
    calibrate_gyro(2000);
    while(1){
        get_imu(&icm);
    }
}

//-------- fusion & Anchoring core (low freq) -------------
void main(){
    stdio_init_all();
    multicore_launch_core1(attitude_core);
    gps_driver_init();
    bmp_init();
    nav_pvt_t pvt;
    pt_t out;

    while (!stdio_usb_connected()) sleep_ms(100);
    printf("UBX NAV-PVT decoder started\r\n");

    get_p0(&out, 10);
    get_initial_alt(&out, 10);
    double alt = 0;

    while (1) {
        if (gps_update(&pvt)) {
            if ((pvt.flags & 0x01) && pvt.fixType >= 2) {
                get_bmp(&out);
                alt = out.bmp_z;

                printf("Lat %ld.%07ld  Lon %ld.%07ld  hMSL %ld mm  fix=%u  sats=%u\r\n",
                    (long)(pvt.lat / 10000000L), (long)(iabs32(pvt.lat) % 10000000L),
                    (long)(pvt.lon / 10000000L), (long)(iabs32(pvt.lon) % 10000000L),
                    (long)pvt.hMSL,
                    pvt.fixType, pvt.numSV);
                printf("Q:%6.3f %6.3f %6.3f %6.3f\n", icm.qm_out[0], icm.qm_out[1], icm.qm_out[2], icm.qm_out[3]);
                printf("A:%6.3f %6.3f %6.3f %6.3f\n", icm.acc_out[0], icm.acc_out[1], icm.acc_out[2]);
                printf("temp : %6.3f\npress: %6.3f\n", out.temp, out.press);
                printf("press_alt : %6.3f\n", alt);
                printf("gainkp : %6.3f gainki : %6.3f\n", icm.Kp, icm.Ki);
                sleep_ms(160);

            } else {
                get_bmp(&out);
                alt = out.bmp_z;
                printf("acquiring... fixType=%u sats=%u\r\n",
                        pvt.fixType, pvt.numSV);
                printf("temp : %6.3f\npress: %6.3f\n", out.temp, out.press);
                printf("press_alt : %6.3f\n", alt);
                printf("gainkp : %6.3f gainki : %6.3f\n", icm.Kp, icm.Ki);
                sleep_ms(160);
            }
        } else {
            tight_loop_contents();
        }
    }
}
}
