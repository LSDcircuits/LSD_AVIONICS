#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "m9n_ubx.h"

int main() {
    stdio_init_all();
    gps_driver_init();

    nav_pvt_t pvt;
    while (!stdio_usb_connected()) sleep_ms(100);
    printf("UBX NAV-PVT decoder started\r\n");

    while (1) {
        if (gps_update(&pvt)) {
            if ((pvt.flags & 0x01) && pvt.fixType >= 2) {
                printf("Lat %ld.%07ld  Lon %ld.%07ld  height %ld.%07ld fix=%u sats=%u\r\n",
                       (long)(pvt.lat / 10000000L), (long)(iabs32(pvt.lat) % 10000000L),
                       (long)(pvt.lon / 10000000L), (long)(iabs32(pvt.lon) % 10000000L),
                       pvt.fixType, pvt.numSV);
            } else {
                printf("acquiring... fixType=%u sats=%u\r\n",
                       pvt.fixType, pvt.numSV);
            }
        } else {
            tight_loop_contents();
        }
    }
}
