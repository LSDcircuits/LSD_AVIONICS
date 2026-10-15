#include <stdio.h>
#include "bmp388.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);
    bmp_init();
    pt_t out;
    while (1) {
        get_pt(&out);
        printf("temp : %6.3f\npress: %6.3f\n\n", out.temp, out.press);
        sleep_ms(160);          // match ODR 0x05
    }
}
