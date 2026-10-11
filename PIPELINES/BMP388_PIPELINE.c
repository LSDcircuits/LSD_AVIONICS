#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP_ADDR      0x76
#define REG_CHIP_ID   0x00
#define REG_DATA      0x04
#define REG_PWR_CTRL  0x1B
#define REG_OSR       0x1C
#define REG_CALIB     0x31

typedef struct {
    double T1, T2, T3;
    double P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
    double t_lin;
} bmp_calib_t;

static void bmp_rd(uint8_t reg, uint8_t *buf, size_t n) {
    i2c_write_blocking(i2c1, BMP_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, BMP_ADDR, buf, n, false);
}

static void bmp_wr(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c1, BMP_ADDR, buf, 2, false);
}

static bool bmp_init(bmp_calib_t *c) {
    uint8_t id;
    bmp_rd(REG_CHIP_ID, &id, 1);
    if (id != 0x50) return false;

    uint8_t nvm[21];
    bmp_rd(REG_CALIB, nvm, 21);

    // Datasheet 9.1: NVM -> PAR_* (16-bit values: LOW byte at lower address)
    c->T1  = (double)((nvm[1]  << 8) | nvm[0])  * 256.0;                          // /2^-8
    c->T2  = (double)((nvm[3]  << 8) | nvm[2])  / 1073741824.0;                   // /2^30
    c->T3  = (double)(int8_t)nvm[4]              / 281474976710656.0;             // /2^48
    c->P1  = ((double)(int16_t)((nvm[6]  << 8) | nvm[5])  - 16384.0) / 1048576.0; // (NVM-2^14)/2^20
    c->P2  = ((double)(int16_t)((nvm[8]  << 8) | nvm[7])  - 16384.0) / 536870912.0;// (NVM-2^14)/2^29
    c->P3  = (double)(int8_t)nvm[9]              / 4294967296.0;                  // /2^32
    c->P4  = (double)(int8_t)nvm[10]             / 137438953472.0;                // /2^37
    c->P5  = (double)((nvm[12] << 8) | nvm[11]) * 8.0;                            // /2^-3
    c->P6  = (double)((nvm[14] << 8) | nvm[13]) / 64.0;                           // /2^6
    c->P7  = (double)(int8_t)nvm[15]             / 256.0;                         // /2^8
    c->P8  = (double)(int8_t)nvm[16]             / 32768.0;                       // /2^15
    c->P9  = (double)(int16_t)((nvm[18] << 8) | nvm[17]) / 281474976710656.0;     // /2^48
    c->P10 = (double)(int8_t)nvm[19]             / 281474976710656.0;             // /2^48
    c->P11 = (double)(int8_t)nvm[20]             / 36893488147419103232.0;        // /2^65

    bmp_wr(REG_OSR, 0x00);
    bmp_wr(REG_PWR_CTRL, 0x33);
    return true;
}

static void bmp_read(bmp_calib_t *c, double *temp_c, double *press_pa) {
    uint8_t d[6];
    bmp_rd(REG_DATA, d, 6);
    uint32_t up = ((uint32_t)d[2] << 16) | (d[1] << 8) | d[0];
    uint32_t ut = ((uint32_t)d[5] << 16) | (d[4] << 8) | d[3];

    // Datasheet 9.2 Temperature
    double pd1  = (double)ut - c->T1;
    double t_lin = pd1 * c->T2 + pd1 * pd1 * c->T3;
    c->t_lin = t_lin;
    *temp_c = t_lin;

    // Datasheet 9.3 Pressure
    double t2 = t_lin * t_lin, t3 = t2 * t_lin;
    double o1 = c->P5 + c->P6*t_lin + c->P7*t2 + c->P8*t3;
    double o2 = (double)up * (c->P1 + c->P2*t_lin + c->P3*t2 + c->P4*t3);
    double up2 = (double)up * up;
    double o4 = up2 * (c->P9 + c->P10*t_lin) + up2 * (double)up * c->P11;
    *press_pa = o1 + o2 + o4;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(26, GPIO_FUNC_I2C);
    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_pull_up(26);
    gpio_pull_up(27);

    static bmp_calib_t cal;
    if (!bmp_init(&cal)) {
        printf("BMP388 not found!\n");
        while (1) tight_loop_contents();
    }

    while (1) {
        double t, p;
        bmp_read(&cal, &t, &p);
        double alt = 44330.0 * (1.0 - pow(p / 101325.0, 0.1903));
        printf("T: %.2f C  P: %.1f Pa  alt: %.1f m\n", t, p, alt);
        sleep_ms(500);
    }
}
