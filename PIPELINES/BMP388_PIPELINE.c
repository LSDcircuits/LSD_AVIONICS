#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define BMP_ADDR      0x76   // SDO low; use 0x77 if SDO tied high
#define REG_CHIP_ID   0x00
#define REG_DATA      0x04   // 6 bytes: press[3], temp[3]
#define REG_PWR_CTRL  0x1B
#define REG_OSR       0x1C
#define REG_CALIB     0x31   // 21 bytes of NVM calibration

typedef struct {
    float T1, T2, T3;
    float P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
    float t_lin;
} bmp_calib_t;

static void bmp_rd(uint8_t reg, uint8_t *buf, size_t n) {
    i2c_write_blocking(i2c0, BMP_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, BMP_ADDR, buf, n, false);
}

static void bmp_wr(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    i2c_write_blocking(i2c0, BMP_ADDR, buf, 2, false);
}

static bool bmp_init(bmp_calib_t *c) {
    uint8_t id;
    bmp_rd(REG_CHIP_ID, &id, 1);
    if (id != 0x50) return false;

    uint8_t nvm[21];
    bmp_rd(REG_CALIB, nvm, 21);

    uint16_t u16;
    int16_t  s16;
    u16 = (nvm[0]  << 8) | nvm[1];  c->T1  = (float)u16 * 256.0f;
    u16 = (nvm[2]  << 8) | nvm[3];  c->T2  = (float)u16 * 131072.0f;
    c->T3  = (float)(int8_t)nvm[4];
    s16 = (nvm[5]  << 8) | nvm[6];  c->P1  = (float)s16;
    s16 = (nvm[7]  << 8) | nvm[8];  c->P2  = (float)s16;
    c->P3  = (float)(int8_t)nvm[9];
    c->P4  = (float)(int8_t)nvm[10];
    u16 = (nvm[11] << 8) | nvm[12]; c->P5  = (float)u16 * 16.0f;
    u16 = (nvm[13] << 8) | nvm[14]; c->P6  = (float)u16 * 16.0f;
    c->P7  = (float)(int8_t)nvm[15];
    c->P8  = (float)(int8_t)nvm[16];
    s16 = (nvm[17] << 8) | nvm[18]; c->P9  = (float)s16;
    c->P10 = (float)(int8_t)nvm[19];
    c->P11 = (float)(int8_t)nvm[20];

    bmp_wr(REG_OSR, 0x00);       // no oversampling
    bmp_wr(REG_PWR_CTRL, 0x33);  // press+temp enabled, normal mode
    return true;
}

static void bmp_read(bmp_calib_t *c, float *temp_c, float *press_pa) {
    uint8_t d[6];
    bmp_rd(REG_DATA, d, 6);
    uint32_t up = ((uint32_t)d[2] << 16) | (d[1] << 8) | d[0];
    uint32_t ut = ((uint32_t)d[5] << 16) | (d[4] << 8) | d[3];

    // Temperature (Bosch float compensation)
    float t = ((float)ut - c->T1) * (c->T2 / 131072.0f);
    c->t_lin = t;
    *temp_c = t + c->T3 / 16.0f;

    // Pressure
    float o1 = c->P5 + c->P6 * t + c->P7 * t * t + c->P8 * t * t * t;
    float o2 = (float)up * (c->P1 + c->P2 * t + c->P3 * t * t + c->P4 * t * t * t);
    float up2 = (float)up * (float)up;
    float o4 = up2 * (c->P9 + c->P10 * t) + up2 * (float)up * c->P11;
    *press_pa = o1 + o2 + o4;
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // let USB serial attach

    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C); // SDA
    gpio_set_function(5, GPIO_FUNC_I2C); // SCL
    gpio_pull_up(4);
    gpio_pull_up(5);

    static bmp_calib_t cal;
    if (!bmp_init(&cal)) {
        printf("BMP388 not found!\n");
        while (1) tight_loop_contents();
    }

    while (1) {
        float t, p;
        bmp_read(&cal, &t, &p);
        float alt = 44330.0f * (1.0f - powf(p / 101325.0f, 0.1903f));
        printf("T: %.2f C  P: %.1f Pa  alt: %.1f m\n", t, p, alt);
        sleep_ms(500);
    }
}
