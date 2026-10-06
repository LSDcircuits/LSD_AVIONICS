#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>
#include "pico/stdio_usb.h"
#include "hardware/i2c.h"

#define I2C_PORT    i2c1
#define SDA_PIN     26
#define SCL_PIN     27

#define QMC_ADDR     0x2C
#define R_CHIPID     0x00   /* RO, must read 0x80          */
#define R_XOUT_L     0x01   /* data block 0x01..0x06       */
#define R_STATUS     0x09   /* bit1 OVFL, bit0 DRDY        */
#define R_CTL1       0x0A   /* OSR2 | OSR1 | ODR | MODE    */
#define R_CTL2       0x0B   /* SRESET|SELF_TEST|RNG|S/R    */
#define R_AXIS_SIGN  0x29   /* NOT in register map — see §7 */

/* returns bytes read, or negative pico error code on NACK/timeout */
static int reg_read(uint8_t reg, uint8_t *dst, size_t len) {
    int r = i2c_write_blocking(I2C_PORT, QMC_ADDR, &reg, 1, true);
    if (r < 0) return r;
    return i2c_read_blocking(I2C_PORT, QMC_ADDR, dst, len, false);
}

static int reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(I2C_PORT, QMC_ADDR, buf, 2, false);
}

static int16_t s16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int main(void) {
    stdio_init_all();
    while (!stdio_usb_connected()) sleep_ms(50);
    printf("boot\n");

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    /* --- presence check: CHIP ID must be 0x80 (datasheet §9.2.1) --- */
    uint8_t id = 0;
    int r = reg_read(R_CHIPID, &id, 1);
    printf("probe 0x2C: ret=%d chipid=0x%02X %s\n", r, id,
           (r >= 0 && id == 0x80) ? "OK" : "FAIL");
    if (r < 0 || id != 0x80) {
        printf("no QMC5883P answering at 0x2C — check wiring\n");
        while (1) tight_loop_contents();
    }

    /* --- §7.6 soft reset, then re-check ID --- */
    reg_write(R_CTL2, 0x80);
    sleep_ms(10);
    reg_read(R_CHIPID, &id, 1);
    printf("after soft reset chipid=0x%02X\n", id);

    /* --- §7.2 continuous-mode setup, in datasheet order --- */
    reg_write(R_AXIS_SIGN, 0x06);   /* step 1: axis sign definition   */
    reg_write(R_CTL2, 0x08);        /* step 2: S/R on, RNG = ±8G      */
    reg_write(R_CTL1, 0xCF);        /* step 3: OSR2=8, OSR1=8, ODR=200Hz,
                                                MODE=continuous (11)  */

    /* readback — proves the writes landed */
    uint8_t c1, c2;
    reg_read(R_CTL1, &c1, 1);
    reg_read(R_CTL2, &c2, 1);
    printf("CTL1=0x%02X CTL2=0x%02X (expect 0xCF 0x08)\n", c1, c2);

    while (1) {
        uint8_t st;
        if (reg_read(R_STATUS, &st, 1) >= 0 && (st & 0x01)) {
            uint8_t d[6];
            if (reg_read(R_XOUT_L, d, 6) >= 0 && !(st & 0x02)) {
                float x = s16(&d[0]) * (100.0f / 3750.0f);   /* ±8G = 3750 LSB/G */
                float y = s16(&d[2]) * (100.0f / 3750.0f);
                float z = s16(&d[4]) * (100.0f / 3750.0f);
                float h = atan2f(y, x) * 180.0f / (float)M_PI;
                if (h < 0) h += 360.0f;
                printf("B: %+7.1f %+7.1f %+7.1f uT | |B|=%5.1f | raw hdg %6.1f deg\n",
                       x, y, z, sqrtf(x*x + y*y + z*z), h);
            }
        }
        sleep_ms(20);
    }
}
