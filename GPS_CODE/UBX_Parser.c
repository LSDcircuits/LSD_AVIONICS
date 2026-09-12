#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// ---------- config ----------
#define GPS_UART        uart0
#define GPS_BAUD        38400
#define GPS_TX_PIN      0
#define GPS_RX_PIN      1
#define UBX_MAX_PAYLOAD 128

// ---------- UBX framing ----------
#define UBX_SYNC1   0xB5
#define UBX_SYNC2   0x62

#define CLASS_NAV   0x01
#define ID_NAV_PVT  0x07
#define ID_NAV_DOP  0x04
#define CLASS_ACK   0x05
#define ID_ACK_ACK  0x01
#define ID_ACK_NAK  0x00

// NAV-PVT payload (92 bytes, little-endian) --------------------------
typedef struct __attribute__((packed)) {
    uint32_t iTOW;      //  0
    uint16_t year;      //  4
    uint8_t  month, day, hour, min, sec;   // 6..10
    uint8_t  valid;     // 11
    uint32_t tAcc;      // 12
    int32_t  nano;      // 16
    uint8_t  fixType;   // 20
    uint8_t  flags;     // 21  bit0 = gnssFixOK
    uint8_t  flags2;    // 22
    uint8_t  numSV;     // 23
    int32_t  lon;       // 24  deg * 1e-7
    int32_t  lat;       // 28  deg * 1e-7
    int32_t  height;    // 32  mm
    int32_t  hMSL;      // 36  mm
    uint32_t hAcc, vAcc;// 40, 44  mm
    int32_t  velN, velE, velD, gSpeed;  // 48..60  mm/s
    int32_t  headMot;   // 64  deg * 1e-5
    uint32_t sAcc, headAcc;  // 68, 72
    uint16_t pDOP;      // 76  * 0.01
    uint8_t  flags3;    // 78
    uint8_t  reserved1[5];
    int32_t  headVeh;   // 84
    int16_t  magDec;    // 88
    uint16_t magAcc;    // 90
} ubx_nav_pvt_t;
_Static_assert(sizeof(ubx_nav_pvt_t) == 92, "bad NAV-PVT size");

// NAV-DOP payload (18 bytes) ------------------------------------------
typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP;  // all * 0.01
} ubx_nav_dop_t;
_Static_assert(sizeof(ubx_nav_dop_t) == 18, "bad NAV-DOP size");

// ---------- receiver state machine ----------
typedef enum { S_SYNC1, S_SYNC2, S_CLS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CK1, S_CK2 } rx_state_t;

static rx_state_t rx_state = S_SYNC1;
static uint8_t  rx_cls, rx_id;
static uint16_t rx_len, rx_idx;
static uint8_t  rx_buf[UBX_MAX_PAYLOAD];
static uint8_t  ck_a, ck_b;

// Feed one byte; returns true when a complete, checksum-valid frame is ready
// (class/id/length/payload in the statics above).
static bool ubx_rx_byte(uint8_t b) {
    switch (rx_state) {
    case S_SYNC1: if (b == UBX_SYNC1) rx_state = S_SYNC2; break;
    case S_SYNC2: rx_state = (b == UBX_SYNC2) ? S_CLS : S_SYNC1; break;
    case S_CLS:  rx_cls = b; ck_a = b; ck_b = ck_a; rx_state = S_ID;   break;
    case S_ID:   rx_id = b;  ck_a += b; ck_b += ck_a; rx_state = S_LEN1; break;
    case S_LEN1: rx_len = b; ck_a += b; ck_b += ck_a; rx_state = S_LEN2; break;
    case S_LEN2:
        rx_len |= (uint16_t)b << 8; ck_a += b; ck_b += ck_a; rx_idx = 0;
        rx_state = (rx_len > 0 && rx_len <= UBX_MAX_PAYLOAD) ? S_PAYLOAD : S_SYNC1;
        break;
    case S_PAYLOAD:
        rx_buf[rx_idx++] = b; ck_a += b; ck_b += ck_a;
        if (rx_idx == rx_len) rx_state = S_CK1;
        break;
    case S_CK1: rx_state = (b == ck_a) ? S_CK2 : S_SYNC1; break;
    case S_CK2:
        rx_state = S_SYNC1;
        return (b == ck_b);
    }
    return false;
}

// ---------- main ----------
int main() {
    stdio_init_all();

    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);

    while (!stdio_usb_connected()) sleep_ms(100);
    printf("UBX decoder started\r\n");

    ubx_nav_pvt_t pvt;
    ubx_nav_dop_t dop;
    bool have_dop = false;

    while (true) {
        while (uart_is_readable(GPS_UART)) {
            uint8_t b = (uint8_t)uart_getc(GPS_UART);
            if (!ubx_rx_byte(b)) continue;

            if (rx_cls == CLASS_NAV && rx_id == ID_NAV_PVT && rx_len == sizeof(ubx_nav_pvt_t)) {
                memcpy(&pvt, rx_buf, sizeof pvt);
                bool ok = (pvt.flags & 0x01) && (pvt.fixType >= 2);   // gnssFixOK

                if (ok) {
                    printf("Lat: %9.5f  Lon: %9.5f  ", pvt.lat * 1e-7, pvt.lon * 1e-7);
                    printf("Speed: %5.1f km/h  Heading: %6.1f deg  ",
                           pvt.gSpeed * 0.0036f,          // mm/s -> km/h
                           pvt.headMot * 1e-5f);          // -> degrees
                    printf("fix=%u sats=%u hAcc=%.1fm", pvt.fixType, pvt.numSV, pvt.hAcc * 1e-3f);
                    if (have_dop) printf("  pDOP=%.2f", dop.pDOP * 0.01f);
                    printf("\r\n");
                } else {
                    printf("Lat: N/A  Lon: N/A  (fixType=%u sats=%u - acquiring)\r\n",
                           pvt.fixType, pvt.numSV);
                }
            }
            else if (rx_cls == CLASS_NAV && rx_id == ID_NAV_DOP && rx_len == sizeof(ubx_nav_dop_t)) {
                memcpy(&dop, rx_buf, sizeof dop);
                have_dop = true;
            }
        }
        tight_loop_contents();
    }
}
