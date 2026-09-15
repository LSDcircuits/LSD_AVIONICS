#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// config 
#define GPS_UART        uart0
#define GPS_BAUD        38400
#define GPS_TX_PIN      0          // Pico TX -> GPS RX
#define GPS_RX_PIN      1          // Pico RX <- GPS TX

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

    s->lon   = (int32_t)((uint32_t)f[24]   | ((uint32_t)f[25] << 8) | ((uint32_t)f[26] << 16)| ((uint32_t)f[27] << 24));

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

// helpers for integer-only string output 
static int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

int main(void) {
    stdio_init_all();
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);

    while (!stdio_usb_connected()) sleep_ms(100);
    printf("UBX NAV-PVT decoder started\r\n");

    nav_pvt_t pvt;
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
            } else {
                printf("%02u:%02u:%02u  (acquiring... fixType=%u sats=%u)\r\n",
                       (unsigned)pvt.hour, (unsigned)pvt.min, (unsigned)pvt.sec,
                       (unsigned)pvt.fixType, (unsigned)pvt.numSV);
            }
        }
        tight_loop_contents();
    }
}
