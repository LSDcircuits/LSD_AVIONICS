#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// ---------- config ----------
#define GPS_UART    uart0
#define GPS_BAUD    38400     // try 9600 if you see garbage
#define GPS_TX_PIN  0         // Pico TX -> GPS RX
#define GPS_RX_PIN  1         // Pico RX <- GPS TX

#define LINE_BUF    100
#define MAX_FIELDS  16

// ---------- tiny NMEA float parser (ddmm.mmm or ddd.dd) ----------
static bool nmea_flt(const char *s, float *out) {
    if (!s || s[0] == '\0') return false;
    bool neg = (*s == '-');
    if (neg) s++;
    long ip = 0;
    while (*s >= '0' && *s <= '9') ip = ip * 10 + (*s++ - '0');
    float val = (float)ip;
    if (*s == '.') {
        s++;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') {
            val += (float)(*s++ - '0') * scale;
            scale *= 0.1f;
        }
    }
    if (*s != '\0') return false;   // junk in the field
    *out = neg ? -val : val;
    return true;
}

// "4916.45" + 'N' -> +49.27417
static bool nmea_deg(const char *v, char hemi, bool is_lat, float *out) {
    float raw;
    if (!nmea_flt(v, &raw)) return false;
    int deg = (int)(raw / 100.0f);
    float m   = raw - (float)deg * 100.0f;
    float r   = (float)deg + m / 60.0f;
    if (hemi == 'S' || hemi == 'W') r = -r;
    *out = r;
    return true;
}

static bool nmea_cksum_ok(const char *s) {
    if (s[0] != '$') return false;
    uint8_t cs = 0;
    const char *p = s + 1;
    while (*p && *p != '*') cs ^= (uint8_t)*p++;
    if (*p != '*') return false;
    unsigned int got;
    return (sscanf(p + 1, "%2X", &got) == 1) && (got == cs);
}

// ---------- RMC parse ----------
// $GNRMC,time,status,lat,N,lon,E,speed_knots,course_deg,date,...
typedef struct {
    bool  valid;
    bool  has_pos;
    float lat, lon;      // decimal degrees
    bool  has_spd;
    float speed_kmh;     // km/h
    bool  has_cog;
    float course;        // heading over ground, degrees true
} nav_t;

static bool parse_rmc(char *line, nav_t *n) {
    if (!nmea_cksum_ok(line)) return false;
    if (strncmp(line + 3, "RMC", 3) != 0) return false;

    char *f[MAX_FIELDS];
    int cnt = 0;
    char *p = line + 7;                    // skip "$xxRMC,"
    f[cnt++] = p;
    while (*p && cnt < MAX_FIELDS) {
        if (*p == ',') { *p = '\0'; f[cnt++] = p + 1; }
        p++;
    }
    if (cnt < 9) return false;

    n->valid   = (f[1][0] == 'A');         // A=valid, V=void
    n->has_pos = nmea_deg(f[2], f[3][0], true,  &n->lat)
              && nmea_deg(f[4], f[5][0], false, &n->lon);

    float kts, cog;
    n->has_spd = nmea_flt(f[6], &kts);
    n->has_cog = nmea_flt(f[7], &cog);
    n->speed_kmh = kts * 1.852f;
    n->course    = cog;
    return true;
}

// ---------- main ----------
int main() {
    stdio_init_all();

    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);

    while (!stdio_usb_connected()) sleep_ms(100);
    printf("GPS nav parser started\r\n");

    char line[LINE_BUF];
    int idx = 0;
    nav_t nav = {0};

    while (true) {
        while (uart_is_readable(GPS_UART)) {
            char c = uart_getc(GPS_UART);

            if (c == '$') {
                idx = 0; line[idx++] = c;
            } else if (c == '\r' || c == '\n') {
                if (idx > 0) {
                    line[idx] = '\0';
                    if (parse_rmc(line, &nav)) {
                        if (nav.valid && nav.has_pos)
                            printf("Lat: %9.5f  Lon: %9.5f  ", nav.lat, nav.lon);
                        else
                            printf("Lat: N/A        Lon: N/A        ");

                        if (nav.valid && nav.has_spd)
                            printf("Speed: %5.1f km/h  ", nav.speed_kmh);
                        else
                            printf("Speed: N/A          ");

                        if (nav.valid && nav.has_cog)
                            printf("Heading: %5.1f deg\r\n", nav.course);
                        else
                            printf("Heading: N/A\r\n");
                    }
                    idx = 0;
                }
            } else if (idx < LINE_BUF - 1) {
                line[idx++] = c;
            } else {
                idx = 0;   // overflow, drop
            }
        }
        tight_loop_contents();
    }
}
