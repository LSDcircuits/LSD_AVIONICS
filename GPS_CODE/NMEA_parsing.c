#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART config
#define GPS_UART        uart0
#define GPS_BAUD        38400     // try 9600 if you see garbage
#define GPS_TX_PIN      0         // Pico TX -> GPS RX
#define GPS_RX_PIN      1         // Pico RX <- GPS TX

#define LINE_BUF_LEN    128
#define MAX_FIELDS      24


// XOR of chars between '$' and '*'. Returns 0 if sentence is malformed.
static bool nmea_checksum_ok(const char *s) {
    if (s[0] != '$') return false;
    uint8_t cs = 0;
    const char *p = s + 1;
    while (*p && *p != '*') cs ^= (uint8_t)*p++;
    if (*p != '*') return false;
    unsigned int sent;
    return (sscanf(p + 1, "%2X", &sent) == 1) && (sent == cs);
}

// Split a,b,, in place -> fields[] = {"a","b","","c"}. Returns field count.
static int nmea_split(char *s, char **fields, int max_fields) {
    int n = 0;
    char *p = s;
    fields[n++] = p;
    while (*p && n < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[n++] = p + 1;
        }
        p++;
    }
    return n;
}

// "4916.45" + 'N' -> +49.27417
static bool nmea_to_deg(const char *v, char hemi, bool is_lat, float *out) {
    if (!v || v[0] == '\0') return false;
    char *end;
    float raw = strtof(v, &end);
    if (end == v) return false;

    int deg_digits = is_lat ? 2 : 3;
    float deg = (float)((int)(raw / 100.0f));
    float min = raw - deg * 100.0f;
    float result = deg + min / 60.0f;

    if (hemi == 'S' || hemi == 'W') result = -result;
    *out = result;
    return true;
}

typedef struct {
    bool  fix_valid;    // GGA fix quality != 0
    bool  has_pos;      // lat/lon fields were non-empty and well-formed
    float lat;          // decimal degrees, +N / -S
    float lon;          // decimal degrees, +E / -W
} gps_pos_t;

// Handles GGA and RMC from any talker (GP, GN, GB, GA, GL)
static bool parse_sentence(char *line, gps_pos_t *pos) {
    if (!nmea_checksum_ok(line)) return false;

    // sentence ID = chars 3..5 of "$GNGGA,..." (skip talker)
    char id[4];
    if (strlen(line) < 6) return false;
    memcpy(id, line + 3, 3);
    id[3] = '\0';

    // strip "$xx" prefix so fields start at field 1
    char *body = line + 3;

    if (strcmp(id, "GGA") == 0) {
        char *f[MAX_FIELDS];
        int n = nmea_split(body, f, MAX_FIELDS);
        // GGA: $xxGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,..
        if (n < 6) return false;
        pos->fix_valid = (f[5][0] != '0' && f[5][0] != '\0');
        pos->has_pos  = nmea_to_deg(f[1], f[2][0], true,  &pos->lat)
                     && nmea_to_deg(f[3], f[4][0], false, &pos->lon);
        return true;
    }
    if (strcmp(id, "RMC") == 0) {
        char *f[MAX_FIELDS];
        int n = nmea_split(body, f, MAX_FIELDS);
        // RMC: $xxRMC,time,A/V,lat,N,lon,E,speed,course,date,...
        if (n < 6) return false;
        pos->fix_valid = (f[1][0] == 'A');
        pos->has_pos  = nmea_to_deg(f[2], f[3][0], true,  &pos->lat)
                     && nmea_to_deg(f[4], f[5][0], false, &pos->lon);
        return true;
    }
    return false; // GSV, GSA, VTG, GLL... ignored for now
}



int main() {
    stdio_init_all();

    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("RP2350 GPS parser started (UART0 @ %d baud)\r\n", GPS_BAUD);

    char line[LINE_BUF_LEN];
    uint16_t idx = 0;
    gps_pos_t pos = {0};

    while (true) {
        while (uart_is_readable(GPS_UART)) {
            char c = uart_getc(GPS_UART);

            if (c == '$') {            // new sentence: reset buffer
                idx = 0;
                line[idx++] = c;
            } else if (c == '\r' || c == '\n') {
                if (idx > 0) {
                    line[idx] = '\0';
                    if (parse_sentence(line, &pos)) {
                        if (pos.fix_valid && pos.has_pos)
                            printf("Lat: %.6f  Lon: %.6f\r\n", pos.lat, pos.lon);
                        else
                            printf("Lat: N/A  Lon: N/A\r\n");
                    }
                    idx = 0;
                }
            } else if (idx < LINE_BUF_LEN - 1) {
                line[idx++] = c;
            } else {
                idx = 0;  // overflow, drop
            }
        }
        tight_loop_contents();
    }
}
