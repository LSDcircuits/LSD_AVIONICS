#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4   // Pico GP4 → M9N RX
#define UART_RX_PIN 5   // Pico GP5 ← M9N TX

typedef struct {
    float latitude;
    float longitude;
    float altitude;
    int fix_quality;   // 0 = no fix, 1 = GPS, 2 = DGPS, etc.
    int num_sats;
    uint8_t valid;     // 1 if data is fresh and valid
} geo_data_t;

static geo_data_t geo = {0};

// Convert NMEA lat/lon (ddmm.mmmm) to decimal degrees
float nmea_to_decimal(const char *nmea, const char *dir) {
    if (!nmea || strlen(nmea) < 3) return 0.0f;
    
    float degrees = 0.0f, minutes = 0.0f;
    
    if (strchr(nmea, '.') - nmea == 4) {  // Lat: ddmm.mmmm
        degrees = (nmea[0] - '0') * 10.0f + (nmea[1] - '0');
        minutes = atof(nmea + 2);
    } else {  // Lon: dddmm.mmmm
        degrees = (nmea[0] - '0') * 100.0f + (nmea[1] - '0') * 10.0f + (nmea[2] - '0');
        minutes = atof(nmea + 3);
    }
    
    float decimal = degrees + (minutes / 60.0f);
    if (*dir == 'S' || *dir == 'W') decimal = -decimal;
    return decimal;
}

// Parse $GNGGA
void parse_gga(const char *sentence) {
    char *tok[15] = {0};
    char buf[128];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    
    int i = 0;
    tok[i] = strtok(buf, ",");
    while (tok[i] && i < 14) tok[++i] = strtok(NULL, ",");
    
    if (i < 10) return;
    
    geo.fix_quality = tok[6] ? atoi(tok[6]) : 0;
    geo.num_sats    = tok[7] ? atoi(tok[7]) : 0;
    
    if (geo.fix_quality > 0 && tok[2] && tok[3] && tok[4] && tok[5]) {
        geo.latitude  = nmea_to_decimal(tok[2], tok[3]);
        geo.longitude = nmea_to_decimal(tok[4], tok[5]);
        geo.altitude  = tok[9] ? atof(tok[9]) : 0.0f;
        geo.valid = 1;
    } else {
        geo.valid = 0;
    }
}

// Verify NMEA checksum (*XX)
int checksum_ok(const char *sentence) {
    if (*sentence != '$') return 0;
    const char *star = strrchr(sentence, '*');
    if (!star || strlen(star) < 3) return 0;
    
    uint8_t calc = 0;
    for (const char *p = sentence + 1; p < star; p++) calc ^= *p;
    
    uint8_t sent = (uint8_t)strtol(star + 1, NULL, 16);
    return calc == sent;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB serial
    
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    
    printf("Pico NMEA parser started\n");
    
    char line[128];
    int idx = 0;
    
    while (1) {
        while (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            
            if (c == '$') {
                idx = 0;  // Start of new sentence
            }
            
            if (idx < sizeof(line) - 1) {
                line[idx++] = c;
            }
            
            if (c == '\n') {
                line[idx] = '\0';
                
                if (strncmp(line, "$GNGGA", 6) == 0 && checksum_ok(line)) {
                    parse_gga(line);
                    
                    if (geo.valid) {
                        printf("FIX: %.6f, %.6f | Alt: %.1fm | Sats: %d | Q: %d\n",
                               geo.latitude, geo.longitude, geo.altitude,
                               geo.num_sats, geo.fix_quality);
                    } else {
                        printf("NO FIX | Sats: %d\n", geo.num_sats);
                    }
                }
                idx = 0;
            }
        }
    }
}
