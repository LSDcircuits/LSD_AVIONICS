/* Purpose of this code is to print M9N GPS data to serial throgh pico insatead of Uart debugger,
next execution is to update a struct per cycle */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// UART config
#define GPS_UART        uart0
#define GPS_BAUD        38400     // try 9600 if you see garbage
#define GPS_TX_PIN      0         // Pico TX -> GPS RX
#define GPS_RX_PIN      1         // Pico RX <- GPS TX

// Simple line buffer for reassembling NMEA sentences
#define LINE_BUF_LEN    128

int main() {
    // USB serial for PC output
    stdio_init_all();

    // GPS UART
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);
    uart_set_hw_flow(GPS_UART, false, false);

    // Give the PC's serial terminal a moment to attach
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("RP2350 GPS reader started (GPS UART0 @ %d baud)\r\n", GPS_BAUD);

    char line[LINE_BUF_LEN];
    uint16_t idx = 0;

    while (true) {
        while (uart_is_readable(GPS_UART)) {
            char c = uart_getc(GPS_UART);

            if (c == '\r' || c == '\n') {
                if (idx > 0) {
                    line[idx] = '\0';
                    printf("%s\r\n", line);
                    idx = 0;
                }
            } else if (idx < LINE_BUF_LEN - 1) {
                line[idx++] = c;
            } else {
                idx = 0;  // overflow, drop the line
            }
        }
        // No tight spinning; yield to USB stack
        tight_loop_contents();
    }
}
