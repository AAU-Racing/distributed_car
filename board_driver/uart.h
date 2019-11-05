#ifndef UART_H
#define UART_H

#include <stdlib.h>
#include <stdint.h>

#define MAIN_GPS_UART 0
#define MAIN_EXTERNAL_UART 1
#define MAIN_DEBUG_UART 2
#define MAIN_TELEMETRY_UART 3
#define DASH_DEBUG_UART 4
#define DEV_DEBUG_UART 5
#define DEV_TELEMETRY_UART 6
#define DEV_ARDUINO_UART 7


void uart_init(uint8_t config);
uint8_t uart_read_byte(uint8_t config);
void uart_send_byte(uint8_t config, uint8_t data);
void uart_send_buf(uint8_t config, uint8_t *data, size_t n);


#endif /* UART_H */
