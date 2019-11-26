#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32l4xx_hal.h>

#include <board_driver/uart.h>
#include <board_driver/iwdg.h>


int main(void) {
		uart_init(DEV_DEBUG_UART);
    HAL_Delay(1000);
    start_iwdg();
    printf("Starting\r\n");

    while (1) {
		printf("Hello usb\r\n");
        HAL_Delay(500);
	}
}
