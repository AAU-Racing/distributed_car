#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>

#include <board_driver/uart.h>
#include <board_driver/can.h>
#include <shield_driver/devboard/TLC59116.h>

int main(void) {
	uart_init();
	printf("UART init complete\n");
	HAL_Delay(100);

	led_driver_init_dev(true);
	printf("i2c init complete\n");

	for (uint8_t i = 0; ; i++) {
	 	set_led_dev(1, 0xF, 0xF, 0xF);
		printf("Transmit complete\n");

		HAL_Delay(100000);
	}
}
