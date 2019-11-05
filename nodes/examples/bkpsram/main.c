#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>

#include <board_driver/uart.h>
#include <board_driver/bkpsram.h>

int main(void) {

	uart_init();
	printf("uart init complete\n");
	HAL_Delay(1);

	init_bkpsram();
	printf("bkpsram init complete\n");

	HAL_Delay(1);

	if (bkpsram_was_enabled_last_run()) {
		printf("bkpsram was enabled last run\n");
		HAL_Delay(1);

		while (1) {
			printf("Value: %u\n", (unsigned int) read_bkpsram(8));
			HAL_Delay(1000);
		}
	}
	else {
		printf("bkpsram was not enabled last run\n");
		HAL_Delay(1);

		write_bkpsram(8, 12);
		printf("Write complete\n");
	}
}
