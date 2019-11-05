#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <stm32f4xx_hal.h>

#include <board_driver/uart.h>
#include <board_driver/adc.h>
#include <shield_driver/devboard/potmeter.h>

int main(void) {
	uart_init();
	printf("uart init complete\n");

	HAL_Delay(100);

	init_adc();
	printf("adc init complete\n");

	init_potmeter_input();
	printf("potmeter init complete\n");

	HAL_Delay(10);

	start_adc();
	printf("adc started\n");
	HAL_Delay(10);

	for (uint16_t i = 0; ; i++) {
		int in1 = read_potmeter(0);
		int in2 = read_potmeter(1);
		int in3 = read_potmeter(2);
		int in4 = read_potmeter(3);

	    printf("%5d, %4d, %4d, %4d, %4d\n", i, in1, in2, in3, in4);

		HAL_Delay(2);
	}
}
