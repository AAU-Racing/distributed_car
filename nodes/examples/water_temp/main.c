#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <stm32f4xx_hal.h>

#include <board_driver/uart.h>
#include <board_driver/adc.h>
#include <board_driver/float_print.h>
#include <shield_driver/mainboard/water_temp.h>

static float convert_water_temp(uint16_t v_out) {
	float v_in		  = 4095;  // Max ADC value = 3.3V
	float r_1  		  = 10000; // R_1 = 10kOhm
	float r_2 		  = (v_out * r_1) / (v_in - v_out);
	 // http://www.bosch-motorsport.com/media/catalog_resources/Temperature_Sensor_NTC_M12_Datasheet_51_en_2782569739pdf.pdf
	 // Also check sync
	float temperature = -31.03 * log(r_2) + 262.55;
	return temperature;
}

int main(void) {
	uart_init();
	printf("uart init complete\n");

	HAL_Delay(100);

	init_adc();
	printf("adc init complete\n");

	init_water_temp();
	printf("water sensor init complete\n");

	HAL_Delay(10);

	start_adc();
	printf("adc started\n");
	HAL_Delay(10);

	for (uint16_t i = 0; ; i++) {
		int in = read_water_in();
		int out = read_water_out();

	    float water_in = convert_water_temp(in);
	    float water_out = convert_water_temp(out);
		(void) water_in;
		(void) water_out;

	    printf("%5d, %4d, %4d, ", i, in, out);
	    // print_double(water_in, 5, 2);
	    // printf(", ");
	    // print_double(water_out, 5, 2);
	    printf("\n");

		HAL_Delay(2);
	}
}
