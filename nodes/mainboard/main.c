#include <stm32f4xx_hal.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <board_driver/uart.h>
#include <board_driver/can.h>
#include <board_driver/obdii.h>
#include <board_driver/adc.h>
#include <board_driver/iwdg.h>
#include <board_driver/float_print.h>

#include <shield_driver/mainboard/gear.h>
#include <shield_driver/mainboard/oil_pressure.h>
#include <shield_driver/mainboard/neutral.h>
#include <shield_driver/mainboard/water_temp.h>

#define DISABLE_ELECTRONIC_GEAR

void setup(void);
void loop(void);

static uint32_t last_oil_transmit = 0;
static uint32_t last_neutral_transmit = 0;
static uint32_t last_water_transmit = 0;

// A main function in the philosophy of Arduino (setup + loop)
int main(void) {
	setup();

	while(1){
		loop();
	}

	return 0;
}

void setup(void){
	uart_init();
	printf("\n\n");
	printf("UART init complete\n");

	init_oil_pressure();
	printf("Oil pressure init complete\n");

	if (get_oil_pressure()) {
		printf("Oil pressure is now high\n");
	}
	else {
		printf("Oil pressure is now low\n");
	}

	neutral_switch_init();
	printf("Neutral switch init complete\n");

	init_adc();
	printf("ADC init complete\n");

	init_water_temp();
	printf("Water sensor init complete\n");

	start_adc();
	printf("ADC started\n");

	//Setting up CAN and it's filters
	if (can_init(CAN_PD0) != CAN_OK) {
		printf("Error initializing CAN\n");
	}
	else {
		printf("CAN init complete\n");
	}

#ifndef DISABLE_ELECTRONIC_GEAR
	init_gear();
	printf("Init gear complete\n");
#endif

	obdii_init();
	printf("OBDII init complete\n");

	if (can_start() != CAN_OK) {
		printf("Error starting CAN\n");
	}
	else {
		printf("CAN started\n");
	}

	// Wait for everything to be ready
	HAL_Delay(25);

#ifndef DISABLE_ELECTRONIC_GEAR
	read_initial_gear();
	printf("Initial gear is %d\n", gear_number());
#endif

	start_iwdg();
	printf("Independent watchdog started\n");

	// Signal all others that the main board is ready
	can_transmit(CAN_MAIN_BOARD_STARTED, (uint8_t[]) { 1 }, 1);
}

static float convert_water_temp(uint16_t v_out) {
	float v_in		  = 4095;  // Max ADC value = 3.3V
	float r_1  		  = 10000; // R_1 = 10kOhm
	float r_2 		  = (v_out * r_1) / (v_in - v_out);
	 // http://www.bosch-motorsport.com/media/catalog_resources/Temperature_Sensor_NTC_M12_Datasheet_51_en_2782569739pdf.pdf
	 // Also check sync
	float temperature = -31.03 * log(r_2) + 262.55;
	return temperature;
}

void loop(void){
	obdii_request_next();

	if (HAL_GetTick() - last_oil_transmit > 100) {
		bool oil_pressure = get_oil_pressure();
		can_transmit(CAN_OIL_PRESSURE, (uint8_t[]) { oil_pressure }, 1);
		last_oil_transmit = HAL_GetTick();
	}

	if (HAL_GetTick() - last_neutral_transmit > 100) {
		bool neutral = neutral_switch_get_state();
		can_transmit(CAN_NEUTRAL_SWITCH, (uint8_t[]) { neutral }, 1);
		last_neutral_transmit = HAL_GetTick();
	}

	if (HAL_GetTick() - last_water_transmit > 1000) {
		uint16_t water_temp = read_water_in();
		uint8_t water_temp_lsb = water_temp & 0xFF;
		uint8_t water_temp_msb = (water_temp >> 8) & 0xFF;
		can_transmit(CAN_WATER_TEMPERATURE_IN, (uint8_t[]) { water_temp_msb, water_temp_lsb }, 2);
		last_water_transmit = HAL_GetTick();

		float temperature = convert_water_temp(water_temp);
		printf("Current coolant temperature: ");
		print_double(temperature, 6, 2);
		printf("\n");
	}

#ifndef DISABLE_ELECTRONIC_GEAR
	change_gear();
#endif

	reset_iwdg();
}
