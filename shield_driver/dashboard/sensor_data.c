#include <math.h>
#include <stm32f4xx.h>

#include <board_driver/can.h>

#include "sensor_data.h"

static int neutral_state = 0;
static bool neutral_is_new = false;

static float water_in = 0;
static bool water_in_is_new = false;

static float water_out = 0;
static bool water_out_is_new = false;

bool get_neutral(bool* neutral) {
	*neutral = neutral_state;
	bool is_new_data = neutral_is_new;
	neutral_is_new = false;
	return is_new_data;
}

bool get_water_in(float* value) {
	*value = water_in;
	bool is_new_data = water_in_is_new;
	water_in_is_new = false;
	return is_new_data;
}

bool get_water_out(float* value) {
	*value = water_out;
	bool is_new_data = water_out_is_new;
	water_out_is_new = false;
	return is_new_data;
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

void sensor_data_callback(CAN_RxFrame* msg) {
	if (msg->StdId == CAN_NEUTRAL_SWITCH) {
		neutral_state = msg->Msg[0];
		neutral_is_new = true;
	}
	else if (msg->StdId == CAN_WATER_TEMPERATURE_IN) {
		uint16_t value = msg->Msg[0] << 8 | msg->Msg[1];
		water_in = convert_water_temp(value);
		water_in_is_new = true;
	}
	else if (msg->StdId == CAN_WATER_TEMPERATURE_OUT) {
		uint16_t value = msg->Msg[0] << 8 | msg->Msg[1];
		water_out = convert_water_temp(value);
		water_out_is_new = true;
	}
}

int sensor_data_init(void){
	return can_filter(CAN_SENSOR_DATA_ID_START, CAN_SENSOR_DATA_MASK, sensor_data_callback);
}
