#include <stdbool.h>
#include <string.h>

#include <shield_driver/com_node/xbee.h>
#include <shield_driver/com_node/water_temperature.h>
#include <shield_driver/dashboard/oil.h>
#include <board_driver/obdii.h>

#include "sd.h"

static bool xbee_initialized = false;

void init_telemtry() {
	if (init_xbee() != HAL_OK) {
		// log_error(CAN_TELEMETRY_ERROR, "Xbee init error");
	}
	else {
		xbee_initialized = true;
	}
}

void handle_telemetry() {
	// Only try to transmit if xbee is initialized
	if (!xbee_initialized) {
		return;
	}

	// Enforce transmission limit to a max of 10 msg/s
	static uint8_t lastTransmit = 0;

	if (HAL_GetTick() - lastTransmit < 100) {
		return;
	}

	lastTransmit = HAL_GetTick();

	// Read data
	char buffer[100];
	memset(buffer, '\0', 100);

	uint16_t water_in;
	uint16_t water_out;
	int delta = water_out - water_in;

	get_water_temp_in(&water_in);
	get_water_temp_out(&water_out);


	// Resistance = 10 k * ADC_VALUE / (MAX_ADC_VALUE - ADC_VALUE)
	// Temperature = -27.271952718 * ln(R) + 240.1851825535 which is an exponential fit according to the datasheet of BOSCH 0 280 130 026.
	// float water_in_fl = -27.271952718 * ln(10000 * water_in / (4095.0 - water_in)) + 240.1851825535;
	// float water_out_fl = -27.271952718 * ln(10000 * water_out / (4095.0 - water_out)) + 240.1851825535;
	// float delta_fl = water_in_fl - water_out_fl;
	OBDII_Mode1_Frame frame = obdii_mode1_response(ControlModuleVoltage);
	float battery_voltage_fl = (frame.Msg[0] * 256 + frame.Msg[1]) / 1000; // By OBDII standard for control module voltage
	frame = obdii_mode1_response(EngineRPM);
	float rpm = (frame.Msg[0] * 256 + frame.Msg[1]) / 4;
	frame = obdii_mode1_response(EngineRPM);
	float throttle = frame.Msg[0] * 100 / 255;
	frame = obdii_mode1_response(OxygenSensorFARatio);
	float lambda = (frame.Msg[0] * 256 + frame.Msg[1]) * 2 / 65536;
	bool oil = 0;
	oilPressure_OK(&oil);

	// Print to string
	sprintf(buffer, "A1;A:%u;B:%u;C:%d;D:%u.%u;E:%u.%u;F:%u.%u;G:%u.%u;H:%u;B2\r\n", water_in, water_out, delta,
		(unsigned) battery_voltage_fl, (unsigned)((battery_voltage_fl - (unsigned) battery_voltage_fl) * 100), // This is a workaround because %f didn't work in printf
		(unsigned) rpm, (unsigned)((rpm - (unsigned) rpm) * 100),
		(unsigned) throttle, (unsigned)((throttle - (unsigned) throttle) * 100),
		(unsigned) lambda, (unsigned)((lambda - (unsigned) lambda) * 100),
		(unsigned) oil);

	// Transmit string
	xbee_transmit((uint8_t*) buffer, strlen(buffer));
}
