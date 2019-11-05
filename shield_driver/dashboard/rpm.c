#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

#include "rpm.h"

#include <board_driver/obdii.h>

#define MAX_LEVEL 45

int get_rpm_level(bool *new) {
	uint16_t rpm;
	*new = get_rpm(&rpm);

	if(rpm < RPM_CRITICAL){
		int rpm_level = MAX_LEVEL - ((RPM_CRITICAL - rpm) / RPM_LEVEL_RANGE) - 1;
		if (rpm_level <= MAX_LEVEL) {
			return rpm_level;
		}
		else {
			return 0;
		}
	}
	else {
		return MAX_LEVEL;
	}
}

bool get_rpm(uint16_t* rpm){
	OBDII_Mode1_Frame obd_rpm = obdii_mode1_response(EngineRPM);
	*rpm = ((256 * obd_rpm.Msg[0]) + obd_rpm.Msg[1]) / 4;
	return obd_rpm.New;
}
