#include <stm32f4xx_hal.h>

#include <stdbool.h>

#include "oil.h"

#include <board_driver/can.h>

static bool oil_pressure_is_ok = false;

bool oil_pressure_ok(){
	return oil_pressure_is_ok;
}

void oil_pressure_callback(CAN_RxFrame* Msg){
	oil_pressure_is_ok = Msg->Msg[0];
}

HAL_StatusTypeDef oil_init(void){
	return can_filter(CAN_OIL_PRESSURE, 0x7FF, oil_pressure_callback);
}
