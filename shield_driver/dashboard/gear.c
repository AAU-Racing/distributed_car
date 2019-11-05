#include <stm32f4xx.h>

#include <board_driver/can.h>

#include "gear.h"

static int gear_no = 0;
static int gear_is_new = false;

bool get_gear(gear_t* gear){
	*gear = gear_no;
	bool is_new_data = gear_is_new;
	gear_is_new = false;
	return is_new_data;
}

void gear_callback(CAN_RxFrame* Msg){
	gear_no = Msg->Msg[0];
	gear_is_new = true;
}

int gear_init(void){
	return can_filter(CAN_GEAR_NUMBER, 0x7ff, gear_callback);
}
