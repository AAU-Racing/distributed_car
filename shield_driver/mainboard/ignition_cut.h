#ifndef IGNITION_CUT_H
#define IGNITION_CUT_H

#include <board_driver/gpio.h>

#define IGN_CUT_PIN 	PIN_8
#define IGN_CUT_PORT	GPIOE

void init_ignition_cut();
void ignition_cut_on();
void ignition_cut_off();

#endif
