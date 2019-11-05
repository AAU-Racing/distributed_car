#include "ignition_cut.h"


void init_ignition_cut() {
	gpio_output_init(IGN_CUT_PORT, IGN_CUT_PIN);
}

void ignition_cut_on() {
	gpio_toggle_on(IGN_CUT_PORT, IGN_CUT_PIN);
}

void ignition_cut_off() {
	gpio_toggle_off(IGN_CUT_PORT, IGN_CUT_PIN);
}
