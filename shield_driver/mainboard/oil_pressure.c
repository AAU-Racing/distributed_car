#include <board_driver/gpio.h>

#include "oil_pressure.h"

bool on = false;

bool get_oil_pressure() {
	return gpio_get_state(OIL_PRESSURE_GPIO_PORT, OIL_PRESSURE_PIN);
}

void init_oil_pressure() {
	gpio_input_init(OIL_PRESSURE_GPIO_PORT, OIL_PRESSURE_PIN, GPIO_PULL_UP);
}
