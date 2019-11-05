#include "neutral.h"

void neutral_switch_init(void) {
	gpio_input_init(NEUTRAL_SWITCH_PORT, NEUTRAL_SWITCH_PIN, GPIO_NO_PULL);
}

bool neutral_switch_get_state() {
    return !gpio_get_state(NEUTRAL_SWITCH_PORT, NEUTRAL_SWITCH_PIN);
}
