#ifndef NEUTRAL_SWITCH_H
#define NEUTRAL_SWITCH_H

#include <board_driver/gpio.h>
#include <stdbool.h>

#define NEUTRAL_SWITCH_PIN    PIN_15
#define NEUTRAL_SWITCH_PORT   GPIOE

void neutral_switch_init(void);
bool neutral_switch_get_state();

#endif /* NEUTRAL_SWITCH_H */
