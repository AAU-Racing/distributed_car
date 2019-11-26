#ifndef TLC59116_DASH_H
#define TLC59116_DASH_H

#include <stdbool.h>

void set_led_dash(unsigned led, unsigned r, unsigned g, unsigned b);
int led_driver_init_dash(bool china);

#endif
