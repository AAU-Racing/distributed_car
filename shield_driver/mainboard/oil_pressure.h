#ifndef OIL_PRESSURE_H
#define OIL_PRESSURE_H

#include <stdbool.h>

#include <board_driver/gpio.h>

#define OIL_PRESSURE_PIN                    PIN_8
#define OIL_PRESSURE_GPIO_PORT              GPIOD

bool get_oil_pressure();
void init_oil_pressure();

#endif
