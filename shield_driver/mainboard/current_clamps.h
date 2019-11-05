#ifndef CURRENT_CLAMPS_H
#define CURRENT_CLAMPS_H

#include <stm32f4xx.h>
#include <board_driver/adc.h>
#include <board_driver/gpio.h>

#define ADC_CURRENT1_PIN                PIN_0
#define ADC_CURRENT1_GPIO_PORT          GPIOB
#define ADC_CURRENT1_CHANNEL            CHANNEL_8

#define ADC_CURRENT2_PIN                PIN_1
#define ADC_CURRENT2_GPIO_PORT          GPIOB
#define ADC_CURRENT2_CHANNEL            CHANNEL_9

void init_current_clamps();
uint16_t read_battery_current();
uint16_t read_inverter_current();

#endif /* CURRENT_CLAMPS_H */
