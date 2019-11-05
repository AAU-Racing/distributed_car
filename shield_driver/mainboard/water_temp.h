#ifndef WATER_TEMP_H
#define WATER_TEMP_H

#include <stm32f4xx.h>
#include <board_driver/adc.h>
#include <board_driver/gpio.h>

#define ADC_WATER_IN_PIN                PIN_4
#define ADC_WATER_IN_GPIO_PORT          GPIOC
#define ADC_WATER_IN_CHANNEL            CHANNEL_14

#define ADC_WATER_OUT_PIN               PIN_7
#define ADC_WATER_OUT_GPIO_PORT         GPIOA
#define ADC_WATER_OUT_CHANNEL           CHANNEL_7

void init_water_temp();
uint16_t read_water_in();
uint16_t read_water_out();

#endif /* WATER_TEMP_H */
