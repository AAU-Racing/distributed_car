#ifndef GEAR_FEEDBACK_H
#define GEAR_FEEDBACK_H

#include <stm32f4xx.h>
#include <board_driver/adc.h>
#include <board_driver/gpio.h>

#define ADC_GEAR_PIN                PIN_1
#define ADC_GEAR_GPIO_PORT          GPIOB
#define ADC_GEAR_CHANNEL            CHANNEL_9

void init_gear_feedback();
uint16_t read_gear_feedback();

#endif
