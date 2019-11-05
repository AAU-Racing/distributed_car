#ifndef POTMETER_H
#define POTMETER_H

#include <stm32f4xx.h>
#include <board_driver/adc.h>
#include <board_driver/gpio.h>

#define ADC_POTMETER1_PIN                PIN_7
#define ADC_POTMETER1_GPIO_PORT          GPIOA
#define ADC_POTMETER1_CHANNEL            CHANNEL_7

#define ADC_POTMETER2_PIN                PIN_6
#define ADC_POTMETER2_GPIO_PORT          GPIOA
#define ADC_POTMETER2_CHANNEL            CHANNEL_6

#define ADC_POTMETER3_PIN                PIN_5
#define ADC_POTMETER3_GPIO_PORT          GPIOA
#define ADC_POTMETER3_CHANNEL            CHANNEL_5

#define ADC_POTMETER4_PIN                PIN_3
#define ADC_POTMETER4_GPIO_PORT          GPIOA
#define ADC_POTMETER4_CHANNEL            CHANNEL_3

void init_potmeter_input();
uint16_t read_potmeter(int rank);

#endif /* POTMETER_H */
