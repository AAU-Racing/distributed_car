#include <stdbool.h>

#include <stm32f4xx_hal.h>
#include <board_driver/adc.h>

#include "water_temp.h"

static uint8_t channel_rank[2];

void init_water_temp() {
	gpio_analog_input_init(ADC_WATER_IN_GPIO_PORT, ADC_WATER_IN_PIN);
	gpio_analog_input_init(ADC_WATER_OUT_GPIO_PORT, ADC_WATER_OUT_PIN);

	init_adc_channel(ADC_WATER_IN_CHANNEL, channel_rank);
	init_adc_channel(ADC_WATER_OUT_CHANNEL, channel_rank + 1);
}

uint16_t read_water_in() {
    return read_adc_value(channel_rank[0]);
}

uint16_t read_water_out() {
    return read_adc_value(channel_rank[1]);
}
