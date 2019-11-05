#include <stdbool.h>

#include <stm32f4xx_hal.h>
#include <board_driver/adc.h>
#include <board_driver/can.h>

#include "current_clamps.h"

static uint8_t channel_rank[2];

void init_current_clamps() {
	gpio_analog_input_init(ADC_CURRENT1_GPIO_PORT, ADC_CURRENT1_PIN);
	gpio_analog_input_init(ADC_CURRENT2_GPIO_PORT, ADC_CURRENT2_PIN);

	init_adc_channel(ADC_CURRENT1_CHANNEL, channel_rank);
	init_adc_channel(ADC_CURRENT2_CHANNEL, channel_rank + 1);
}

uint16_t read_battery_current() {
    return read_adc_value(channel_rank[0]);
}

uint16_t read_inverter_current() {
    return read_adc_value(channel_rank[1]);
}
