#include <stm32f4xx_hal.h>
#include <board_driver/adc.h>

#include "gear_feedback.h"

static uint8_t channel_rank;

void init_gear_feedback() {
	gpio_analog_input_init(ADC_GEAR_GPIO_PORT, ADC_GEAR_PIN);

	init_adc_channel(ADC_GEAR_CHANNEL, &channel_rank);
}

uint16_t read_gear_feedback() {
    return read_adc_value(channel_rank);
}
