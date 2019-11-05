#include <stdbool.h>

#include "potmeter.h"

static uint8_t channel_rank[4];

void init_potmeter_input() {
	gpio_analog_input_init(ADC_POTMETER1_GPIO_PORT, ADC_POTMETER1_PIN);
	gpio_analog_input_init(ADC_POTMETER2_GPIO_PORT, ADC_POTMETER2_PIN);
	gpio_analog_input_init(ADC_POTMETER3_GPIO_PORT, ADC_POTMETER3_PIN);
	gpio_analog_input_init(ADC_POTMETER4_GPIO_PORT, ADC_POTMETER4_PIN);

	init_adc_channel(ADC_POTMETER1_CHANNEL, channel_rank);
	init_adc_channel(ADC_POTMETER2_CHANNEL, channel_rank + 1);
	init_adc_channel(ADC_POTMETER3_CHANNEL, channel_rank + 2);
	init_adc_channel(ADC_POTMETER4_CHANNEL, channel_rank + 3);
}

uint16_t read_potmeter(int rank) {
	rank %= 4;

    return read_adc_value(channel_rank[rank]);
}
