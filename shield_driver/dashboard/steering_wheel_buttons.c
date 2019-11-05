#include <stdint.h>
#include <stm32f4xx_hal.h>

#include <board_driver/gpio.h>

#include "steering_wheel_buttons.h"

bool state[SW_BUTTONn];
bool last_state[SW_BUTTONn];
uint32_t last_toogle[SW_BUTTONn];

#define DEBOUNCE_DELAY 25

#define SW_BUTTON_PORT ((GPIO_TypeDef* const []) { \
	SW_BUTTON1_GPIO_PORT, \
	SW_BUTTON2_GPIO_PORT, \
	SW_BUTTON3_GPIO_PORT, \
	SW_BUTTON4_GPIO_PORT, \
	SW_BUTTON5_GPIO_PORT, \
	SW_BUTTON6_GPIO_PORT, \
	SW_BUTTON7_GPIO_PORT, \
	SW_BUTTON8_GPIO_PORT, \
})

#define SW_BUTTON_PIN ((const uint16_t const []) { \
	SW_BUTTON1_PIN, \
	SW_BUTTON2_PIN, \
	SW_BUTTON3_PIN, \
	SW_BUTTON4_PIN, \
	SW_BUTTON5_PIN, \
	SW_BUTTON6_PIN, \
	SW_BUTTON7_PIN, \
	SW_BUTTON8_PIN, \
})

bool sw_button_get_state(sw_button b) {
	bool read = gpio_get_state(SW_BUTTON_PORT[b], SW_BUTTON_PIN[b]);

	if (read != last_state[b]) {
		last_toogle[b] = HAL_GetTick();
	}

	if (HAL_GetTick() - last_toogle[b] > DEBOUNCE_DELAY) {
		state[b] = read;
	}

	last_state[b] = read;

	return !state[b];
}

void sw_buttons_init(void) {
	for (sw_button b = 0; b < SW_BUTTONn; b++) {
		//Tepmpoary fix for crashing in gpio_exti_init with sw_button2
		// if (b != SW_BUTTON2)
		state[b] = false;
		last_state[b] = false;
		last_toogle[b] = HAL_GetTick();
		gpio_input_init(SW_BUTTON_PORT[b], SW_BUTTON_PIN[b], GPIO_PULLUP);
	}
}

#if 0
__weak void SW_Button1_Callback() {}
__weak void SW_Button2_Callback() {} //Button 2 is currently unavailable
__weak void SW_Button3_Callback() {}
__weak void SW_Button4_Callback() {}
__weak void SW_Button5_Callback() {}

void sw_buttons_handle(uint16_t sw_button) {
	switch(sw_button) {
		case SW_BUTTON1_PIN:
			SW_Button1_Callback();
		case SW_BUTTON2_PIN:
			SW_Button2_Callback();
		case SW_BUTTON3_PIN:
			SW_Button3_Callback();
		case SW_BUTTON4_PIN:
			SW_Button4_Callback();
		case SW_BUTTON5_PIN:
			SW_Button5_Callback();
	}
}

void sw_buttons_init(void) {
	for (sw_button b = 0; b < SW_BUTTONn; b++) {
		//Tepmpoary fix for crashing in gpio_exti_init with sw_button2
		// if (b != SW_BUTTON2)
			gpio_exti_init(SW_BUTTON_PORT[b], SW_BUTTON_PIN[b], GPIO_FALLING_EDGE, sw_buttons_handle);
	}
}
#endif
