#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stdint.h>

#include <board_driver/gpio.h>

#include "buttons.h"

bool triggeredBtn[DASHBOARD_BUTTONn];

#define BUTTON_PORT ((GPIO_TypeDef* const []) { \
	BUTTON1_GPIO_PORT, \
	BUTTON2_GPIO_PORT, \
	BUTTON3_GPIO_PORT, \
	BUTTON4_GPIO_PORT, \
	BUTTON5_GPIO_PORT, \
	BUTTON6_GPIO_PORT, \
	BUTTON7_GPIO_PORT, \
	BUTTON8_GPIO_PORT, \
})

#define BUTTON_PIN ((const uint16_t const []) { \
	BUTTON1_PIN, \
	BUTTON2_PIN, \
	BUTTON3_PIN, \
	BUTTON4_PIN, \
	BUTTON5_PIN, \
	BUTTON6_PIN, \
	BUTTON7_PIN, \
	BUTTON8_PIN, \
})

static uint8_t button_number(uint16_t pin) {
	switch(pin) {
		case BUTTON1_PIN:
			return 0;
		case BUTTON2_PIN:
			return 1;
		case BUTTON3_PIN:
			return 2;
		case BUTTON4_PIN:
			return 3;
		case BUTTON5_PIN:
			return 4;
		case BUTTON6_PIN:
			return 5;
		case BUTTON7_PIN:
			return 6;
		case BUTTON8_PIN:
			return 7;
		default:
			return 0xFF;
	}
}

bool dashboard_button_get_state(dashboard_button b) {
	return gpio_get_state(BUTTON_PORT[b], BUTTON_PIN[b]);
}

void dashboard_buttons_init(void) {
	for (dashboard_button b = 0; b < DASHBOARD_BUTTONn; b++) {
		gpio_input_init(BUTTON_PORT[b], BUTTON_PIN[b], GPIO_PULLUP);
	}
}

#if 0
bool is_triggered(dashboard_button b){
	bool triggered = triggeredBtn[b];
	triggeredBtn[b] = false;
	return triggered;
}

void buttons_handle(uint16_t number) {
	triggeredBtn[button_number(number)] = true;
}

void dashboard_buttons_init(void) {
	for (dashboard_button b = 0; b < DASHBOARD_BUTTONn; b++) {
		triggeredBtn[b] = false;
		gpio_exti_init(BUTTON_PORT[b], BUTTON_PIN[b], GPIO_FALLING_EDGE, buttons_handle);
	}
}

#endif
