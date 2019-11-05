#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>

#include <stdbool.h>

#include <board_driver/gpio.h>

#include "power.h"

__weak void power_interupt_callback(uint16_t pin){
	(void) pin;
}

void power_interupt_pin_init() {
	gpio_exti_init(POWER_INTERUPT_GPIO_PORT, POWER_INTERUPT_PIN, GPIO_FALLING_EDGE, power_interupt_callback);
}

bool power_get_state() {
	return gpio_get_state(POWER_INTERUPT_GPIO_PORT, POWER_INTERUPT_PIN);
}
