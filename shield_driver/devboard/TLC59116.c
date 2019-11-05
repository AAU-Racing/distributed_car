#include <board_driver/TLC59116.h>
#include <stm32f4xx_hal.h>

#define N_LEDS_PR_CONTROLLER 5

void set_led_dev(unsigned led, unsigned r, unsigned g, unsigned b) {
	address addr = controller_lut((led % 5) / N_LEDS_PR_CONTROLLER);

	set_rgb(addr, led % N_LEDS_PR_CONTROLLER, r, g, b);
}

static void init_reset_pin(void) {
	__HAL_RCC_GPIOE_CLK_ENABLE();

	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin       = GPIO_PIN_15;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(1); // We need to wait for the pin to switch state
}

int led_driver_init_dev(bool china) {
	// Handle reset pin
	init_reset_pin();

	led_driver_init_base(china);

	init_controller(ADDR1);

	return 0;
}
