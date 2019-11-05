#include <board_driver/gpio.h>
#include <board_driver/pwm.h>

#include "actuator.h"

static TIM_HandleTypeDef timer;

void init_actuator() {
	ACTUATOR1_PWM_CLK_ENABLE();
	ACTUATOR2_PWM_CLK_ENABLE();

	gpio_output_init(ACTUATOR1_EN_PORT, ACTUATOR1_EN_PIN);
	gpio_output_init(ACTUATOR1_DIR_PORT, ACTUATOR1_DIR_PIN);

	gpio_output_init(ACTUATOR2_EN_PORT, ACTUATOR2_EN_PIN);
	gpio_output_init(ACTUATOR2_DIR_PORT, ACTUATOR2_DIR_PIN);

	ACTUATOR_PWM_TIMER_CLK_ENABLE();
	init_pwm(ACTUATOR_PWM_TIMER, &timer);
	init_pwm_pin(ACTUATOR1_PWM_PORT, ACTUATOR1_PWM_PIN, ACTUATOR1_PWM_GPIO_AF);
	init_pwm_pin(ACTUATOR2_PWM_PORT, ACTUATOR2_PWM_PIN, ACTUATOR2_PWM_GPIO_AF);
}

void actuator_backward_start() {
	configure_pwm_channel(&timer, ACTUATOR1_PWM_CHANNEL, 100);
	start_pwm_channel(&timer, ACTUATOR1_PWM_CHANNEL);
}

void actuator_backward_slow() {
	configure_pwm_channel(&timer, ACTUATOR1_PWM_CHANNEL, 25);
	start_pwm_channel(&timer, ACTUATOR1_PWM_CHANNEL);
}

void actuator_backward_stop() {
	stop_pwm_channel(&timer, ACTUATOR1_PWM_CHANNEL);
}

void actuator_forward_start() {
	gpio_toggle_on(ACTUATOR2_EN_PORT, ACTUATOR2_EN_PIN);
}

void actuator_forward_stop() {
	gpio_toggle_off(ACTUATOR2_EN_PORT, ACTUATOR2_EN_PIN);
}
