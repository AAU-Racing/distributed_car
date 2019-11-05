#include <stm32f4xx_hal.h>
#include <board_driver/gpio.h>

#include "wheel_sensor.h"

// Setup timer for sensor.
// setup interupts for seach input pin
// sample dt (in seconds) between interupts using timer
// take 1/dt to get freq

uint16_t ticks[WHEEL_INP_N] = {0};
float freq[WHEEL_INP_N] = {0};
#define TIMx TIM3
#define TIMx_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()

TIM_HandleTypeDef TimHandle;

static void init_timer(void) {
	TIMx_CLK_ENABLE();

	// uint32_t presc_10khz = (uint32_t) ((SystemCoreClock /2) / 10000) - 1; // 10khz
	uint32_t presc_1mhz = (uint32_t) ((SystemCoreClock / 2) / 1e6) - 1;

	// TODO set the correct timer config
	TimHandle.Instance = TIMx;
	// TimHandle.Init.Period = 10000 - 1;
	TimHandle.Init.Prescaler = presc_1mhz;
	TimHandle.Init.ClockDivision = 0;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
		// Error
	}

	// Perhaps enable interrupt to force a tick handle so a sample also runs when the car is not moving
	// if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
	// 	// Error
	// }
}


static inline unsigned wheel_sensor_tim_get_cnt(void) {
	return __HAL_TIM_GET_COUNTER(&TimHandle);
}


static inline void handle_tick(enum wheel_input inp) {
	uint16_t last = ticks[inp];
	uint16_t now = wheel_sensor_tim_get_cnt();

	if (last > now) {
		// timer has overflown
		// TODO handle
	}

	const uint16_t dt = now - last;
	float dt_sec = (double)dt * (double)1e6;
	if (dt_sec == 0) {
		// avoids division by zero
		freq[inp] = -1.0;
		return;
	}
	float f = 1.0f / (float)dt_sec;
	freq[inp] = f;
}

static void rpm1_gpio_callback(uint16_t pin_num) {
	(void) pin_num;
	handle_tick(RPM1);
}

static void rpm2_gpio_callback(uint16_t pin_num) {
	(void) pin_num;
	handle_tick(RPM2);
}

static void rpm3_gpio_callback(uint16_t pin_num) {
	(void) pin_num;
	handle_tick(RPM3);
}

static void rpm4_gpio_callback(uint16_t pin_num) {
	(void) pin_num;
	handle_tick(RPM4);
}

static void init_interrupt(enum wheel_input inp) {
	switch (inp) {
		case RPM1:
			gpio_exti_init(RPM1_GPIO_PORT, RPM1_PIN, GPIO_FALLING_EDGE, rpm1_gpio_callback);
			break;
		case RPM2:
			gpio_exti_init(RPM2_GPIO_PORT, RPM2_PIN, GPIO_FALLING_EDGE, rpm2_gpio_callback);
			break;
		case RPM3:
			gpio_exti_init(RPM3_GPIO_PORT, RPM3_PIN, GPIO_FALLING_EDGE, rpm3_gpio_callback);
			break;
		case RPM4:
			gpio_exti_init(RPM4_GPIO_PORT, RPM4_PIN, GPIO_FALLING_EDGE, rpm4_gpio_callback);
			break;

		default: return;
	}
}

void wheel_sensor_init(void) {
	init_timer();

	init_interrupt(WHEEL_INP_FR);
	init_interrupt(WHEEL_INP_FL);
	init_interrupt(WHEEL_INP_RR);
	init_interrupt(WHEEL_INP_RL);
}

serializable_float wheel_sensor_fr(void) {
	return (serializable_float) freq[WHEEL_INP_FR];
}

serializable_float wheel_sensor_fl(void) {
	return (serializable_float) freq[WHEEL_INP_FL];
}

serializable_float wheel_sensor_rr(void) {
	return (serializable_float) freq[WHEEL_INP_RR];
}

serializable_float wheel_sensor_rl(void) {
	return (serializable_float) freq[WHEEL_INP_RL];
}
