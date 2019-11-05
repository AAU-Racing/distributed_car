#include <stm32f4xx_hal.h>

#include "pwm.h"

#define PERIOD_VALUE (1000 - 1)
#define TIMx_COUNTER_CLK 10000000

HAL_StatusTypeDef init_pwm(TIM_TypeDef *instance, TIM_HandleTypeDef* htim) {
	/*####  Configure the TIM peripheral ####*/
	/* Initialize TIMx peripheral as follow:
	+ Prescaler = (SystemCoreClock/2)/18000000 (the subtracted 1 is because hardware adds 1 to the prescaler)
	+ Period = 1800  (to have an output frequency equal to 10 KHz)
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
	htim->Instance = instance;

	htim->Init.Prescaler     = ((HAL_RCC_GetSysClockFreq() / 2) / TIMx_COUNTER_CLK) - 1;
	htim->Init.Period        = PERIOD_VALUE;
	htim->Init.ClockDivision = 0;
	htim->Init.CounterMode   = TIM_COUNTERMODE_UP;

	return HAL_TIM_PWM_Init(htim);
}

HAL_StatusTypeDef configure_pwm_channel(TIM_HandleTypeDef* htim, uint32_t channel, float percent) {
	TIM_OC_InitTypeDef sConfig;

	/*#### Configure the PWM channels ####*/
	/* Common configuration for all channels */
	sConfig.OCMode 		= TIM_OCMODE_PWM1;
	sConfig.OCPolarity  = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode  = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;

	/* Set the pulse value for channel 1 */
	sConfig.Pulse = PERIOD_VALUE * percent / 100;

	return HAL_TIM_PWM_ConfigChannel(htim, &sConfig, channel);
}

void init_pwm_pin(GPIO_TypeDef *port, uint32_t pin, uint32_t alternate) {
  	GPIO_InitTypeDef   GPIO_InitStruct;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = alternate;
    GPIO_InitStruct.Pin = pin;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

HAL_StatusTypeDef start_pwm_channel(TIM_HandleTypeDef* htim, uint32_t channel) {
	/*#### Start PWM signals generation ####*/
	return HAL_TIM_PWM_Start(htim, channel);
}

HAL_StatusTypeDef stop_pwm_channel(TIM_HandleTypeDef* htim, uint32_t channel) {
	/*#### Stop PWM signals generation ####*/
	return HAL_TIM_PWM_Stop(htim, channel);
}
