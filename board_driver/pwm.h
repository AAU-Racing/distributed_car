#ifndef PWM_H
#define PWM_H

#include <stm32f4xx_hal.h>

HAL_StatusTypeDef init_pwm(TIM_TypeDef *instance, TIM_HandleTypeDef* htim);
HAL_StatusTypeDef configure_pwm_channel(TIM_HandleTypeDef* htim, uint32_t channel, float percent);
HAL_StatusTypeDef start_pwm_channel(TIM_HandleTypeDef* htim, uint32_t channel);
HAL_StatusTypeDef stop_pwm_channel(TIM_HandleTypeDef* htim, uint32_t channel);
void init_pwm_pin(GPIO_TypeDef *port, uint32_t pin, uint32_t alternate);

#endif
