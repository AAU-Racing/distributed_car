#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stm32f4xx_hal.h>
#include <board_driver/gpio.h>

#define ACTUATOR_PWM_TIMER				TIM4
#define ACTUATOR_PWM_TIMER_CLK_ENABLE()  __HAL_RCC_TIM4_CLK_ENABLE()

#define ACTUATOR1_EN_PIN				PIN_11
#define ACTUATOR1_EN_PORT				GPIOD

#define ACTUATOR1_DIR_PIN				PIN_10
#define ACTUATOR1_DIR_PORT				GPIOD

#define ACTUATOR1_PWM_PIN				GPIO_PIN_13
#define ACTUATOR1_PWM_PORT				GPIOD
#define ACTUATOR1_PWM_CLK_ENABLE()		__HAL_RCC_GPIOD_CLK_ENABLE()
#define ACTUATOR1_PWM_CHANNEL			TIM_CHANNEL_2
#define ACTUATOR1_PWM_GPIO_AF			GPIO_AF2_TIM4


#define ACTUATOR2_EN_PIN				PIN_14
#define ACTUATOR2_EN_PORT				GPIOD

#define ACTUATOR2_DIR_PIN				PIN_15
#define ACTUATOR2_DIR_PORT				GPIOD

#define ACTUATOR2_PWM_PIN				GPIO_PIN_12
#define ACTUATOR2_PWM_PORT				GPIOD
#define ACTUATOR2_PWM_CLK_ENABLE()		__HAL_RCC_GPIOD_CLK_ENABLE()
#define ACTUATOR2_PWM_CHANNEL			TIM_CHANNEL_1
#define ACTUATOR2_PWM_GPIO_AF			GPIO_AF2_TIM4

void init_actuator();
void actuator_forward_start();
void actuator_forward_stop();
void actuator_backward_slow();
void actuator_backward_start();
void actuator_backward_stop();

#endif
