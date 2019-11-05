#ifndef STEERING_WHEEL_BUTTONS_H
#define STEERING_WHEEL_BUTTONS_H

#include <stm32f4xx_hal.h>
#include <stdbool.h>

typedef enum {
	SW_BUTTON1,
	SW_BUTTON2,
	SW_BUTTON3,
	SW_BUTTON4,
	SW_BUTTON5,
	SW_BUTTON6,
	SW_BUTTON7,
	SW_BUTTON8,

	SW_BUTTONn // number of buttons
} sw_button;

#define SW_BUTTON1_PIN                    PIN_1
#define SW_BUTTON1_GPIO_PORT              GPIOB

#define SW_BUTTON2_PIN                    PIN_7
#define SW_BUTTON2_GPIO_PORT              GPIOE

#define SW_BUTTON3_PIN                    PIN_8
#define SW_BUTTON3_GPIO_PORT              GPIOE

#define SW_BUTTON4_PIN                    PIN_9
#define SW_BUTTON4_GPIO_PORT              GPIOE

#define SW_BUTTON5_PIN                    PIN_10
#define SW_BUTTON5_GPIO_PORT              GPIOE

#define SW_BUTTON6_PIN                    PIN_11
#define SW_BUTTON6_GPIO_PORT              GPIOE

#define SW_BUTTON7_PIN                    PIN_12
#define SW_BUTTON7_GPIO_PORT              GPIOE

#define SW_BUTTON8_PIN                    PIN_13
#define SW_BUTTON8_GPIO_PORT              GPIOE

void sw_buttons_init(void);
bool sw_button_get_state(sw_button b);

#endif /* STEERING_WHEEL_BUTTONS_H */
