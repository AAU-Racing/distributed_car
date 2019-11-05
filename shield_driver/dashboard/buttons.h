#ifndef DASHBOARD_BUTTONS_H
#define DASHBOARD_BUTTONS_H

#include <stm32f4xx_hal.h>
#include <stdbool.h>

typedef enum {
	DASHBOARD_BUTTON1,
	DASHBOARD_BUTTON2,
	DASHBOARD_BUTTON3,
	DASHBOARD_BUTTON4,
	DASHBOARD_BUTTON5,
	DASHBOARD_BUTTON6,
	DASHBOARD_BUTTON7,
	DASHBOARD_BUTTON8,

	DASHBOARD_BUTTONn // number of buttons
} dashboard_button;

#define BUTTON1_PIN                    PIN_14
#define BUTTON1_GPIO_PORT              GPIOD

#define BUTTON2_PIN                    PIN_15
#define BUTTON2_GPIO_PORT              GPIOD

#define BUTTON3_PIN                    PIN_2
#define BUTTON3_GPIO_PORT              GPIOD

#define BUTTON4_PIN                    PIN_3
#define BUTTON4_GPIO_PORT              GPIOD

#define BUTTON5_PIN                    PIN_4
#define BUTTON5_GPIO_PORT              GPIOD

#define BUTTON6_PIN                    PIN_5
#define BUTTON6_GPIO_PORT              GPIOD

#define BUTTON7_PIN                    PIN_6
#define BUTTON7_GPIO_PORT              GPIOD

#define BUTTON8_PIN                    PIN_0
#define BUTTON8_GPIO_PORT              GPIOE

void dashboard_buttons_init(void);
bool dashboard_button_get_state(dashboard_button b);
bool is_triggered(dashboard_button b);

#endif /* DASHBOARD_BUTTONS_H */
