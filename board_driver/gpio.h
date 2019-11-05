#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>
#include <stm32f4xx.h>

typedef enum {
	PIN_0  = 0x0001,
	PIN_1  = 0x0002,
	PIN_2  = 0x0004,
	PIN_3  = 0x0008,
	PIN_4  = 0x0010,
	PIN_5  = 0x0020,
	PIN_6  = 0x0040,
	PIN_7  = 0x0080,
	PIN_8  = 0x0100,
	PIN_9  = 0x0200,
	PIN_10 = 0x0400,
	PIN_11 = 0x0800,
	PIN_12 = 0x1000,
	PIN_13 = 0x2000,
	PIN_14 = 0x4000,
	PIN_15 = 0x8000
} GPIO_Pin;

typedef enum {
	GPIO_INPUT 	= 0,
	GPIO_OUTPUT = 1,
	GPIO_AF	    = 2,
	GPIO_ANALOG = 3,
} GPIO_Mode;

typedef enum {
	GPIO_PUSHPULL  = 0,
	GPIO_OPENDRAIN = 1,
} GPIO_OutputType;

typedef enum {
	GPIO_LOW_SPEED 		 = 0,
	GPIO_MEDIUM_SPEED 	 = 1,
	GPIO_HIGH_SPEED		 = 2,
	GPIO_VERY_HIGH_SPEED = 3,
} GPIO_Speed;

typedef enum {
	GPIO_NO_PULL   = 0,
	GPIO_PULL_UP   = 1,
	GPIO_PULL_DOWN = 2,
} GPIO_Pull;

typedef enum {
	GPIO_RISING_EDGE  		 = 1,
	GPIO_FALLING_EDGE 		 = 2,
	GPIO_RISING_FALLING_EDGE = 3,
} GPIO_InterruptMode;

typedef enum {
	GPIO_AF0  = 0,
	GPIO_AF1  = 1,
	GPIO_AF2  = 2,
	GPIO_AF3  = 3,
	GPIO_AF4  = 4,
	GPIO_AF5  = 5,
	GPIO_AF6  = 6,
	GPIO_AF7  = 7,
	GPIO_AF8  = 8,
	GPIO_AF9  = 9,
	GPIO_AF10 = 10,
	GPIO_AF11 = 11,
	GPIO_AF12 = 12,
	GPIO_AF13 = 13,
	GPIO_AF14 = 14,
	GPIO_AF15 = 15,
} GPIO_AlternateFunction;

typedef void (*GPIO_Callback)(uint16_t);

void gpio_input_init(GPIO_TypeDef *port, GPIO_Pin pin, GPIO_Pull pull);
void gpio_analog_input_init(GPIO_TypeDef *port, GPIO_Pin pin);
void gpio_exti_init(GPIO_TypeDef *port, GPIO_Pin pin, GPIO_InterruptMode mode, GPIO_Callback callback);
void gpio_output_init(GPIO_TypeDef *port, GPIO_Pin pin);
void gpio_af_init(GPIO_TypeDef *port, GPIO_Pin pin, GPIO_Speed speed, GPIO_OutputType ot, GPIO_AlternateFunction af);
bool gpio_get_state(GPIO_TypeDef *port, GPIO_Pin pin);
void gpio_toggle_on(GPIO_TypeDef *port, GPIO_Pin pin);
void gpio_toggle_off(GPIO_TypeDef *port, GPIO_Pin pin);

#endif /* GPIO_H */
