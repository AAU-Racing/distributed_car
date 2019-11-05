#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "gpio.h"

// Store callback functions
GPIO_Callback callback_funcions[16];

// Allow different external interrupt lines to be initialized
static IRQn_Type get_interrupt_line(uint8_t pos) {
	static IRQn_Type irqn[16] = {
		EXTI0_IRQn,
		EXTI1_IRQn,
		EXTI2_IRQn,
		EXTI3_IRQn,
		EXTI4_IRQn,
		EXTI9_5_IRQn,
		EXTI9_5_IRQn,
		EXTI9_5_IRQn,
		EXTI9_5_IRQn,
		EXTI9_5_IRQn,
		EXTI15_10_IRQn,
		EXTI15_10_IRQn,
		EXTI15_10_IRQn,
		EXTI15_10_IRQn,
		EXTI15_10_IRQn,
		EXTI15_10_IRQn,
	};

	return irqn[pos];
}

static uint8_t pin_number(GPIO_Pin pin) {
	int i = 0;

	// Pin 0 = 0x0001U
	// Pin 1 = 0x0002U
	// Pin 2 = 0x0004U
	// Etc.
	while (pin != 1) {
		pin = pin >> 1;
		i++;
	}

	return i;
}

static uint8_t port_number(GPIO_TypeDef *port) {
	uint32_t num = (uint32_t) port; // Convert pointer to int

	num -= AHB1PERIPH_BASE; // Extract offset

	return num / 0x400; // Offset of 0x400 per port
}

static void init_gpio_clock(GPIO_TypeDef *port) {
	static uint32_t bits[9] = {
		RCC_AHB1ENR_GPIOAEN,
		RCC_AHB1ENR_GPIOBEN,
		RCC_AHB1ENR_GPIOCEN,
		RCC_AHB1ENR_GPIODEN,
		RCC_AHB1ENR_GPIOEEN,
		RCC_AHB1ENR_GPIOFEN,
		RCC_AHB1ENR_GPIOGEN,
		RCC_AHB1ENR_GPIOHEN,
		RCC_AHB1ENR_GPIOIEN,
	};

	// Enable clock
	SET_BIT(RCC->AHB1ENR, bits[port_number(port)]);
}

// Init functions
static void set_mode(GPIO_TypeDef *port, uint8_t pos, GPIO_Mode mode) {
	uint32_t mode_msk = 3 << (pos * 2U);
	uint32_t mode_val = (mode & 3) << (pos * 2U);

	MODIFY_REG(port->MODER, mode_msk, mode_val);	// Set pin bits (2 bits wide) (see mode typedef)
}

static void set_pull(GPIO_TypeDef *port, uint8_t pos, GPIO_Pull pull) {
	MODIFY_REG(port->PUPDR, 3 << (pos * 2U), pull << (pos * 2U));	// Set pin bits (2 bits wide) (see pull typedef)
}

static void set_speed(GPIO_TypeDef *port, uint8_t pos, GPIO_Speed speed) {
	uint32_t speed_msk = 3 << (pos * 2U);
	uint32_t speed_val = (speed & 3) << (pos * 2U);

	MODIFY_REG(port->OSPEEDR, speed_msk, speed_val);	// Set pin bits (2 bits wide) (see speed typedef)
}

static void set_output_type(GPIO_TypeDef *port, uint8_t pos, GPIO_OutputType ot) {
	uint32_t output_msk = 1 << pos;
	uint32_t output_val = ot << pos;

	MODIFY_REG(port->OTYPER, output_msk, output_val); // Set pin bits (1 bit wide) (see output type typdef)
}

static void set_af(GPIO_TypeDef *port, uint8_t pos, GPIO_AlternateFunction af) {
	uint8_t reg_num = pos >> 3;
	uint8_t trunc_pos = pos & 0x7;
	uint32_t af_msk = 0xF << (trunc_pos * 4U);
	uint32_t af_val = (af & 0xF) << (trunc_pos * 4U);

													// Get register number (low or high). Pin 0-7 = low, pin 8-15 = high
	MODIFY_REG(port->AFR[reg_num], af_msk, af_val); // Set pin bits (4 bits wide) (see alternate function typedef)
}

void gpio_af_init(GPIO_TypeDef *port, GPIO_Pin pin, GPIO_Speed speed, GPIO_OutputType ot, GPIO_AlternateFunction af) {
	// Init clock
	init_gpio_clock(port);
	// Get pin number
	uint8_t pos = pin_number(pin);

	// Set parameters
	set_af(port, pos, af);
	set_speed(port, pos, speed);
	set_output_type(port, pos, ot);
	set_pull(port, pos, GPIO_NO_PULL);
	set_mode(port, pos, GPIO_AF);
}

void gpio_input_init(GPIO_TypeDef *port, GPIO_Pin pin, GPIO_Pull pull) {
	// Init clock
	init_gpio_clock(port);

	// Get pin number
	uint8_t pos = pin_number(pin);

	// Set parameters
	set_mode(port, pos, GPIO_INPUT);
	set_pull(port, pos, pull);
	set_speed(port, pos, GPIO_MEDIUM_SPEED);
}

void gpio_analog_input_init(GPIO_TypeDef *port, GPIO_Pin pin) {
	// Init clock
	init_gpio_clock(port);

	// Get pin number
	uint8_t pos = pin_number(pin);

	// Set parameters
	set_mode(port, pos, GPIO_ANALOG);
	set_pull(port, pos, GPIO_NO_PULL);
	set_speed(port, pos, GPIO_MEDIUM_SPEED);
}

void gpio_exti_init(GPIO_TypeDef *port, GPIO_Pin pin, GPIO_InterruptMode mode, GPIO_Callback callback) {
	// Init clock
	init_gpio_clock(port);

	// Get pin number
	uint8_t pos = pin_number(pin);

	// Select port for interrupt (note that only 1 port for each pin number can interrupt)
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);  	// Start clock for SYSCFG
	uint8_t reg_num = pos >> 2;						// Get register number
	MODIFY_REG(SYSCFG->EXTICR[reg_num],
		0xF << ((pos & 0x3) * 4U),
		port_number(port) << ((pos & 0x3) * 4U)); 	// Set port for external interrupt line

	// Enable interrupt for that pin number
	SET_BIT(EXTI->IMR, pin);

	// Clear edge configuration
	CLEAR_BIT(EXTI->RTSR, pin);
	CLEAR_BIT(EXTI->FTSR, pin);

	// Set edge configuration
	if ((mode & GPIO_RISING_EDGE) != 0) {
		SET_BIT(EXTI->RTSR, pin);
	}

	if ((mode & GPIO_FALLING_EDGE) != 0) {
		SET_BIT(EXTI->FTSR, pin);
	}

	// Set parameters
	set_mode(port, pos, GPIO_INPUT);
	set_pull(port, pos, GPIO_NO_PULL);

	// Set priority and enable interrupt
	NVIC_SetPriority(get_interrupt_line(pos), NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0xF, 0xF));
	NVIC_EnableIRQ(get_interrupt_line(pos));

	// Set callback function for that pin
	callback_funcions[pos] = callback;

}

void gpio_output_init(GPIO_TypeDef *port, GPIO_Pin pin) {
	// Init clock
	init_gpio_clock(port);

	// Get pin number
	uint8_t pos = pin_number(pin);

	// Set parameters
	set_mode(port, pos, GPIO_OUTPUT);
	set_pull(port, pos, GPIO_PULLUP);
	set_speed(port, pos, GPIO_HIGH_SPEED);
	set_output_type(port, pos, GPIO_PUSHPULL);
}

void gpio_toggle_on(GPIO_TypeDef *port, GPIO_Pin pin) {
	uint8_t pos = pin_number(pin);				 	// Get pin position
	SET_BIT(port->ODR, pos);						// Set the output bit for that pin
}

void gpio_toggle_off(GPIO_TypeDef *port, GPIO_Pin pin) {
	uint8_t pos = pin_number(pin);				 	// Get pin position
	CLEAR_BIT(port->ODR, pos);						// Clear the output bit for that pin
}

// Get state of a normal pin
bool gpio_get_state(GPIO_TypeDef *port, GPIO_Pin pin) {
	uint8_t pos = pin_number(pin);					// Get pin position
	return (port->IDR >> pos) & 1;				// Shift input bit to 1st bit and 'and' with 1 to remove data for other pins
}

// Interrupt callback
static void EXTI_Callback(GPIO_Pin pin) {
	if((EXTI->PR & pin) != 0) {			// 0 if not pending
      	EXTI->PR = pin; 				// Clear pending
		uint8_t pos = pin_number(pin); 	// Get pin number as int
  		callback_funcions[pos](pin); 	// Call calback function
    }
}

// Handle interrupts
void EXTI0_IRQHandler(void) { EXTI_Callback(PIN_0); }
void EXTI1_IRQHandler(void) { EXTI_Callback(PIN_1); }
void EXTI2_IRQHandler(void) { EXTI_Callback(PIN_2); }
void EXTI3_IRQHandler(void) { EXTI_Callback(PIN_3); }
void EXTI4_IRQHandler(void) { EXTI_Callback(PIN_4); }
void EXTI9_5_IRQHandler(void) {
	EXTI_Callback(PIN_5);
	EXTI_Callback(PIN_6);
	EXTI_Callback(PIN_7);
	EXTI_Callback(PIN_8);
	EXTI_Callback(PIN_9);
}
void EXTI15_10_IRQHandler(void) {
	EXTI_Callback(PIN_10);
	EXTI_Callback(PIN_11);
	EXTI_Callback(PIN_12);
	EXTI_Callback(PIN_13);
	EXTI_Callback(PIN_14);
	EXTI_Callback(PIN_15);
}
