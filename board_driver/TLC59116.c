#include <stm32f4xx_hal.h>

#include "i2c.h"
#include "can.h"

#include "TLC59116.h"

static bool china_led = false;

enum registers {
	MODE1, MODE2,

	PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7,
	PWM8, PWM9, PWM10, PWM11, PWM12, PWM13, PWM14, PWM15,

	GRPPWM,
	GRPFREQ,

	LEDOUT0, LEDOUT1, LEDOUT2, LEDOUT3,
	SUBADR1, SUBADR2, SUBADR3,

	ALLCALLADR,
	IREF,

	EFLAG1, EFLAG2,
};

address controller_lut(uint8_t x) {
	return ADDR1 + x * 2;
}

static void write_cmd(address addr, enum registers reg, uint8_t value) {
	uint8_t p[] = {(uint8_t)reg, value};

	if (i2c_master_transmit(addr, p, 2) != 0) {
		while(1); // TODO remove
	}
}

static void write_rgb(address addr, enum registers reg, uint8_t r, uint8_t g, uint8_t b) {
	uint8_t p[] = {(uint8_t)reg | (5 << 5), r, g, b};

	if (i2c_master_transmit(addr, p, 4) != 0) {
		while(1); // TODO remove
	}
}

void set_rgb(address addr, unsigned led, unsigned r, unsigned g, unsigned b) {
	enum registers lookup[] = {
		PWM0, PWM3, PWM6, PWM9, PWM12,
	};

	write_rgb(addr, lookup[led], r, g, b);
}

void init_controller(address ctrl) {
	write_cmd(ctrl, MODE1, 0x00);
	write_cmd(ctrl, MODE2, 0x00);

	for (enum registers ledout = LEDOUT0; ledout <= LEDOUT3; ledout++) {
		write_cmd(ctrl, ledout, 0xAA);
	}

	for (enum registers pwm_reg = PWM0; pwm_reg <= PWM15; pwm_reg++) {
		write_cmd(ctrl, pwm_reg, 0x00);
	}

}

int led_driver_init_base(bool china) {
	china_led = china;

	if (i2c_init() != 0) {
		while(1);
		return -1;
	}

	return 0;
}
