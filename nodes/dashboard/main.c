#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>

#include <board_driver/can.h>
#include <board_driver/obdii.h>
#include <board_driver/iwdg.h>

#include <shield_driver/dashboard/TLC59116.h>
#include <shield_driver/dashboard/ea_dogxl160w-7.h>

#include <shield_driver/dashboard/buttons.h>
#include <shield_driver/dashboard/steering_wheel_buttons.h>

#include <shield_driver/dashboard/rpm.h>
#include <shield_driver/dashboard/gear.h>
#include <shield_driver/dashboard/oil.h>
#include <shield_driver/dashboard/sensor_data.h>

#define DISABLE_ELECTRONIC_GEAR

#define BLINK_PULSE_WIDTH 200

uint8_t brightness_level = 1;
bool oil_pressure_warning_state = false;
bool warning_active = false;
uint32_t last_state_change = 0;
bool triggered = false;
bool com_fail_warning_state = false;

uint8_t red[] = 	{0x04, 0x00, 0x00};
uint8_t green[] = 	{0x00, 0x03, 0x00};
uint8_t blue[] = 	{0x00, 0x00, 0x06};
uint8_t yellow[] = 	{0x05, 0x02, 0x00};
uint8_t white[] = 	{0x02, 0x02, 0x02};

void setup(void);
void loop(void);
void brightness_level_up();
void brightness_level_down();
void write_to_led(uint8_t led_number, uint16_t r, uint16_t g, uint16_t b);
void oil_pressure_warning();
void com_fail_warning();
void show_rpm(void);
void check_neutral_switch();
void check_water_temp();

#ifndef DISABLE_ELECTRONIC_GEAR
	gear_t show_gear(void);
	bool check_neutral_request(gear_t gear, bool neutral_btn, bool clutch);
	bool check_down_request(gear_t gear, bool down_btn, bool clutch);
	bool check_up_request(gear_t gear, bool up_btn);
#endif

int main(void) {
	setup();

	while (1) {
		loop();
	}
}

void setup(void) {
	can_init(CAN_PD0);

	// Init simple peripherals
	led_driver_init(true);
	sw_buttons_init();

	// CAN filters
#ifdef DISABLE_ELECTRONIC_GEAR
	gear_init();
#endif
	oil_init();
	obdii_init();
	sensor_data_init();

	// Start CAN
	can_start();

	HAL_Delay(10);

	start_iwdg();

	can_transmit(CAN_DASHBOARD_STARTED, (uint8_t[]) { 1 }, 1);
}

void loop(void) {
	CAN_Statistics stats = can_get_stats();

	bool up_btn = sw_button_get_state(SW_BUTTON6);
	bool neutral_btn = sw_button_get_state(SW_BUTTON4);
	bool down_btn = sw_button_get_state(SW_BUTTON2);
#ifndef DISABLE_ELECTRONIC_GEAR
	bool clutch = sw_button_get_state(SW_BUTTON1);
#endif
	bool oil_pressure = oil_pressure_ok();


#ifndef DISABLE_ELECTRONIC_GEAR
	gear_t gear = show_gear();
#endif

#ifndef DISABLE_ELECTRONIC_GEAR
	if (!check_neutral_request(gear, neutral_btn, clutch)) {
		if (!check_down_request(gear, down_btn, clutch)) {
			if (!check_up_request(gear, up_btn)) {
				triggered = false;
			}
		}
	}
#else
	if (neutral_btn && up_btn) {
		brightness_level_up();
	}
	else if (neutral_btn && down_btn) {
		brightness_level_down();
	}
#endif

	if (oil_pressure && HAL_GetTick() - stats.last_receive < 10000 && warning_active) {
		for (int i = 0; i < 15; i++) {
			write_to_led(i, 0, 0, 0);
		}
	}

	check_neutral_switch();
	check_water_temp();

	if (HAL_GetTick() - stats.last_receive > 10000 || stats.last_receive == 0) {
	 	com_fail_warning();
	} else if (oil_pressure) {
		show_rpm();
		warning_active = false;
	} else {
		oil_pressure_warning();
	}

	reset_iwdg();
}

void write_to_led(uint8_t led_number, uint16_t r, uint16_t g, uint16_t b) {
	r = r * brightness_level;
	g = g * brightness_level;
	b = b * brightness_level;

	set_led(led_number, r, g, b);
}

void brightness_level_up() {
	if (brightness_level < 8) {
		brightness_level++;
	}
}

void brightness_level_down() {
	if (brightness_level > 1) {
		brightness_level--;
	}
}

void com_fail_warning() {
	warning_active = true;
	if (HAL_GetTick() - last_state_change > BLINK_PULSE_WIDTH) {
		com_fail_warning_state = !com_fail_warning_state;
		last_state_change = HAL_GetTick();

		if (com_fail_warning_state) {
			for(int i = 0; i < 15; i++){
				write_to_led(i, blue[0] / 2, blue[1] / 2, blue[2] / 2);
			}
		}
		else {
			for(int i = 0; i < 15; i++){
				write_to_led(i, 0, 0, 0);
			}
		}
	}
}

void oil_pressure_warning() {
	warning_active = true;
	if (HAL_GetTick() - last_state_change > BLINK_PULSE_WIDTH) {
		oil_pressure_warning_state = !oil_pressure_warning_state;
		last_state_change = HAL_GetTick();

		if (oil_pressure_warning_state) {
			for(int i = 0; i < 15; i++){
				write_to_led(i, red[0], red[1], red[2]);
			}
		}
		else {
			for(int i = 0; i < 15; i++){
				write_to_led(i, 0, 0, 0);
			}
		}
	}
}

void show_rpm(void){
	bool new;
	int rpm_level = get_rpm_level(&new);

	if (warning_active || new) {
		for(int led = 0; led < 15; led++) {
			if (rpm_level > 30 + led) {
				write_to_led(led, blue[0], blue[1], blue[2]);
			}
			else if (rpm_level > 15 + led) {
				write_to_led(led, red[0], red[1], red[2]);
			}
			else if (rpm_level > led) {
				write_to_led(led, green[0], green[1], green[2]);
			}
			else {
				write_to_led(led, 0, 0, 0);
			}
		}
	}
}

void check_neutral_switch() {
	bool neutral = false;
	bool new = get_neutral(&neutral);

	if (new) {
		if (neutral) {
			write_to_led(15, red[0], red[1], red[2]);
		}
		else {
			write_to_led(15, 0, 0, 0);
		}
	}
}

gear_t show_gear(void){
	gear_t gear;
	bool new = get_gear(&gear);

	if (!new) {
		return gear;
	}

	// Can only show gear 1 to 4 and neutral
	if(gear < GEAR_5){
		for(uint8_t led = 15; led < 20; led++){
			write_to_led(led, 0, 0, 0);
		}

		if (gear == GEAR_N) {
			write_to_led((gear + 15), red[0], red[1], red[2]);
		}
		else {
			write_to_led((gear + 15), green[0], green[1], green[2]);
		}
	}
	else {
		for(uint8_t led = 15; led < 20; led++){
			write_to_led(led, white[0], white[1], white[2]);
		}
	}

	return gear;
}

static void transmit_gear_request(uint8_t button_msg) {
	if (!triggered) {
		can_transmit(CAN_GEAR_BUTTONS, (uint8_t[]) { button_msg }, 1);
		triggered = true;
	}
}

bool check_neutral_request(gear_t gear, bool neutral_btn, bool clutch) {
	if (neutral_btn) {
		if (!clutch) {
			write_to_led(20, blue[0], blue[1], blue[2]);
			return true;
		}
		else if (gear != GEAR_1) {
			write_to_led(20, red[0], red[1], red[2]);
			return true;
		}
		else {
			write_to_led(20, 0, 0, 0);
		}

		transmit_gear_request(CAN_GEAR_BUTTON_NEUTRAL);

		return true;
	}

	return false;
}


bool check_down_request(gear_t gear, bool down_btn, bool clutch) {
	if (down_btn) {
		if (!clutch) {
			write_to_led(20, blue[0], blue[1], blue[2]);
			return true;
		}
		else if (gear < GEAR_2) {
			write_to_led(20, red[0], red[1], red[2]);
			return true;
		}
		else {
			write_to_led(20, 0, 0, 0);
		}

		transmit_gear_request(CAN_GEAR_BUTTON_DOWN);

		return true;
	}

	return false;
}

bool check_up_request(gear_t gear, bool up_btn) {
	if (up_btn) {
		if (gear >= GEAR_5) {
			write_to_led(20, red[0], red[1], red[2]);
			return true;
		}
		else {
			write_to_led(20, 0, 0, 0);
		}

		transmit_gear_request(CAN_GEAR_BUTTON_UP);

		return true;
	}

	return false;
}

void check_water_temp() {
	float value;
	bool new = get_water_in(&value);

	if (!new) {
		return;
	}

	int temperature = (int) value;

	static int offset = 50;
	static int step = 5;

	int index = (temperature - offset) / step;

	for (int led = 24; led >= 20; led--) {
		int led_number = 24 - led;

		if (index >= 15 + led_number) {
			write_to_led(led, red[0], red[1], red[2]);
		}
		else if (index >= 10 + led_number) {
			write_to_led(led, yellow[0], yellow[1], yellow[2]);
		}
		else if (index >= 5 + led_number) {
			write_to_led(led, green[0], green[1], green[2]);
		}
		else if (index >= led_number) {
			write_to_led(led, blue[0], blue[1], blue[2]);
		}
		else {
			write_to_led(led, 0, 0, 0);
		}
	}
}
