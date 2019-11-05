#include <stdbool.h>

#include <board_driver/can.h>
#include <board_driver/bkpsram.h>

#include "gear.h"
#include "gear_feedback.h"
#include "neutral.h"
#include "actuator.h"
#include "ignition_cut.h"

#define ACTUATOR_DELAY 20

static uint8_t gear = 255;
static uint8_t wanted_gear = 0;

static bool started = false;
static uint32_t start = 0;

static bool neutral_seen = false;
static bool overridden = false;

static void commit_gear() {
	ignition_cut_off();
	started = false;
	start = HAL_GetTick();
	write_bkpsram(BKPSRAM_GEAR, gear);
	can_transmit(CAN_GEAR_NUMBER, (uint8_t[]) {gear}, 1);
}

static void start_change() {
	started = true;
	start = HAL_GetTick();
	ignition_cut_on();
}

static void check_neutral_switch() {
	if (neutral_switch_get_state()) {
		neutral_seen = true;
	}
}

static void neutral_to_first() {
	if (!started) {
		start_change();
		actuator_forward_start();
	}
	else {
		if (HAL_GetTick() - start > ACTUATOR_DELAY) {
			actuator_forward_stop();
			gear = 1;
			commit_gear();
		}
	}
}

static void first_to_neutral() {
	if (!started) {
		start_change();
		actuator_backward_slow();
	}
	else {
		if (HAL_GetTick() - start > ACTUATOR_DELAY) {
			actuator_backward_stop();
			gear = 0;
			commit_gear();
		}
	}
}

static void gear_down() {
	if (!started) {
		start_change();
		actuator_forward_start();
	}
	else {
		check_neutral_switch();

		if (HAL_GetTick() - start > ACTUATOR_DELAY) {
			actuator_forward_stop();
			gear--;
			commit_gear();
		}
	}
}

static void gear_up() {
	if (!started) {
		start_change();
		actuator_backward_start();
	}
	else {
		check_neutral_switch();

		if (HAL_GetTick() - start > ACTUATOR_DELAY) {
			actuator_backward_stop();
			gear++;
			commit_gear();
		}
	}
}

static void gear_button_callback(CAN_RxFrame *msg) {
	if (msg->Msg[0] == CAN_GEAR_BUTTON_NEUTRAL) {
		wanted_gear = 0;
	}
	else if (msg->Msg[0] == CAN_GEAR_BUTTON_UP) {
		if (wanted_gear < 6) {
			wanted_gear++;
		}
	}
	else if (msg->Msg[0] == CAN_GEAR_BUTTON_DOWN) {
		if (wanted_gear > 1) {
			wanted_gear--;
		}
	}
	else if (msg->Msg[0] >= CAN_GEAR_BUTTON_OVERRIDE_NEUTRAL &&
			 msg->Msg[0] <= CAN_GEAR_BUTTON_OVERRIDE_6) {
		gear = msg->Msg[0] - CAN_GEAR_BUTTON_OVERRIDE_NEUTRAL;
		wanted_gear = gear;
		commit_gear();
		overridden = true;
	 }
}

// Public functions
int init_gear() {
	init_actuator();
	init_ignition_cut();
	init_bkpsram();

	return can_filter(CAN_GEAR_BUTTONS, 0x7FF, gear_button_callback);
}

void read_initial_gear() {
	if (bkpsram_was_enabled_last_run()) {
		gear = read_bkpsram(BKPSRAM_GEAR);
	}
	else {
		do {
			gear_up();
		} while (started);

		// Wait between shifting
		HAL_Delay(50);

		if (neutral_seen) {
			do {
				gear_down();
			} while (started);
		}
		else {
			while (!neutral_seen) {
				do {
					gear_down();
				} while (started);

				// Wait between shifting
				HAL_Delay(50);
			}
		}
	}

	wanted_gear = gear;
	commit_gear();
}

void change_gear() {
	if (overridden) {
		overridden = false;
		printf("Gear overridden to %u\n", gear);
	}

	if (wanted_gear == gear) {
		return;
	}

	if (wanted_gear >= 1 && gear == 0) {
		neutral_to_first();
	}
	else if (wanted_gear == 0 && gear == 1) {
		first_to_neutral();
	}
	else if (wanted_gear > gear) {
		gear_up();
	}
	else if (wanted_gear < gear) {
		gear_down();
	}
}

uint8_t gear_number() {
	return gear;
}
