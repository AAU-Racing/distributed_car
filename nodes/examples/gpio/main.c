#include <stm32f4xx_hal.h>

#include <shield_driver/dashboard/TLC59116.h>
#include <shield_driver/dashboard/buttons.h>
#include <shield_driver/dashboard/steering_wheel_buttons.h>
#include <board_driver/gpio.h>

#include <stdint.h>
#include <stdbool.h>

int main(void) {
	led_driver_init(true);
	// dashboard_buttons_init();
	sw_buttons_init();

	while(1) {
		for (sw_button b = 0; b < SW_BUTTONn; b++) {
			bool state = sw_button_get_state(b);

			uint8_t red = state * 0xF;
			uint8_t green = 0;
			uint8_t blue = 0;

			set_led(b, red, green, blue);
		}

		/* for (uint8_t color = 0; color < 3; color++) {
			for (uint8_t led = 0; led < 10; led++) {
				uint8_t red = (color == 0) * 0xF;
				uint8_t green = (color == 1) * 0xF;
				uint8_t blue = (color == 2) * 0xF;

				if (is_triggered(DASHBOARD_BUTTON1)) {
					set_led(19, 0xF, 0xF, 0xF);
				}

				set_led(led, red, green, blue);

				HAL_Delay(200);
			}
		} */
	}
}


// void SW_Button1_Callback(void) { set_led(10, 0xF, 0xF, 0xF); }
// void SW_Button2_Callback(void) { set_led(11, 0xF, 0xF, 0xF); }
// void SW_Button3_Callback(void) { set_led(12, 0xF, 0xF, 0xF); }
// void SW_Button4_Callback(void) { set_led(13, 0xF, 0xF, 0xF); }
// void SW_Button5_Callback(void) { set_led(14, 0xF, 0xF, 0xF); }
