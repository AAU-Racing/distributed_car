#include <stm32f4xx_hal.h>

#include <shield_driver/dashboard/TLC59116.h>

#include <stdint.h>

#include <shield_driver/dashboard/rpm.h>
#include <board_driver/can.h>

int main(void) {
	led_driver_init_dash(false);
	can_init(CAN_PD0);
	HAL_Delay(10);
	can_start();

	while(1) {
		for (uint8_t color = 0; color < 3; color++) {
			for (uint8_t led = 0; led < 25; led++) {
				uint8_t red = (color == 0) * 0xF;
				uint8_t green = (color == 1) * 0xF;
				uint8_t blue = (color == 2) * 0xF;

				set_led_dash(led, red, green, blue);

				HAL_Delay(10);
			}
		}
	}
}
