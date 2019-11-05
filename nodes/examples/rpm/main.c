#include <stm32f4xx_hal.h>
#include <stdint.h>

#include <shield_driver/dashboard/TLC59116.h>
#include <shield_driver/dashboard/rpm.h>

int main(void) {
	led_driver_init(true);

	while(1){
		for(uint8_t led = 0; led < 15; led++){
			set_led(led, 0, 0, 0);
		}

		for(uint16_t rpm = 800; rpm < RPM_CRITICAL + 1000; rpm += 10){
			uint8_t rpm_level = 10 - ((RPM_CRITICAL - rpm) / RPM_LEVEL_RANGE); //get_rpm_level();
			if(!(rpm_level < 11))
				rpm_level = 0;
			else if(rpm > RPM_CRITICAL)
				rpm_level = 11;


			for(uint8_t led = 0; led < 5; led++){
				if(rpm_level > led)
					set_led(led, 0, 0xF, 0);
			}

			for(uint8_t led = 5; led < 10; led++){
				if(rpm_level > led)
					set_led(led, 0xf, 0, 0);
			}

			if(rpm_level == 11){
				for(uint8_t led = 10; led < 15; led++){
					set_led(led, 0, 0, 0xf);
				}
			}

			HAL_Delay(10);
		}
	}
}
