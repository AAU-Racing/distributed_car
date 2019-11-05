#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>

#include <board_driver/usb/usb.h>
#include <board_driver/iwdg.h>


int main(void) {
	usb_init();
    HAL_Delay(1000);
    start_iwdg();
    printf("Starting\r\n");

    while (1) {
		printf("Hello usb\r\n");
        HAL_Delay(500);
	}
}
