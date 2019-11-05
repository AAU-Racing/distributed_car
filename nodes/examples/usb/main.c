#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>
#include <board_driver/usb/usb.h>


int main(void) {
	usb_init();
	HAL_Delay(3000);

	printf("Starting\n");

	/* Remember getchar also reads \n and
	will wait until it recieves a newline before returning */
	while (1) {
		char c;
		printf( "Enter a value :\n");
		c = getchar();
		printf( "You entered: ");
		putchar(c);
		putchar('\n');
	}
}
