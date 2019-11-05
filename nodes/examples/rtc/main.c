#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <board_driver/uart.h>
#include <board_driver/rtc.h>


int main(void) {
	uart_init();
	printf("Starting\n\n");
	printf("RTC status: %d\n", BSP_RTC_Init());

	while (1) {
		Date_Time_t now;
		RTC_Get_Date_Time(&now);
		printf("time: %02d:%02d:%02d\n", now.hours, now.minutes, now.seconds);
		HAL_Delay(1000);
	}
}
