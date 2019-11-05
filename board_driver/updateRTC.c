#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include <board_driver/uart.h>
#include <board_driver/rtc.h>

int main(void) {
	uart_init();
	printf("Starting\n\n");
	printf("RTC status: %d\n", BSP_RTC_Init());

	struct tm ti;

	char month[3];

	sscanf(__DATE__, "%s %d %d", month, &ti.tm_mday, &ti.tm_year);

	ti.tm_year -= 1900;

	if(!strcmp(month, "Jan"))
		ti.tm_mon = 1;
	else if(!strcmp(month, "Feb"))
		ti.tm_mon = 2;
	else if(!strcmp(month, "Mar"))
		ti.tm_mon = 3;
	else if(!strcmp(month, "Apr"))
		ti.tm_mon = 4;
	else if(!strcmp(month, "May"))
		ti.tm_mon = 5;
	else if(!strcmp(month, "Jun"))
		ti.tm_mon = 6;
	else if(!strcmp(month, "Jul"))
		ti.tm_mon = 7;
	else if(!strcmp(month, "Aug"))
		ti.tm_mon = 8;
	else if(!strcmp(month, "Sep"))
		ti.tm_mon = 9;
	else if(!strcmp(month, "Oct"))
		ti.tm_mon = 10;
	else if(!strcmp(month, "Nov"))
		ti.tm_mon = 11;
	else if(!strcmp(month, "Dec"))
		ti.tm_mon = 12;

	sscanf(__TIME__, "%d:%d:%d", &ti.tm_hour, &ti.tm_min, &ti.tm_sec);

	uint32_t unixTime = mktime(&ti);
	RTC_Update_Date_Time(unixTime);

	while (1) {
		Date_Time_t now;
		RTC_Get_Date_Time(&now);
		printf("time: %02d:%02d:%02d, %02d/%02d/%02d\n", now.hours, now.minutes, now.seconds, now.date, now.month, (now.year + 1900));
		HAL_Delay(1000);
	}
}
