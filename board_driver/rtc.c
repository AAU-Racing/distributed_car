#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include <board_driver/uart.h>
#include <board_driver/rtc.h>


RTC_HandleTypeDef RTCHandle;
RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCdate;
RTC_TypeDef *RTCBR = RTC;

//This function initialises the hardware used by the RTC. It is automatically called by HAL_RTC_Init()
void HAL_RTC_MspInit(RTC_HandleTypeDef *RTCHandle)
{
	(void)RTCHandle;
	__HAL_RCC_PWR_CLK_ENABLE();					//Enables the Power Controller (PWR) APB1 interface clock using
	HAL_PWR_EnableBkUpAccess();					//Enables access to the RTC domain(RTC registers, RTC backup data registers and backup SRAM)
	__HAL_RCC_LSE_CONFIG(RCC_LSE_ON);			//Turns on the LSE. Without this line we have unpredictable behaviour
	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);	//Select the RTC clock source
	__HAL_RCC_RTC_ENABLE();						//Enable RTC Clock
}

//This function configures the RTC Prescaler and RTC hour format using the HAL_RTC_Init() function
//Then it sets the date and time
int BSP_RTC_Init()
{
	RTCHandle.Instance 			= RTC;
	RTCHandle.Init.HourFormat 	= RTC_HOURFORMAT_24;
	RTCHandle.Init.AsynchPrediv = PRESCALER1;
	RTCHandle.Init.SynchPrediv 	= PRESCALER2;
	RTCHandle.Init.OutPut 		= RTC_OUTPUT_DISABLE;
	RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RTCHandle.Init.OutPutType 	= RTC_OUTPUT_TYPE_OPENDRAIN;
	HAL_RTC_Init(&RTCHandle);
	
	uint32_t unixtime = RTC_UNIX_INIT();
	uint32_t RTC_Time = RTC_Get_Time_Unix();

	//Compares compiled time with RTC time
	if(unixtime > RTC_Time)
		RTC_Update_Date_Time(unixtime); //If compile time is the newest, use this as new RTC time

	return HAL_RTC_GetState(&RTCHandle) == HAL_RTC_STATE_READY ? 0 : -1;
}

//Gets computer time and converts to unix timestamp
uint32_t RTC_UNIX_INIT()
{
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

    return mktime(&ti);
}

//This function gets the current date and time
//IMPORTANT: Both HAL_RTC_GetTime and HAL_RTC_GetDate have to be called in order to get the corect values
void RTC_Get_Date_Time(Date_Time_t* now)
{
	HAL_RTC_GetTime(&RTCHandle, &RTCtime, FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandle, &RTCdate, FORMAT_BIN);
	now->subseconds = RTCtime.SubSeconds;
	now->seconds 	= RTCtime.Seconds;
	now->minutes 	= RTCtime.Minutes;
	now->hours 		= RTCtime.Hours;
	now->date 		= RTCdate.Date;
	now->month		= RTCdate.Month;
	now->year 		= RTCdate.Year;
}

//Converts RTC Time to Unix Timestamp 
uint32_t RTC_Get_Time_Unix()
{
	struct tm ti;

	HAL_RTC_GetTime(&RTCHandle, &RTCtime, FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandle, &RTCdate, FORMAT_BIN);
	ti.tm_sec 	= RTCtime.Seconds;
	ti.tm_min 	= RTCtime.Minutes;
	ti.tm_hour = RTCtime.Hours;
	ti.tm_mday	= RTCdate.Date;
	ti.tm_mon	= RTCdate.Month;
	ti.tm_year	= RTCdate.Year;

	return mktime(&ti);
}

//Sets new RTC time
void RTC_Update_Date_Time(uint32_t unixTime){

    time_t t = unixTime;
    struct tm ti;
    localtime_r(&t, &ti);

    RTCtime.Hours       = (uint8_t)ti.tm_hour;
    RTCtime.Minutes     = (uint8_t)ti.tm_min;
    RTCtime.Seconds     = (uint8_t)ti.tm_sec;
    RTCtime.SubSeconds  = ((uint8_t)0);

    RTCdate.Month       = (uint8_t)(ti.tm_mon);
    RTCdate.Date        = (uint8_t)ti.tm_mday;
    RTCdate.Year        = (uint8_t)ti.tm_year;

    HAL_RTC_SetTime(&RTCHandle, &RTCtime, FORMAT_BIN);
    HAL_RTC_SetDate(&RTCHandle, &RTCdate, FORMAT_BIN);

}
