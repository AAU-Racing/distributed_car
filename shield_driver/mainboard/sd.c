#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <ff_gen_drv.h>
#include <sd_diskio.h>
#include <board_driver/rtc.h>
#include "sd.h"

#define FILE_VERSION 1

void printError(FRESULT rc);

void create_log(char *file_name) {
	create_file(file_name, "LOG", "CSV");
}

void create_error(char *file_name) {
	create_file(file_name, "ERROR", "CSV");
}

void create_file(char *file_name, char *prefix, char *extension) {
	(void)BSP_RTC_Init();
	FIL f;
	uint16_t i = 0;

	do {
		sprintf(file_name, "%s%u.%s", prefix, i, extension);
		i++;
	}
	while (f_open(&f, file_name, FA_CREATE_NEW | FA_WRITE) != FR_OK);

	unsigned bw;
	Date_Time_t now;
	RTC_Get_Date_Time(&now);

	char time_str[27] = {'\0'};
	sprintf(time_str, "time: %02d:%02d:%02d, %02d/%02d/%02d\n", now.hours, now.minutes, now.seconds, now.date, now.month, (now.year + 1900));

	f_write(&f, &time_str, 27, &bw);

	f_sync(&f);
	f_close(&f);
}

void writeTime(char *file_name, uint32_t time) {
	FIL f;
	unsigned int bw;

	f_open(&f, file_name, FA_WRITE|FA_OPEN_EXISTING);
	f_lseek(&f, 1);
	f_write(&f, &time, 4, &bw);

	f_sync(&f);
	f_close(&f);
}

void writeStr(char *file_name, char *str, uint8_t len) {
	FIL f;
	unsigned int bw;

	f_open(&f, file_name, FA_WRITE|FA_OPEN_EXISTING);
	f_lseek(&f, 1+4);
	f_write(&f, str, len, &bw);

	f_sync(&f);
	f_close(&f);
}

void appendData(char *file_name, void *data, uint16_t len) {
	FIL f;
	unsigned int bw;

	f_open(&f, file_name, FA_WRITE|FA_OPEN_EXISTING);
	f_lseek(&f, f_size(&f));
	f_write(&f, data, len, &bw);

	f_sync(&f);
	f_close(&f);
}

void printError(FRESULT rc) {
	switch (rc) {
		case FR_OK:
			printf("FR_OK - Operation succeded\n");
			break;
		case FR_DISK_ERR:
			printf("FR_DISK_ERR - A hard error occured in the low level disk I/O layer.\n");
			break;
		case FR_INT_ERR:
			printf("FR_INT_ERR - Assertion failed.\n");
			break;
		case FR_NOT_READY:
			printf("FR_NOT_READY - A physical drive cannot work.\n");
			break;
		case FR_NO_FILE:
			printf("FR_NO_FILE - Could not find the file.\n");
			break;
		case FR_NO_PATH:
			printf("FR_NO_PATH - Could not find the path.\n");
			break;
		case FR_INVALID_NAME:
			printf("FR_INVALID_NAME - The path name format is invalid.\n");
			break;
		case FR_DENIED:
			printf("FR_DENIED - Access denied due to prohibited access or directory full.\n");
			break;
		case FR_EXIST:
			printf("FR_EXIST - Access denied due to prohibited access.\n");
			break;
		case FR_INVALID_OBJECT:
			printf("FR_INVALID_OBJECT - The file/object is invalid.\n");
			break;
		case FR_WRITE_PROTECTED:
			printf("FR_WRITE_PROTECTED - The physical drive is write protected.\n");
			break;
		case FR_INVALID_DRIVE:
			printf("FR_INVALID_DRIVE - The logical drive number is invalid.\n");
			break;
		case FR_NOT_ENABLED:
			printf("FR_NOT_ENABLED - The volume has no work area.\n");
			break;
		case FR_NO_FILESYSTEM:
			printf("FR_NO_FILESYSTEM - There is no valid FAT volume on the physical drive.\n");
			break;
		case FR_MKFS_ABORTED:
			printf("FR_MKFS_ABORTED - The f_mkfs() aborted due to any parameter error.\n");
			break;
		case FR_TIMEOUT:
			printf("FR_TIMEOUT - Could not get a grant to access the volume within defined period.\n");
			break;
		case FR_LOCKED:
			printf("FR_LOCKED - The operation is rejected according to the file sharing policy.\n");
			break;
		case FR_NOT_ENOUGH_CORE:
			printf("FR_NOT_ENOUGH_CORE - LFN working buffer could not be allocated.\n");
			break;
		case FR_TOO_MANY_OPEN_FILES:
			printf("FR_TOO_MANY_OPEN_FILES - Number of open files > FS_SHARE.\n");
			break;
		case FR_INVALID_PARAMETER:
			printf("FR_INVALID_PARAMETER - Given parameter is invalid.\n");
	}

}
