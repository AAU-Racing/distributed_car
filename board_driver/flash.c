#include <stm32f4xx_hal.h>

#include <stdint.h>
#include <stddef.h>

#include "flash.h"


int write_flash(uint32_t start_address, uint8_t *data, size_t len) {
	//unlock flash writing
	HAL_FLASH_Unlock();

	//clear flags.
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	//Erase sector.
	FLASH_Erase_Sector(FLASH_SECTOR_4, VOLTAGE_RANGE_3);

	for(size_t i = 0; i < len; i++) {
		if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, start_address++, data[i]) != HAL_OK) {
			HAL_FLASH_Lock();
			return 1;
		}
	}

	HAL_FLASH_Lock();

	return 0;
}
