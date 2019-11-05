#include <stm32f4xx_hal.h>

#include <board_driver/can.h>

bool oil_pressure_ok();
HAL_StatusTypeDef oil_init(void);
