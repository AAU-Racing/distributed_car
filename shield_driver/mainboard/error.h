#include <stm32l4xx_hal.h>
#include <stdbool.h>

bool get_error(uint16_t *id, uint32_t *error_id);
HAL_StatusTypeDef error_init(void);
