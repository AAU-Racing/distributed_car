#include <stm32f4xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

#define RPM_CRITICAL 13000
#define RPM_LEVEL_RANGE 200

int get_rpm_level();
bool get_rpm(uint16_t* rpm);
