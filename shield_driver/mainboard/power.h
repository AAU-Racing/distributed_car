#include <stm32f4xx_hal.h>
#include <stdbool.h>

#define POWER_INTERUPT_PIN                    PIN_8
#define POWER_INTERUPT_GPIO_PORT              GPIOE


void power_interupt_pin_init(void);
bool power_get_state();
