#include <stm32f4xx_hal.h>


#define enable_all_interrupts() __enable_irq()
#define diable_all_interrupts() __disable_irq()
#define set_interrupt_offset(addr) SCB->VTOR = addr
