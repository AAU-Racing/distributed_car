#include <stm32f4xx.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "iwdg.h"

#define ENABLE_WRITE_ACCESS 0x5555
#define SET_PRESCALER32 0x03
#define SET_RELOAD_TIME 0xFFF
#define START_IDWG 0xCCCC
#define RESET_IWDG 0xAAAA

//Sets up the IWDG, by writing to the registers: Key, Prescale and reaload.
//If not key register is set to 0x5555 it is not allowed to set the prescaler and
//reload register.
void start_iwdg()
{
    IWDG->KR = START_IDWG;
    IWDG->KR = ENABLE_WRITE_ACCESS;
    IWDG->PR = SET_PRESCALER32;
    IWDG->RLR = SET_RELOAD_TIME;

    reset_iwdg();

    //Busy loop: Should not continue if the Reload Value Update (RVU) or the
    //Prescare Value Update (PVU) bit is set.
    while((IWDG->SR & IWDG_SR_PVU_Msk) && (IWDG->SR & IWDG_SR_RVU_Msk)) { }
}

//At startup check if through IWDGRSTF if reset was caused by IWDG.
int was_reset_by_iwdg()
{
    return !!(RCC->CSR & RCC_CSR_IWDGRSTF_Msk);
}

//Refreshes the IWDG by writing 0xAAAA to the key register.
void reset_iwdg()
{
    IWDG->KR = RESET_IWDG;
}
