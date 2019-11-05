#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stm32f4xx.h>

void rng_init()
{
	//Enable random number clock
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_RNGEN);

    //Enable random number generator on Chip
	SET_BIT(RNG->CR, RNG_CR_RNGEN);
}

//Includes min, but excludes max
uint32_t get_rng(int min, int max)
{
	return (RNG->DR % (max - min)) + min;
}
