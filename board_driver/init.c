#include <stm32l4xx_hal.h>

#include <stdint.h>
#include <stdbool.h>

#include "init.h"


static void enable_instruction_cache() {
    SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
}

static void enable_data_cache() {
    SET_BIT(FLASH->ACR, FLASH_ACR_DCEN);
}

static void enable_prefetch_buffer() {
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTEN);
}

void init_board() {
    enable_instruction_cache();
    enable_data_cache();
    enable_prefetch_buffer();

    // Set Interrupt Group Priority
    NVIC_SetPriorityGrouping(3);

    HAL_InitTick(TICK_INT_PRIORITY);
}

static void start_pwr_clock() {
    SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);
    // Delay after PWR clock enabling
    (void) READ_BIT(RCC->APB1ENR1, RCC_APB1ENR1_PWREN);
}

static void pwr_voltage_scaling_config() {
    SET_BIT(PWR->CR1, PWR_CR1_VOS_0); // Set voltage regulator to scale 1
}

static void wait_until_hsi_ready() {
    while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET) {}
}

static void enable_hsi() {
    SET_BIT(RCC->CR, RCC_CR_HSION);

    wait_until_hsi_ready();
}

static void disable_pll() {
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RCC_CR_PLLRDY) {}
}

static void configure_pll() {
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk, RCC_PLLCFGR_PLLSRC_HSI);   // Set HSI as input source for PLL
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, 3 << RCC_PLLCFGR_PLLM_Pos);  // PLLM == 4
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, 40 << RCC_PLLCFGR_PLLN_Pos); // PLLN == 40
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk, 0 << RCC_PLLCFGR_PLLR_Pos);  // PLLR == 2
}

static void enable_pll() {
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET) {}
}

static void set_flash_latency() {
    // We set FLASH_LATENCY_5 as we are in vcc range 2.7-3.6 at 168mhz
    // See datasheet table 10 at page 80 (stm32l4xx reference manual)
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_4WS << FLASH_ACR_LATENCY_Pos);
}

static void configure_ahbclk() {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);
}

static void configure_sysclk() {
    // Set sysclk source to PLL
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
}

static void configure_apb1clk() {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_DIV1);
}

static void configure_apb2clk() {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV1);
}

void set_system_clock_168mhz(void) {

    start_pwr_clock();

    pwr_voltage_scaling_config();

    enable_hsi();
    disable_pll(); // To enable configuration of the pll
    configure_pll();
    enable_pll();

    set_flash_latency();

    configure_ahbclk();
    configure_sysclk();
    configure_apb1clk();
    configure_apb2clk();

    // Update CMSIS Core clk variable
    SystemCoreClockUpdate();

    // Update tick system to reflect clock changes
    HAL_InitTick(TICK_INT_PRIORITY);
}


void boot(uint32_t address) {
	uint32_t appStack = (uint32_t) *(uint32_t*)address;
	void (*appEntry)(void) = (void*)*(uint32_t*)(address + 4);

	__disable_irq();
	__set_MSP(appStack);
	appEntry();
}
