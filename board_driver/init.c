#include <stm32f4xx_hal.h>

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
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
    // Delay after PWR clock enabling
    (void) READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
}

static void pwr_voltage_scaling_config() {
    SET_BIT(PWR->CR, PWR_CR_VOS); // Set voltage regulator to scale 1
}

static void wait_until_hse_ready() {
    while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET) {}
}

static void enable_hse() {
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    wait_until_hse_ready();
}

static void disable_pll() {
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RCC_CR_PLLRDY) {}
}

static void configure_pll() {
	const uint32_t HSE_mhz = HSE_VALUE / 10e5;
    const bool HSE_is_odd  = (HSE_mhz & 1) == 1;

    // Sysclk is ((HSE / PLLM) * PLLN) / PLLP -- ((x / x) * 336) / 2 = 168
    // USBclk is ((HSE / PLLM) * PLLN) / PLLQ -- ((x / x) * 336) / 7 = 48

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC); // Set HSE as input source for PLL
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, (HSE_is_odd ? HSE_mhz : (HSE_mhz / 2)) << RCC_PLLCFGR_PLLM_Pos); // should match HSE freq in mhz so HSE / PLLM == 1
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, (HSE_is_odd ? (160 * 2) : 160) << RCC_PLLCFGR_PLLN_Pos); // 320 / 2 == 160 == max core clk
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLP_Msk, 0 << RCC_PLLCFGR_PLLP_Pos); // PLLP division factor = 2, ((HSE/pllm) * plln) / pllp == 160 (max core clk)
    MODIFY_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLQ_Msk, 7 << RCC_PLLCFGR_PLLQ_Pos); // ((HSE/pllm) * plln) / pllq == 48 (USB needs 48 and sdio needs 48 or lower)
}

static void enable_pll() {
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET) {}
}

static void set_flash_latency() {
    // We set FLASH_LATENCY_5 as we are in vcc range 2.7-3.6 at 168mhz
    // See datasheet table 10 at page 80 (stm32f4xx reference manual)
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY_Msk, FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos);
}

static void configure_ahbclk() {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE_Msk, RCC_CFGR_HPRE_DIV1);
}

static void configure_sysclk() {
    // Set sysclk source to PLL
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW_Msk, RCC_CFGR_SW_PLL);
}

static void configure_apb1clk() {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1_Msk, RCC_CFGR_PPRE1_DIV4);
}

static void configure_apb2clk() {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2_Msk, RCC_CFGR_PPRE2_DIV2);
}

void set_system_clock_168mhz(void) {

    start_pwr_clock();

    pwr_voltage_scaling_config();

    enable_hse();
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
