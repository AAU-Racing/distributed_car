#include "bkpsram.h"

#include <stm32f4xx.h>

static bool was_enabled;

static void enable_bkp_sram() {
    SET_BIT(PWR->CR, PWR_CR_DBP); // This also disables write protection for RTC
}

static void start_bkp_sram_clock() {
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_BKPSRAMEN);
}

static bool backup_regulator_is_ready() {
    return READ_BIT(PWR->CSR, PWR_CSR_BRR);
}

static void enable_backup_regulator() {
    if (!backup_regulator_is_ready()) {
        SET_BIT(PWR->CSR, PWR_CSR_BRE);
    }

    while (!backup_regulator_is_ready()) { }
}

void init_bkpsram() {
    // Assume PWR controller was enabled by the init code

    enable_bkp_sram();

    start_bkp_sram_clock();

    was_enabled = backup_regulator_is_ready();

    enable_backup_regulator();
}

bool bkpsram_was_enabled_last_run() {
    return was_enabled;
}

uint32_t read_bkpsram(uint16_t relative_address) {
    return *((__IO uint32_t*) (BKPSRAM_BASE + relative_address));
}

void write_bkpsram(uint16_t relative_address, uint32_t value) {
    *((__IO uint32_t*) (BKPSRAM_BASE + relative_address)) = value;
}
