#ifndef BKPSRAM_GUARD_H
#define BKPSRAM_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#define BKPSRAM_GEAR 0

void init_bkpsram();
bool bkpsram_was_enabled_last_run();
uint32_t read_bkpsram(uint16_t relative_address);
void write_bkpsram(uint16_t relative_address, uint32_t value);

#endif
