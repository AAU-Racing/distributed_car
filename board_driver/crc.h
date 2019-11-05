#ifndef CRC_H
#define CRC_H

void crc_init(void);
uint32_t crc_calculate(uint32_t buffer[], uint8_t n);
#endif
