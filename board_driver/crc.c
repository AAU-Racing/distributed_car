#include <stm32f4xx.h>


#define crc_reset() (CRC->CR = 1)
// Use __RBIT instead of __REV, because it reverse the bits correct.
// Big-Endian is used to reverse the bits so the largest significant bit is first.
// 0x12345678 is reversed to 0x87654321

// If you want to calculate it on an online crc32 calculater,
// 0x12345678 has to get manualy reversed to 0x78 0x56 0x34 0x12

#define crc_write(x) {CRC->DR = __RBIT(x);}
// __RBIT to reverse bits correct and XOR with 0xFFFFFFFF
#define crc_read() (__RBIT(CRC->DR) ^ 0xFFFFFFFF)

void crc_init (){
    // Enable clock
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_CRCEN);
}

uint32_t crc_calculate(uint32_t *buffer, int n){
    //Reset CRC register
    crc_reset();
    //SET_BIT(CRC->CR, CRC_CR_RESET);

    // For all data
    for (uint8_t i = 0; i < n; i++) {
        // Accumulate CRC
        crc_write(buffer[i]);
    }

    // Result in data register
    return crc_read();
}
