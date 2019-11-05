#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>

#include <board_driver/uart.h>
#include <board_driver/crc.h>

int main(void) {
	// Test the hex value 0x12345678
	uint32_t buf[] = {0x12345678};
	// The test array lenght is = 1
	size_t n = 1;
	// Set the result to a random value
	uint32_t result = 1234;
	// The result for 0x12345678 = 0xAF6D87D2
	uint32_t expecthex = 0xAF6D87D2;

	// Init uart and crc
	uart_init();
	crc_init();

	printf("crc init complete\n");

	HAL_Delay(100);

	// Override the new calculated crc result to result
	result = crc_calculate(buf, n);
	printf("crc calculate complete\n");

	// Print the result and expected
	printf("Result = %x\n", (unsigned int) result);
	printf("Result expected HEX= %x\n", (unsigned int) expecthex);

	while(1);
}
