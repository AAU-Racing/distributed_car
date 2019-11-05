#include <stm32f4xx_hal.h>

#include <shield_driver/mainboard/xbee.h>
#include <board_driver/uart.h>

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

int main(void) {
	uart_init();
	init_xbee();

	printf("init complete\n");

	HAL_Delay(10);

	while(1) {
		if (xbee_transmit((uint8_t*) "A1;127;125;-2;2;12.012;B2", strlen("A1;127;125;-2;2;12.012;B2")) == HAL_OK) {
			printf("Transmitting\n");
		}
		HAL_Delay(10);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	(void) UartHandle;
	printf("TX cplt\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	(void) UartHandle;
	printf("RX cplt\n");
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	(void) UartHandle;
	printf("Error\n");
}
