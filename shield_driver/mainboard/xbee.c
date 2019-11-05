#include <stm32f4xx_hal.h>
#include <stdio.h>

#include "xbee.h"

static UART_HandleTypeDef UartHandle;

static uint32_t lastTransmit = 0;

HAL_StatusTypeDef init_xbee(void) {
    UartHandle.Instance = XBEE;

    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	XBEE_TX_GPIO_CLK_ENABLE();
	XBEE_RX_GPIO_CLK_ENABLE();
	XBEE_CLK_ENABLE();

	GPIO_InitTypeDef gpio_init = {
		.Pin       = XBEE_TX_PIN,
		.Mode      = GPIO_MODE_AF_PP,
		.Pull      = GPIO_PULLUP,
		.Speed     = GPIO_SPEED_FAST,
		.Alternate = XBEE_TX_AF,
	};
	HAL_GPIO_Init(XBEE_TX_GPIO_PORT, &gpio_init);
	gpio_init.Pin       = XBEE_RX_PIN;
	gpio_init.Alternate = XBEE_RX_AF;
	HAL_GPIO_Init(XBEE_RX_GPIO_PORT, &gpio_init);

	// Insert rx / tx DMA here

	HAL_NVIC_SetPriority(XBEE_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(XBEE_IRQn);

    return HAL_UART_Init(&UartHandle);
}

HAL_StatusTypeDef xbee_transmit(uint8_t* buf, size_t n) {
	if (HAL_GetTick() - lastTransmit < 100) {
		return HAL_ERROR;
	}

	lastTransmit = HAL_GetTick();

    return HAL_UART_Transmit(&UartHandle, buf, n, 10);
}

void XBEE_IRQHandler(void) {
    HAL_UART_IRQHandler(&UartHandle);
}
