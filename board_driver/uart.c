#include <stm32f4xx_hal.h>
#include <stdio.h>

#include "uart.h"
#include "ringbuffer.h"

#define BUF_SIZE 1024

UART_HandleTypeDef UartHandle;
ringbuffer_t uartx_rec;
ringbuffer_t uartx_send;
uint8_t uartx_rec_buf[BUF_SIZE];
uint8_t uartx_send_buf[BUF_SIZE];


void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	(void)huart;

	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	USARTx_CLK_ENABLE();

	GPIO_InitTypeDef gpio_init = {
		.Pin       = USARTx_TX_PIN,
		.Mode      = GPIO_MODE_AF_PP,
		.Pull      = GPIO_PULLUP,
		.Speed     = GPIO_SPEED_FAST,
		.Alternate = USARTx_TX_AF,
	};
	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &gpio_init);
	gpio_init.Pin       = USARTx_RX_PIN;
	gpio_init.Alternate = USARTx_RX_AF;
	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &gpio_init);

	// Insert rx / tx DMA here

	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	(void)huart;

	USARTx_FORCE_RESET();
	USARTx_RELEASE_RESET();

	HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
	HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
}

void uart_init(void) {
	UartHandle.Instance = USARTx;

	UartHandle.Init.BaudRate     = 115200;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	rb_init(&uartx_rec, uartx_rec_buf, BUF_SIZE);
	rb_init(&uartx_send, uartx_send_buf, BUF_SIZE);
	HAL_UART_Init(&UartHandle);
	SET_BIT(UartHandle.Instance->CR1, USART_CR1_RXNEIE);
}

void USARTx_IRQHandler(void) {
	uint32_t isrflags = READ_REG(UartHandle.Instance->SR);

	if (isrflags & USART_SR_RXNE) {
		uint8_t data_in = (uint8_t)(UartHandle.Instance->DR & (uint8_t)0x00FF);
		rb_push(&uartx_rec, data_in);
	}

	if (isrflags & USART_SR_TXE) {
		uint8_t data_out;
		if (rb_pop(&uartx_send, &data_out)) {
			CLEAR_BIT(UartHandle.Instance->CR1, USART_CR1_TXEIE);
		} else {
			UartHandle.Instance->DR = data_out & (uint8_t)0x00FF;
		}
	}
}

uint8_t uart_read_byte(void) {
	uint8_t data;
	while (rb_pop(&uartx_rec, &data));
	return data;
}

void uart_send_byte(uint8_t data) {
	while (rb_isFull(&uartx_send));
	rb_push(&uartx_send, data);
	SET_BIT(UartHandle.Instance->CR1, USART_CR1_TXEIE);
}

void uart_send_buf(uint8_t *data, size_t n) {
	for (size_t i = 0; i < n; i++) {
		uart_send_byte(data[i]);
	}
}
