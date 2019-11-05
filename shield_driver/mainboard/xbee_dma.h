#ifndef XBEE_H
#define XBEE_H

#include <stm32f4xx_hal.h>

#define XBEE						UART4
#define XBEE_CLK_ENABLE()			__HAL_RCC_UART4_CLK_ENABLE()
#define XBEE_IRQn					UART4_IRQn
#define XBEE_IRQHandler				UART4_IRQHandler

#define XBEE_TX_PORT 				GPIOA
#define XBEE_TX_PIN					GPIO_PIN_0
#define XBEE_TX_AF					GPIO_AF8_UART4
#define XBEE_TX_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOA_CLK_ENABLE()
#define XBEE_TX_DMA_STREAM			DMA1_Stream2
#define XBEE_TX_DMA_CHANNEL			DMA_CHANNEL_4
#define XBEE_TX_DMA_CLK_ENABLE()	__HAL_RCC_DMA1_CLK_ENABLE()
#define XBEE_TX_IRQn				DMA1_Stream2_IRQn
#define XBEE_TX_IRQHandler			DMA1_Stream2_IRQHandler

#define XBEE_RX_PORT				GPIOA
#define XBEE_RX_PIN					GPIO_PIN_1
#define XBEE_RX_AF					GPIO_AF8_UART4
#define XBEE_RX_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define XBEE_RX_DMA_STREAM			DMA1_Stream4
#define XBEE_RX_DMA_CHANNEL			DMA_CHANNEL_4
#define XBEE_RX_DMA_CLK_ENABLE()	__HAL_RCC_DMA1_CLK_ENABLE()
#define XBEE_RX_IRQn				DMA1_Stream4_IRQn
#define XBEE_RX_IRQHandler			DMA1_Stream4_IRQHandler

HAL_StatusTypeDef xbee_transmit(uint8_t* data, uint8_t length);
HAL_StatusTypeDef init_xbee();

#endif
