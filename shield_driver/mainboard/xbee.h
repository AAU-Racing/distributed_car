#ifndef XBEE_H
#define XBEE_H

#include <stm32f4xx_hal.h>

#define XBEE                           UART4
#define XBEE_CLK_ENABLE()              __HAL_RCC_UART4_CLK_ENABLE()
#define XBEE_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define XBEE_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define XBEE_FORCE_RESET()             __HAL_RCC_UART4_FORCE_RESET()
#define XBEE_RELEASE_RESET()           __HAL_RCC_UART4_RELEASE_RESET()

/* Definition for XBEE Pins */
#define XBEE_TX_PIN                    GPIO_PIN_0
#define XBEE_TX_GPIO_PORT              GPIOA
#define XBEE_TX_AF                     GPIO_AF8_UART4
#define XBEE_RX_PIN                    GPIO_PIN_1
#define XBEE_RX_GPIO_PORT              GPIOA
#define XBEE_RX_AF                     GPIO_AF8_UART4

/* Definition for XBEE's NVIC */
#define XBEE_IRQn                      UART4_IRQn
#define XBEE_IRQHandler                UART4_IRQHandler


void HAL_UART_MspInit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef init_xbee(void);
HAL_StatusTypeDef xbee_transmit(uint8_t* buf, size_t n);
size_t UARTx_Receive(uint8_t *pData, uint16_t size);


#endif /* XBEE_H */
