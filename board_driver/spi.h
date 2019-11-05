#ifndef SPI_H
#define SPI_H

#include <stm32f4xx_hal.h>

#define SPIx                               SPI1
#define SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_FORCE_RESET()				 	__HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()				__HAL_RCC_SPI1_RELEASE_RESET()

#define SPIx_SCK_AF                        GPIO_AF5_SPI1
#define SPIx_SCK_GPIO_PORT                 GPIOA
#define SPIx_SCK_PIN                       GPIO_PIN_5
#define SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

#define SPIx_MOSI_AF                  		GPIO_AF5_SPI1
#define SPIx_MOSI_GPIO_PORT           		GPIOA
#define SPIx_MOSI_PIN						GPIO_PIN_7
#define SPIx_MOSI_GPIO_CLK_ENABLE()   		__HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_DISABLE()  		__HAL_RCC_GPIOA_CLK_DISABLE()


void SPI_init(void);
HAL_StatusTypeDef SPI_transmit(void *tx, uint32_t size);

#endif /* SPI_H */
