#include <stm32f4xx_hal.h>

#include "spi.h"

#define TX_TIMEOUT_MS	(1000)

static SPI_HandleTypeDef spi_handle;

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	(void)hspi; // unused parameter

	SPIx_SCK_GPIO_CLK_ENABLE();
	SPIx_MOSI_GPIO_CLK_ENABLE();
	SPIx_CLK_ENABLE();

	// Configure SPI SCK
	GPIO_InitTypeDef gpio = {
		.Pin       = SPIx_SCK_PIN,
		.Mode      = GPIO_MODE_AF_PP,
		.Pull      = GPIO_NOPULL,
		.Speed     = GPIO_SPEED_FAST,
		.Alternate = SPIx_SCK_AF,
	};
	HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &gpio);

	// Configure SPI MOSI
	gpio.Pin       = SPIx_MOSI_PIN;
	gpio.Alternate = SPIx_MOSI_AF;
	gpio.Pull      = GPIO_NOPULL;
	HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &gpio);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
	(void) hspi;

	SPIx_FORCE_RESET();
	SPIx_RELEASE_RESET();

	HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
	HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);
}

void SPI_init(void) {
	if (HAL_SPI_GetState(&spi_handle) == HAL_SPI_STATE_RESET) {
		spi_handle.Instance                = SPIx;

		spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
		spi_handle.Init.Direction         = SPI_DIRECTION_2LINES;
		spi_handle.Init.CLKPhase          = SPI_PHASE_2EDGE;
		spi_handle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
		spi_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
		spi_handle.Init.CRCPolynomial     = 7;
		spi_handle.Init.DataSize          = SPI_DATASIZE_8BIT;
		spi_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
		spi_handle.Init.NSS               = SPI_NSS_SOFT;
		spi_handle.Init.TIMode            = SPI_TIMODE_DISABLED;
		spi_handle.Init.Mode              = SPI_MODE_MASTER;

		HAL_SPI_Init(&spi_handle);
	}
}

HAL_StatusTypeDef SPI_transmit(void *tx, uint32_t size) {
	return HAL_SPI_Transmit(&spi_handle, tx, size, TX_TIMEOUT_MS);
}
