#include <string.h>

#include "xbee.h"

#define RXBUFFERSIZE 100
#define TXBUFFERSIZE 104

static UART_HandleTypeDef UartHandle;
//static uint8_t rxBuffer[RXBUFFERSIZE] = {0};
static uint8_t txBuffer[TXBUFFERSIZE] = {0};
static uint32_t lastTransmit = 0;

HAL_StatusTypeDef init_xbee() {
	UartHandle.Instance			 = XBEE;
    UartHandle.Init.BaudRate     = 115200;
    UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
    UartHandle.Init.StopBits     = UART_STOPBITS_1;
    UartHandle.Init.Parity       = UART_PARITY_NONE;
    UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    UartHandle.Init.Mode         = UART_MODE_TX_RX;
    UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_StatusTypeDef result = HAL_UART_Init(&UartHandle);

	if(result != HAL_OK) {
		return result;
	}

	static DMA_HandleTypeDef hdma_tx;
	static DMA_HandleTypeDef hdma_rx;

	// Clock
	XBEE_TX_GPIO_CLK_ENABLE();
	XBEE_TX_DMA_CLK_ENABLE();
	XBEE_RX_GPIO_CLK_ENABLE();
	XBEE_RX_DMA_CLK_ENABLE();
	XBEE_CLK_ENABLE();

	HAL_GPIO_Init(XBEE_TX_PORT, &(GPIO_InitTypeDef) {
		.Pin	   = XBEE_TX_PIN,
		.Mode 	   = GPIO_MODE_AF_PP,
		.Pull	   = GPIO_PULLUP,
		.Speed 	   = GPIO_SPEED_FAST,
		.Alternate = XBEE_TX_AF
	});

	HAL_GPIO_Init(XBEE_RX_PORT, &(GPIO_InitTypeDef) {
		.Pin	   = XBEE_RX_PIN,
		.Mode	   = GPIO_MODE_AF_PP,
		.Pull	   = GPIO_PULLUP,
		.Speed	   = GPIO_SPEED_FAST,
		.Alternate = XBEE_RX_AF
	});

	// DMA TX
	hdma_tx.Instance                 = XBEE_TX_DMA_STREAM;

	hdma_tx.Init.Channel             = XBEE_TX_DMA_CHANNEL;
	hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode                = DMA_NORMAL;
	hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
	hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

	HAL_DMA_Init(&hdma_tx);

	__HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);

	// DMA RX
	hdma_rx.Instance                 = XBEE_RX_DMA_STREAM;

	hdma_rx.Init.Channel             = XBEE_RX_DMA_CHANNEL;
	hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_rx.Init.Mode                = DMA_NORMAL;
	hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
	hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_rx.Init.MemBurst            = DMA_MBURST_INC4;
	hdma_rx.Init.PeriphBurst         = DMA_PBURST_INC4;

	HAL_DMA_Init(&hdma_rx);

	__HAL_LINKDMA(&UartHandle, hdmarx, hdma_rx);

	HAL_NVIC_SetPriority(XBEE_TX_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(XBEE_TX_IRQn);

	HAL_NVIC_SetPriority(XBEE_RX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(XBEE_RX_IRQn);

	HAL_NVIC_SetPriority(XBEE_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(XBEE_IRQn);

	return HAL_OK;
	//return HAL_UART_Receive_DMA(&UartHandle, rxBuffer, RXBUFFERSIZE);
}

HAL_StatusTypeDef xbee_transmit(uint8_t* data, uint8_t length) {
	if (length > TXBUFFERSIZE - 4) {
		return HAL_ERROR;
	}

	if (HAL_GetTick() - lastTransmit < 100) {
		return HAL_ERROR;
	}

	lastTransmit = HAL_GetTick();

	memset(txBuffer, '\0', TXBUFFERSIZE);

	memcpy(txBuffer, "A1", 2);
	memcpy(txBuffer + length + 2, "B2", 2);

	memcpy(txBuffer + 2, data, length);

	return HAL_UART_Transmit_DMA(&UartHandle, txBuffer, length + 4);
}

void XBEE_RX_IRQHandler(void)
{
	printf("1\n");
	HAL_DMA_IRQHandler(UartHandle.hdmarx);
}

void XBEE_TX_IRQHandler(void)
{
	printf("2\n");
	HAL_DMA_IRQHandler(UartHandle.hdmatx);
}

void XBEE_IRQHandler(void)
{
	printf("3\n");
	HAL_UART_IRQHandler(&UartHandle);
}
