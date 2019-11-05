#include <stm32f4xx_hal.h>

#include "adc.h"

#define ADC_DMA_CHANNEL 			0
#define ADC_DMA_PERIPH_TO_MEMORY 	0
#define ADC_DMA_PERIPH_INC		 	DMA_SxCR_PINC
#define ADC_DMA_MEM_INC				DMA_SxCR_MINC
#define ADC_DMA_PDATAALIGN_WORD		DMA_SxCR_PSIZE_1
#define ADC_DMA_MDATAALIGN_WORD     DMA_SxCR_MSIZE_1
#define ADC_DMA_CIRCULAR			DMA_SxCR_CIRC
#define ADC_DMA_PRIORITY_HIGH		DMA_SxCR_PL_1

#define TEMPSENSOR_DELAY_US   10
#define STABILZATION_DELAY_US 10

static int sequence_number = 1;
__IO uint32_t values[16];
ADC_TypeDef* handle;
DMA_Stream_TypeDef* dma_stream;

static void clk_init() {
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
}

static void enable_adc() {
	SET_BIT(handle->CR2, ADC_CR2_ADON);
}

static void wait_for_stabilization() {
	// Delay for ADC stabilization time
	// Compute number of CPU cycles to wait for
	uint32_t counter = ADC_STAB_DELAY_US * 160; // 160 MHz core clock
	while(counter != 0U) {
		counter--;
	}
}

static void enable_adc_dma_mode() {
	SET_BIT(handle->CR2, ADC_CR2_DMA);
}

static void set_prescaler() {
	// Div 8 relative to APB2
	SET_BIT(ADC->CCR, ADC_CCR_ADCPRE);
}

static void enable_scan_conv() {
	SET_BIT(handle->CR1, ADC_CR1_SCAN);
}

static void set_resolution() {
	// 12 bit resolution
	MODIFY_REG(handle->CR1, ADC_CR1_RES_Msk, 0);
}

static void set_data_align() {
	// Align right
	CLEAR_BIT(handle->CR2, ADC_CR2_ALIGN);
}

static void enable_continuous_mode() {
	SET_BIT(handle->CR2, ADC_CR2_CONT);
}

static void enable_dma_continuous_mode() {
	SET_BIT(handle->CR2, ADC_CR2_DDS);
}

static void dma_init() {
	SET_BIT(dma_stream->CR, ADC_DMA_MEM_INC);
	CLEAR_BIT(dma_stream->CR, ADC_DMA_PERIPH_INC);
	MODIFY_REG(dma_stream->CR, DMA_SxCR_DIR, ADC_DMA_PERIPH_TO_MEMORY);
	MODIFY_REG(dma_stream->CR, DMA_SxCR_PSIZE_Msk, ADC_DMA_PDATAALIGN_WORD);
	MODIFY_REG(dma_stream->CR, DMA_SxCR_MSIZE_Msk, ADC_DMA_MDATAALIGN_WORD);
	MODIFY_REG(dma_stream->CR, DMA_SxCR_CHSEL, ADC_DMA_CHANNEL);
	MODIFY_REG(dma_stream->CR, DMA_SxCR_CHSEL, ADC_DMA_CHANNEL);
	SET_BIT(dma_stream->CR, ADC_DMA_CIRCULAR);
	MODIFY_REG(dma_stream->CR, DMA_SxCR_PL, ADC_DMA_PRIORITY_HIGH);

	dma_stream->FCR = 0;
}

void init_adc() {
	handle = ADC1;
	dma_stream = DMA2_Stream0;

	clk_init();
	enable_adc();
	wait_for_stabilization();
	enable_adc_dma_mode();
	set_prescaler();
	enable_scan_conv();
	set_resolution();
	set_data_align();
	enable_continuous_mode();
	enable_dma_continuous_mode();

	dma_init();
}

static void set_sampletime(ADC_Channel channel) {
	int shift = channel > CHANNEL_9 ? 10 : 0;
	int pos = 3 * (channel - shift);

	uint32_t sampletime = SAMPLETIME_56CYCLES << pos;

	if (channel > CHANNEL_9) {
		handle->SMPR1 |= sampletime;
	}
	else {
		handle->SMPR2 |= sampletime;
	}
}

static void set_sequence_channel_number(ADC_Channel channel) {
	// Shift 1 for sequence 1-6, shift 7 for sequence 7-12, else shift 13
	int shift = 1 + ((sequence_number - 1) / 6) * 6;
	int pos = 5 * (sequence_number - shift);

	uint32_t sequence = channel << pos;

	if (sequence_number < 7) {
		handle->SQR3 |= sequence;
	}
	else if (sequence_number < 13) {
		handle->SQR2 |= sequence;
	}
	else {
		handle->SQR1 |= sequence;
	}
}

static void enable_vbat() {
	SET_BIT(ADC->CCR, ADC_CCR_VBATE);
}

static void wait_for_temp_sensor_stabilization() {
	// Delay for temperature sensor stabilization time
	// Compute number of CPU cycles to wait for
	uint32_t counter = ADC_TEMPSENSOR_DELAY_US * 160; // 160 MHz core clock
	while (counter != 0) {
	  counter--;
	}
}

static void enable_tempsensor_and_vref(ADC_Channel channel) {
	SET_BIT(ADC->CCR, ADC_CCR_TSVREFE);

	if (channel == ADC_CHANNEL_TEMPSENSOR) {
		wait_for_temp_sensor_stabilization();
	}
}

void init_adc_channel(ADC_Channel channel, uint8_t *array_index) {
	*array_index = sequence_number - 1;

	set_sampletime(channel);
	set_sequence_channel_number(channel);


	if (channel == CHANNEL_VBAT) {
		enable_vbat();
	}

	if (channel == CHANNEL_TEMPSENSOR || channel == CHANNEL_VREFINT) {
		enable_tempsensor_and_vref(channel);
	}

	sequence_number++;
}

static void set_number_of_conversions() {
	// Notice off by one.
	int l_bits = sequence_number - 2;
	MODIFY_REG(handle->SQR1, ADC_SQR1_L_Msk, l_bits << ADC_SQR1_L_Pos);
}

static void set_dma_number_of_conversions() {
    dma_stream->NDTR = sequence_number - 1;
}

static void set_peripheral_address() {
	dma_stream->PAR = (uint32_t) &handle->DR;
}

static void set_memory_address() {
	dma_stream->M0AR = (uint32_t) values;
}

static void enable_dma_stream() {
	SET_BIT(dma_stream->CR, DMA_SxCR_EN);
}

static void start_conversion() {
	SET_BIT(handle->CR2, ADC_CR2_SWSTART);
}

void start_adc() {
	set_number_of_conversions();
	set_dma_number_of_conversions();
	set_peripheral_address();
	set_memory_address();
	enable_dma_stream();

    start_conversion();
}

int read_adc_value(uint8_t number) {
	return values[number];
}
