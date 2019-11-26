#include <stm32l4xx_hal.h>

#include "adc.h"

#define ADC_DMA_PERIPH_TO_MEMORY 	0
#define ADC_DMA_PERIPH_INC		 	  DMA_CCR_PINC
#define ADC_DMA_MEM_INC				    DMA_CCR_MINC
#define ADC_DMA_PDATAALIGN_WORD		DMA_CCR_PSIZE_1
#define ADC_DMA_MDATAALIGN_WORD   DMA_CCR_MSIZE_1
#define ADC_DMA_CIRCULAR			    DMA_CCR_CIRC
#define ADC_DMA_PRIORITY_HIGH		  DMA_CCR_PL_1

#define TEMPSENSOR_DELAY_US   10
#define STABILZATION_DELAY_US 10

static int sequence_number = 1;
__IO uint32_t values[16];
ADC_TypeDef* handle;
DMA_Channel_TypeDef* dma_stream;

static void clk_init() {
	SET_BIT(RCC->AHB3ENR, RCC_AHB2ENR_ADCEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA2EN);
}

static void enable_adc() {
	SET_BIT(handle->CR, ADC_CR_ADEN);
}

static void wait_for_stabilization() {
	// Delay for ADC stabilization time
	// Compute number of CPU cycles to wait for
	uint32_t counter = STABILZATION_DELAY_US * 80; // 80 MHz core clock
	while(counter != 0U) {
		counter--;
	}
}

static void enable_adc_dma_mode() {
	SET_BIT(handle->CFGR, ADC_CFGR_DMAEN);
}

static void set_prescaler() {
	// Div 8 relative to sysclock
	MODIFY_REG(ADC1_COMMON->CCR, ADC_CCR_PRESC_Msk, 0x4 << ADC_CCR_PRESC_Pos);
}

static void enable_scan_conv() {
	// SET_BIT(handle->CR, ADC_CR_SCAN);
}

static void set_resolution() {
	// 12 bit resolution
	MODIFY_REG(handle->CFGR, ADC_CFGR_RES_Msk, 0);
}

static void set_data_align() {
	// Align right
	CLEAR_BIT(handle->CFGR, ADC_CFGR_ALIGN);
}

static void enable_continuous_mode() {
	SET_BIT(handle->CFGR, ADC_CFGR_CONT);
}

static void enable_dma_continuous_mode() {
	SET_BIT(handle->CFGR, ADC_CFGR_DMACFG);
}

static void dma_init() {
	SET_BIT(dma_stream->CCR, ADC_DMA_MEM_INC);
	CLEAR_BIT(dma_stream->CCR, ADC_DMA_PERIPH_INC);
	MODIFY_REG(dma_stream->CCR, DMA_CCR_DIR, ADC_DMA_PERIPH_TO_MEMORY);
	MODIFY_REG(dma_stream->CCR, DMA_CCR_PSIZE_Msk, ADC_DMA_PDATAALIGN_WORD);
	MODIFY_REG(dma_stream->CCR, DMA_CCR_MSIZE_Msk, ADC_DMA_MDATAALIGN_WORD);
	SET_BIT(dma_stream->CCR, ADC_DMA_CIRCULAR);
	MODIFY_REG(dma_stream->CCR, DMA_CCR_PL, ADC_DMA_PRIORITY_HIGH);
	MODIFY_REG(DMA1_CSELR->CSELR, 0xF, 0);
}

void init_adc() {
	handle = ADC1;
	dma_stream = DMA1_Channel1;

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

	uint32_t sampletime = 0x5 << pos;

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
	SET_BIT(ADC1_COMMON->CCR, ADC_CCR_VBATEN);
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
	SET_BIT(ADC1_COMMON->CCR, ADC_CCR_TSEN);

	if (channel == CHANNEL_TEMPSENSOR) {
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

	if (channel == CHANNEL_TEMPSENSOR) {
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
    dma_stream->CNDTR = sequence_number - 1;
}

static void set_peripheral_address() {
	dma_stream->CPAR = (uint32_t) &handle->DR;
}

static void set_memory_address() {
	dma_stream->CMAR = (uint32_t) values;
}

static void enable_dma_stream() {
	SET_BIT(dma_stream->CCR, DMA_CCR_EN);
}

static void start_conversion() {
	SET_BIT(handle->CR, ADC_CR_ADSTART);
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
