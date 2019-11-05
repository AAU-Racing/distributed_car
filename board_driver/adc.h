#ifndef ADC_H_GUARD
#define ADC_H_GUARD

typedef enum {
    CHANNEL_0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    CHANNEL_9,
    CHANNEL_10,
    CHANNEL_11,
    CHANNEL_12,
    CHANNEL_13,
    CHANNEL_14,
    CHANNEL_15,
    CHANNEL_16,
    CHANNEL_17,
    CHANNEL_18,
    CHANNEL_TEMPSENSOR = CHANNEL_16,
    CHANNEL_VREFINT = CHANNEL_17,
    CHANNEL_VBAT = CHANNEL_18,
} ADC_Channel;

typedef enum {
    SAMPLETIME_3CYCLES,
    SAMPLETIME_15CYCLES,
    SAMPLETIME_28CYCLES,
    SAMPLETIME_56CYCLES,
    SAMPLETIME_84CYCLES,
    SAMPLETIME_112CYCLES,
    SAMPLETIME_144CYCLES,
    SAMPLETIME_480CYCLES,
} ADC_Sampletime;

void init_adc();
void init_adc_channel(ADC_Channel channel, uint8_t *number);
void start_adc();
int read_adc_value(uint8_t number);

#endif
