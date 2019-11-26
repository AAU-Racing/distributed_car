#include <stdio.h>
#include <stm32l4xx.h>

#include "gpio.h"
#include "uart.h"
#include "ringbuffer.h"

#define BUF_SIZE 1024
#define CHANNELS 2
#define CONFIGS 3
#define INSTANCES 2


#define UART_INSTANCE ((USART_TypeDef* const []) { \
    USART2,	\
    USART1, \
    USART1, \
})

#define UART_RX_PIN ((const GPIO_Pin const []) { \
    PIN_3,  \
    PIN_10, \
    PIN_7,  \
})

#define UART_RX_PORT ((GPIO_TypeDef* const []) { \
    GPIOA, \
    GPIOA, \
    GPIOB, \
})

#define UART_TX_PIN ((const GPIO_Pin const []) { \
    PIN_2,  \
    PIN_9,  \
    PIN_6, 	\
})

#define UART_TX_PORT ((GPIO_TypeDef* const []) { \
    GPIOA, \
    GPIOA, \
    GPIOB, \
})

typedef struct {
    ringbuffer_t rb_tx;
    ringbuffer_t rb_rx;
    uint8_t buffer_tx[BUF_SIZE];
    uint8_t buffer_rx[BUF_SIZE];
} uart_channel;

uart_channel channels[CHANNELS];

uint8_t channel_configs[CONFIGS] = { 255, 255, 255 };
uint8_t current_channel = 0;
uint8_t instance_channels[INSTANCES] = {255, 255};

static void init_rb(uint8_t config) {
    uart_channel* channel = channels + channel_configs[config];

    rb_init(&channel->rb_rx, channel->buffer_rx, BUF_SIZE);
    rb_init(&channel->rb_tx, channel->buffer_tx, BUF_SIZE);
}

static void init_gpio(uint8_t config) {
    gpio_af_init(UART_TX_PORT[config], UART_TX_PIN[config], GPIO_HIGH_SPEED, GPIO_PUSHPULL, GPIO_AF7);
    gpio_af_init(UART_RX_PORT[config], UART_RX_PIN[config], GPIO_HIGH_SPEED, GPIO_PUSHPULL, GPIO_AF7);
}


void uart_init(uint8_t config) {
    channel_configs[config] = current_channel++;
    init_rb(config);
    init_gpio(config);

    USART_TypeDef* instance = UART_INSTANCE[config];

    if (instance == USART1){
        instance_channels[0] = channel_configs[config];
        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);

        NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0x0));
        NVIC_EnableIRQ(USART1_IRQn);
    } else if (instance == USART2) {
        instance_channels[1] = channel_configs[config];
        SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART2EN);

        NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0x1));
        NVIC_EnableIRQ(USART2_IRQn);
    }

    SET_BIT(instance->CR1, USART_CR1_TE);
    SET_BIT(instance->CR1, USART_CR1_RE);

    // APB2 and APB2 = 80 MHz
    // 80 MHz / (115200 baud * 16) = 43.402777778
    // 0.402777778 * 16 = 6.444444448, floor(6.444444448) = 0x6
    // 43 << 4 = 0x2b0
    // 0x2b0 + 0x6 = 0x2b6
    instance->BRR = 0x2b6;

    SET_BIT(instance->CR1, USART_CR1_RXNEIE);
    SET_BIT(instance->CR1, USART_CR1_UE);
}

void USARTx_IRQHandler(uint8_t config, USART_TypeDef* instance) {
    uart_channel* channel = channels + instance_channels[config];

    if (READ_BIT(instance->ISR, USART_ISR_RXNE)) {
        uint8_t data_in = (uint8_t) (instance->RDR & 0xFF);
        rb_push(&channel->rb_rx, data_in);
    }

    if (READ_BIT(instance->ISR, USART_ISR_TXE)) {
        uint8_t data_out;
        if (rb_pop(&channel->rb_tx, &data_out) == 1) {
            CLEAR_BIT(instance->CR1, USART_CR1_TXEIE); // no data available
        } else {
            instance->TDR = data_out;
        }
    }
}

void USART1_IRQHandler() {
    USARTx_IRQHandler(0, USART1);
}

void USART2_IRQHandler() {
    USARTx_IRQHandler(1, USART2);
}

uint8_t uart_read_byte(uint8_t config) {
    uint8_t data;
    uart_channel* channel = channels + channel_configs[config];
    while (rb_pop(&channel->rb_rx, &data));
    return data;
}

void uart_send_byte(uint8_t config, uint8_t data) {
    uart_channel* channel = channels + channel_configs[config];
    USART_TypeDef* instance = UART_INSTANCE[config];

    while (rb_isFull(&channel->rb_tx));

    rb_push(&channel->rb_tx, data);
    SET_BIT(instance->CR1, USART_CR1_TXEIE);
}

void uart_send_buf(uint8_t config, uint8_t *data, size_t n) {
    for (size_t i = 0; i < n; i++) {
        uart_send_byte(config, data[i]);
    }
}
