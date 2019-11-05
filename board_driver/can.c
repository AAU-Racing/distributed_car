#include <stm32f4xx.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "can.h"
#include "gpio.h"

#define MAX_FRAME_LEN       8
#define MAX_STD_ID          0x7FF
#define NUMBER_BANKS        28
#define BUFFER_SIZE         256
#define NO_EMPTY_MAILBOX    255

#define CAN_INSTANCE ((CAN_TypeDef* const []) { \
    CAN1, \
    CAN2, \
    CAN1, \
    CAN2, \
    CAN1, \
})

#define CAN_RX_PIN ((const GPIO_Pin const []) { \
    PIN_11, \
    PIN_5,	 \
    PIN_8,	 \
    PIN_12, \
    PIN_0,  \
})

#define CAN_RX_PORT ((GPIO_TypeDef* const []) { \
    GPIOA, \
    GPIOB, \
    GPIOB, \
    GPIOB, \
    GPIOD, \
})

#define CAN_TX_PIN ((const GPIO_Pin const []) { \
    PIN_12, \
    PIN_6,	 \
    PIN_9,	 \
    PIN_13, \
    PIN_1,  \
})

#define CAN_TX_PORT ((GPIO_TypeDef* const []) { \
    GPIOA, \
    GPIOB, \
    GPIOB, \
    GPIOB, \
    GPIOD, \
})

#define MIN(x, y) (((x) < (y)) ? (x) : (y))

typedef struct {
    uint16_t StdId;
    uint8_t Length;
    uint8_t Msg[8];
} CAN_TransmitFrame;

// General variables for CAN
static volatile CAN_TypeDef *handle;
static bool initialized = false;
static bool started = false;
static int filter_num = 0;
static volatile CAN_Statistics stats;

// Callback function stack
static volatile CAN_RX_Callback rx_callback[14];
static volatile uint8_t top = 0;

// Transmit queue
static volatile uint16_t head = 0;
static volatile uint16_t tail = 0;
static volatile uint16_t size = 0;
static volatile CAN_TransmitFrame transmit_buffer[BUFFER_SIZE];

// Local function prototypes
// can_filter
static bool is_valid_filter(uint16_t id, uint16_t mask);
static void set_callback_for_filter(CAN_RX_Callback callback);
static void init_filter(uint16_t id, uint16_t mask);
static void enter_filter_init_mode();
static void setup_filter(uint16_t id, uint16_t mask);
static void activate_filter();
static void leave_filter_init_mode();

// can_init
static void prepare_config(uint8_t config);
static void set_can_instance(uint8_t config);
static void start_clock();
static void exit_sleep();
static uint8_t enter_init();
static void setup_can();
static void setup_timing();
static void configure_gpio_pin(GPIO_TypeDef *port, GPIO_Pin pin);
static void reset_filters();
static void enable_interrupt_vectors();
static void enable_error_interrupts();
static void enable_tx_interrupt();
static uint8_t leave_init();

// can_start
static void enable_rx_interrupt();

// can_transmit
static bool is_valid_id(uint16_t id);
static bool is_valid_length(uint8_t length);
static uint8_t select_empty_mailbox();
static bool is_mailbox_empty(uint32_t mailbox);
static bool is_buffer_full();
static void enqueue_message(uint16_t id, volatile uint8_t msg[], uint8_t length);
static void put_message_in_mailbox(uint8_t transmitmailbox, uint16_t id, volatile uint8_t msg[], uint8_t length);
static uint32_t get_message_mask_low(uint8_t length);
static uint32_t get_message_mask_high(uint8_t length);
static void send_message(uint8_t transmitmailbox);

// CAN_TxCallback
static void clear_request_complete_flags();

// CAN_RxCallback
static void get_latest_msg(CAN_RxFrame *frame);
static void release_receive_mailbox();

// CAN_ErrorCallback
static void handle_general_errors();
static void handle_message_errors();
static void clear_error_interrupt_flag();

// Functions
static void naive_memcpy(volatile uint8_t *dst, volatile uint8_t *src, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        dst[i] = src[i];
    }
}

uint8_t can_filter(uint16_t id, uint16_t mask, CAN_RX_Callback callback) {
    // Validate input
    if (!is_valid_filter(id, mask)) {
        return CAN_INVALID_ID;
    }

    if (!initialized) {
        return CAN_DRIVER_ERROR;
    }

    set_callback_for_filter(callback);
    init_filter(id, mask);

    filter_num++;

    return CAN_OK;
}

static bool is_valid_filter(uint16_t id, uint16_t mask) {
    return id <= MAX_STD_ID && mask <= MAX_STD_ID;
}

static void set_callback_for_filter(CAN_RX_Callback callback) {
    rx_callback[top++] = callback;
}

static void init_filter(uint16_t id, uint16_t mask) {
    enter_filter_init_mode();
    setup_filter(id, mask);
    activate_filter();
    leave_filter_init_mode();
}

static void enter_filter_init_mode() {
    SET_BIT(handle->FMR, CAN_FMR_FINIT);
}

static void setup_filter(uint16_t id, uint16_t mask) {
    handle->sFilterRegister[filter_num].FR1 = id << 21; // FR1[31:21]
    handle->sFilterRegister[filter_num].FR2 = mask << 21; // FR2[31:21]
}

static void activate_filter() {
    SET_BIT(handle->FA1R, 1 << filter_num);
}

static void leave_filter_init_mode() {
    CLEAR_BIT(handle->FMR, CAN_FMR_FINIT);
}

CAN_Statistics can_get_stats() {
    return stats;
}

uint8_t can_init(uint8_t config) {
    prepare_config(config);

    if (enter_init() == CAN_INIT_TIMEOUT) {
        return CAN_INIT_TIMEOUT;
    }

    setup_can();
    setup_timing();

    configure_gpio_pin(CAN_RX_PORT[config], CAN_RX_PIN[config]);
    configure_gpio_pin(CAN_TX_PORT[config], CAN_TX_PIN[config]);

    enter_filter_init_mode();
    reset_filters();
    leave_filter_init_mode();

    enable_interrupt_vectors();
    enable_error_interrupts();
    enable_tx_interrupt();

    if (leave_init() == CAN_INIT_TIMEOUT) {
        return CAN_INIT_TIMEOUT;
    }

    initialized = true;

    return CAN_OK;
}

static void prepare_config(uint8_t config) {
    set_can_instance(config);
    start_clock();
    exit_sleep();
}

static void set_can_instance(uint8_t config) {
    handle = CAN_INSTANCE[config];
}

static void start_clock() {
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN);
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN2EN);
}

static void exit_sleep() {
    CLEAR_BIT(handle->MCR, CAN_MCR_SLEEP);
}

static uint8_t enter_init() {
    SET_BIT(handle->MCR, CAN_MCR_INRQ); // Request entering initialisation

    uint32_t tickstart = HAL_GetTick();

    while((handle->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) // Wait for initialisation
    {
        if((HAL_GetTick() - tickstart ) > 10U) // Wait up to 10 ms then timeout
        {
            return CAN_INIT_TIMEOUT;
        }
    }

    return CAN_OK;
}

static void setup_can() {
    CLEAR_BIT(handle->MCR, CAN_MCR_TTCM); // Disable time triggered communication (which is just counter of when reception has occured)
    SET_BIT(handle->MCR, CAN_MCR_ABOM); // Enable automatic bus off management aka. try to rejoin the bus when bus becomes off
    SET_BIT(handle->MCR, CAN_MCR_AWUM); // Enable automatic wakeup
    CLEAR_BIT(handle->MCR, CAN_MCR_NART); // Enable automatic retransmission aka. retransmit if an error occours during transmission
    CLEAR_BIT(handle->MCR, CAN_MCR_RFLM); // Diable receive FIFO locked mode (disable means overwriting when receiving a new message)
    CLEAR_BIT(handle->MCR, CAN_MCR_TXFP); // Enable transmit priority (Choose the most important of the three transmit mailboxes based on ID)
    CLEAR_BIT(handle->BTR, CAN_BTR_SILM); // Normal mode
    CLEAR_BIT(handle->BTR, CAN_BTR_LBKM); // Disable loopback
}

static void setup_timing() {
    // 500 kbps with 87.5% sample point

    // AHBCLK = SYSCLOCK / 1 = 160 / 1 MHz = 160 MHz
    // APB1CLK = AHBCLK / 4 = 160 / 4 MHz = 40 MHz

    // Length of 1 tq = prescaler * (1 / APB1CLK)
    // NBT = 1 / bitrate
    // TQ per bit = NBT / 1 tq

    MODIFY_REG(handle->BTR, CAN_BTR_SJW_Msk, 0 << CAN_BTR_SJW_Pos); // Set synchronization jump width to 1 TQ (BTR[25:24] + 1)
    MODIFY_REG(handle->BTR, CAN_BTR_TS1_Msk, 12 << CAN_BTR_TS1_Pos); // Set time segment 1 to 13 TQ (BTR[19:16] + 1)
    MODIFY_REG(handle->BTR, CAN_BTR_TS2_Msk, 1 << CAN_BTR_TS2_Pos); // Set time segment 2 to 2 TQ (BTR[22:20] + 1)
    MODIFY_REG(handle->BTR, CAN_BTR_BRP_Msk, 4 << CAN_BTR_BRP_Pos); // Set baud rate prescaler to 5 (BTR[9:0] + 1)
}

static void configure_gpio_pin(GPIO_TypeDef *port, GPIO_Pin pin) {
    gpio_af_init(port, pin, GPIO_HIGH_SPEED, GPIO_PUSHPULL, GPIO_AF9);
}

static void reset_filters() {
    handle->FFA1R = 0; // Set all filters to FIFO 0
    handle->FS1R = 0x0FFFFFFF; // Set all filters to 32-bit single filter
    handle->FM1R = 0; // Set all filters to mask mode

    int filter_bank = 14; //handle == CAN1 ? 0 : 28;
    filter_num = handle == CAN1 ? 0 : 14;

    MODIFY_REG(handle->FMR, CAN_FMR_CAN2SB_Msk, filter_bank << CAN_FMR_CAN2SB_Pos);
}

static void enable_interrupt_vectors() {
    // RX fifo 0 interrupt (the only fifo we use)
    NVIC_SetPriority(handle == CAN1 ? CAN1_RX0_IRQn : CAN2_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0x0));
    NVIC_EnableIRQ(handle == CAN1 ? CAN1_RX0_IRQn : CAN2_RX0_IRQn);

    // TX interrupt
    NVIC_SetPriority(handle == CAN1 ? CAN1_TX_IRQn : CAN2_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0x1));
    NVIC_EnableIRQ(handle == CAN1 ? CAN1_TX_IRQn : CAN2_TX_IRQn);

    // Status change/error interrupt
    NVIC_SetPriority(handle == CAN1 ? CAN1_SCE_IRQn : CAN2_SCE_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x15, 0x0));
    NVIC_EnableIRQ(handle == CAN1 ? CAN1_SCE_IRQn : CAN2_SCE_IRQn);
}

static void enable_error_interrupts() {
    // SET_BIT(handle->IER, CAN_IER_ERRIE); // Allow error interrupts
    // SET_BIT(handle->IER, CAN_IER_LECIE); // Enable last error code interrupt (error code for last message if an error occoured)
    // SET_BIT(handle->IER, CAN_IER_BOFIE); // Enable bus-off error interrupt
    // SET_BIT(handle->IER, CAN_IER_EWGIE); // Enable error warning interrupt
    // SET_BIT(handle->IER, CAN_IER_EPVIE); // Enable error passive interrupt
}

static void enable_tx_interrupt() {
    SET_BIT(handle->IER, CAN_IER_TMEIE);
}

static uint8_t leave_init() {
    CLEAR_BIT(handle->MCR, CAN_MCR_INRQ); // Request exiting initialisation

    uint32_t tickstart = HAL_GetTick();

    while((handle->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) // Wait for leaving initialisation
    {
        if((HAL_GetTick() - tickstart ) > 10U) // Wait up to 10 ms then timeout
        {
            return CAN_INIT_TIMEOUT;
        }
    }

    return CAN_OK;
}

uint8_t can_start() {
    if (!initialized) {
        return CAN_DRIVER_ERROR;
    }

    enable_rx_interrupt();

    started = true;

    return CAN_OK;
}

static void enable_rx_interrupt() {
    SET_BIT(handle->IER, CAN_IER_FMPIE0);
}

uint8_t can_transmit(uint16_t id, uint8_t msg[], uint8_t length) {
    if (!is_valid_id(id)) {
        return CAN_INVALID_ID;
    }

    if (!is_valid_length(length)) {
        return CAN_INVALID_FRAME;
    }

    if (!started) {
        return CAN_DRIVER_ERROR;
    }

    uint8_t transmitmailbox = select_empty_mailbox();

    if (transmitmailbox == NO_EMPTY_MAILBOX) {
        if (is_buffer_full()) {
            return CAN_BUFFER_FULL;
        }
        else {
            enqueue_message(id, msg, length);
            return CAN_OK; // Queued for transmission
        }
    }

    put_message_in_mailbox(transmitmailbox, id, msg, length);
    send_message(transmitmailbox);

    return CAN_OK;
}

static bool is_valid_id(uint16_t id) {
    return id <= MAX_STD_ID;
}

static bool is_valid_length(uint8_t length) {
    return length <= MAX_FRAME_LEN;
}

static uint8_t select_empty_mailbox() {
    if (is_mailbox_empty(CAN_TSR_TME0)) { // TME0 = Transmit mailbox 0 empty
        return 0;
    }
    else if (is_mailbox_empty(CAN_TSR_TME1)) {
        return 1;
    }
    else if (is_mailbox_empty(CAN_TSR_TME2)) {
        return 2;
    }

    return NO_EMPTY_MAILBOX; // Default mailbox
}

static bool is_mailbox_empty(uint32_t mailbox) {
    return (handle->TSR & mailbox) == mailbox;
}

static bool is_buffer_full() {
    return size == BUFFER_SIZE;
}

static void enqueue_message(uint16_t id, volatile uint8_t msg[], uint8_t length) {
    transmit_buffer[head].StdId = id;
    transmit_buffer[head].Length = length;
    naive_memcpy(transmit_buffer[tail].Msg, msg, length);

    // Increment queue head and size
    tail = (tail + 1) % BUFFER_SIZE;
    size++;
}

static void put_message_in_mailbox(uint8_t transmitmailbox, uint16_t id, volatile uint8_t msg[], uint8_t length) {
    handle->sTxMailBox[transmitmailbox].TIR = 0;
    MODIFY_REG(handle->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_STID_Msk, id << CAN_TI0R_STID_Pos); // EXID = 0, IDE = 0, RTR = 0, TXRQ = 0, STID = id
    MODIFY_REG(handle->sTxMailBox[transmitmailbox].TDTR, CAN_TDT0R_DLC_Msk, length); // Set DLC

    handle->sTxMailBox[transmitmailbox].TDLR = (*((uint32_t*) &msg[0])) & get_message_mask_low(length);
    handle->sTxMailBox[transmitmailbox].TDHR = (*((uint32_t*) &msg[4])) & get_message_mask_high(length);
}

static uint32_t get_message_mask_low(uint8_t length) {
    return (length > 3 ? 0xFF << 24 : 0) | (length > 2 ? 0xFF << 16 : 0) | (length > 1 ? 0xFF << 8 : 0) | (length > 0 ? 0xFF : 0);
}

static uint32_t get_message_mask_high(uint8_t length) {
    return (length > 7 ? 0xFF << 24 : 0) | (length > 6 ? 0xFF << 16 : 0) | (length > 5 ? 0xFF << 8 : 0) | (length > 4 ? 0xFF : 0);
}

static void send_message(uint8_t transmitmailbox) {
    SET_BIT(handle->sTxMailBox[transmitmailbox].TIR, CAN_TI0R_TXRQ);
}

////////////////////////
// Callback functions //
////////////////////////
void CAN_TxCallback() {
    stats.transmit++;

    clear_request_complete_flags();

    if (size > 0) {
        uint8_t transmitmailbox = select_empty_mailbox();

        if (transmitmailbox != NO_EMPTY_MAILBOX) {
            put_message_in_mailbox(transmitmailbox, transmit_buffer[head].StdId, transmit_buffer[head].Msg, transmit_buffer[head].Length);
            send_message(transmitmailbox);

            // Increment tail for next item and decrement size to indicate removal.
            tail = (tail + 1) % BUFFER_SIZE;
            size--;
        }
    }
}

static void clear_request_complete_flags() {
    SET_BIT(handle->TSR, CAN_TSR_RQCP0);
    SET_BIT(handle->TSR, CAN_TSR_RQCP1);
    SET_BIT(handle->TSR, CAN_TSR_RQCP2);
}

void CAN_RxCallback() {
    stats.receive++;
    stats.last_receive = HAL_GetTick();
    CAN_RxFrame frame;

    get_latest_msg(&frame);
    release_receive_mailbox();

    // Callback for filter
    rx_callback[frame.FMI](&frame);
}

static void get_latest_msg(CAN_RxFrame *frame) {
    frame->FMI = (handle->sFIFOMailBox[0].RDTR & CAN_RDT0R_FMI_Msk) >> CAN_RDT0R_FMI_Pos;
    frame->StdId = (handle->sFIFOMailBox[0].RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos;
    frame->Length = handle->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk;

    (*((uint32_t*) &frame->Msg[0])) = handle->sFIFOMailBox[0].RDLR & get_message_mask_low(frame->Length);
    (*((uint32_t*) &frame->Msg[4])) = handle->sFIFOMailBox[0].RDHR & get_message_mask_high(frame->Length);
}

static void release_receive_mailbox() {
    SET_BIT(handle->RF0R, CAN_RF0R_RFOM0);
}

void CAN_ErrorCallback() {
    stats.error_total++;

    handle_general_errors();
    handle_message_errors();

    clear_error_interrupt_flag();
}

static void handle_general_errors() {
    if (READ_BIT(handle->ESR, CAN_ESR_BOFF) == CAN_ESR_BOFF) { // If callback was due to bus off
        stats.error_bof++;
    }

    if (READ_BIT(handle->ESR, CAN_ESR_EWGF) == CAN_ESR_EWGF) { // If callback was due to error warning flag
        stats.error_ewg++;
    }

    if (READ_BIT(handle->ESR, CAN_ESR_EPVF) == CAN_ESR_EPVF) { // If callback was due to error passive flag
        stats.error_epv++;
    }
}

static void handle_message_errors() {
    if ((handle->ESR & CAN_ESR_LEC) != 0) { // If message was due to message error flag
        switch ((handle->ESR & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos) {
            case 1:
            stats.error_stuff++; // Incorrect bit stuffing which causes wrong synchronization
            break;
            case 2:
            stats.error_form++; // Wrong state of static bits (aka. frame form error). E.g. CRC delimiter, ACK delimiter, End of Frame
            break;
            case 3:
            stats.error_ack++; // No acknowledge on transmit
            break;
            case 4:
            stats.error_recess++; // During any transmission, the controller wanted to transmit a recessive bit (1/true) but observed a dominant bit (0/false)
            break;
            case 5:
            stats.error_dominant++; // During any transmission, the controller wanted to transmit a dominant bit (0/false) but observed a recessive bit (1/true)
            break;
            case 6:
            stats.error_crc++; // Error in CRC value of received message
            break;
        }
    }
}

static void clear_error_interrupt_flag() {
    CLEAR_BIT(handle->MSR, CAN_MSR_ERRI);
}

// Interrupts
void CAN1_RX0_IRQHandler() {
    CAN_RxCallback();
}
void CAN2_RX0_IRQHandler() {
    CAN_RxCallback();
}
void CAN1_TX_IRQHandler() {
    CAN_TxCallback();
}
void CAN2_TX_IRQHandler() {
    CAN_TxCallback();
}
void CAN1_SCE_IRQHandler() {
    CAN_ErrorCallback();
}
void CAN2_SCE_IRQHandler() {
    CAN_ErrorCallback();
}
