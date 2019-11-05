#include "usbd_cdc_interface.h"
#include "../ringbuffer.h"


#define RX_DATA_SIZE  1024


ringbuffer_t usb_rec;
uint8_t usb_rec_buf[RX_DATA_SIZE];

uint8_t UserRxBuffer[RX_DATA_SIZE]; /* Received Data over USB are stored in this buffer */

/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops = {
	CDC_Itf_Init,
	CDC_Itf_DeInit,
	CDC_Itf_Control,
	CDC_Itf_Receive
};


static int8_t CDC_Itf_Init(void) {
	USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
	rb_init(&usb_rec, usb_rec_buf, RX_DATA_SIZE);

	return (USBD_OK);
}


static int8_t CDC_Itf_DeInit(void) {
  return (USBD_OK);
}


/*
 * @brief  CDC_Itf_Control
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length) {
	return (USBD_OK);
}


static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len) {
	for (size_t i = 0; i < *Len; i++) {
		rb_push(&usb_rec, Buf[i]);
	}
	USBD_CDC_ReceivePacket(&USBD_Device);
	return (USBD_OK);
}


uint8_t cdc_recieve() {
	uint8_t data;
	while (rb_pop(&usb_rec, &data));
	return data;
}

void cdc_send(void *buf, size_t len) {
	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)buf, len);
	USBD_CDC_TransmitPacket(&USBD_Device);
}
