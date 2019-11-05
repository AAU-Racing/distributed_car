#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#include <usbd_cdc.h>


extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;
uint8_t cdc_recieve();
void cdc_send(void *buf, size_t len);

#endif /* USBD_CDC_IF_H */
