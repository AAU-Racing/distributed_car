#ifndef USB_H
#define USB_H

#include <stdlib.h>
#include "usbd_cdc_interface.h"

int usb_init(void);
void usb_send(void *buf, size_t len);
uint8_t usb_recieve();

#endif /* USB_H */
