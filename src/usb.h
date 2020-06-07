/*
 * usb.h
 *
 *  Created on: Jun. 7, 2020
 *      Author: magyarm
 */

#ifndef SRC_USB_H_
#define SRC_USB_H_

#define JTAG_ENDPOINT_ADDRESS 0x81
#define SERIAL_ENDPOINT_ADDRESS 0x83

void exti15_10_isr(void);
char *serialno_read(char *s);

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding);

void usb_init(void);

#endif /* SRC_USB_H_ */
