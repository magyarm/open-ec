/*
 * usb.h
 *
 *  Created on: Jun. 7, 2020
 *      Author: magyarm
 */

#ifndef SRC_USB_H_
#define SRC_USB_H_

#define CDCACM_GDB_ENDPOINT	    1
#define CDCACM_UART_ENDPOINT	4

#define CDCACM_GDB_DATA_ENDPOINT    0x81
#define CDCACM_GDB_COMM_ENDPOINT    0x82
#define CDCACM_UART_DATA_ENDPOINT   0x84
#define CDCACM_UART_COMM_ENDPOINT   0x85


#define CDCACM_PACKET_SIZE 	64

void exti15_10_isr(void);
char *serialno_read(char *s);

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding);

void usb_init(void);

#endif /* SRC_USB_H_ */
