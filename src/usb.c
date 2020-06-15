/*
 * usb.c
 *
 *  Created on: Jun. 7, 2020
 *      Author: magyarm
 */

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <libopencm3/stm32/usart.h>

#include <stdlib.h>

#include "platform.h"

#include "usb.h"
#include "usb_private.h"

extern void serial_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);

#if defined(STM32L0) || defined(STM32F3) || defined(STM32F4)
char serial_no[13];
#else
char serial_no[9];
#endif

usbd_device *usbd_dev_handler;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[256];

volatile uint8_t attached = 0;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xEF,		/* Miscellaneous Device */
	.bDeviceSubClass = 2,		/* Common Class */
	.bDeviceProtocol = 1,		/* Interface Association */
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1D50,
	.idProduct = 0x6018,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

/* Serial ACM interface */
static const struct usb_endpoint_descriptor uart_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_UART_COMM_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor uart_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_UART_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDCACM_UART_DATA_ENDPOINT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) uart_cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = UART_DATA_IFACE,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 2, /* SET_LINE_CODING supported*/
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = UART_COMM_IFACE,
		.bSubordinateInterface0 = 3,
	 }
};

static const struct usb_interface_descriptor uart_comm_iface[] = {
	{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_COMM_IFACE,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 5,

	.endpoint = uart_comm_endp,

	.extra = &uart_cdcacm_functional_descriptors,
	.extralen = sizeof(uart_cdcacm_functional_descriptors)
	}
};

static const struct usb_interface_descriptor uart_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_DATA_IFACE,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = uart_data_endp,
}};

static const struct usb_iface_assoc_descriptor uart_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 2,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,
	.iFunction = 0,
};

static const struct usb_interface ifaces[] = {
	{
			.num_altsetting = 1,
			.iface_assoc = &uart_assoc,
			.altsetting = uart_comm_iface,
	}, {
			.num_altsetting = 1,
			.altsetting = uart_data_iface,
	}
};

static const char *usb_strings[] = {
	"Mike-EC",
	"USB Debugger"
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

/* USB interrupt handler */
void usb_wakeup_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_hp_can_tx_isr(void)
	__attribute__ ((alias ("usb_int_relay")));

void usb_lp_can_rx0_isr(void)
	__attribute__ ((alias ("usb_int_relay")));


static void usb_int_relay(void)
{
	usbd_poll(usbd_dev_handler);
}

static void cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr, bool dcd)
{
	char buf[10];
	struct usb_cdc_notification *notif = (void*)buf;
	/* We echo signals back to host as notification */
	notif->bmRequestType = 0xA1;
	notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notif->wValue = 0;
	notif->wIndex = iface;
	notif->wLength = 2;
	buf[8] = (dsr ? 2 : 0) | (dcd ? 1 : 0);
	buf[9] = 0;
	usbd_ep_write_packet(dev, 0x82 + iface, buf, 10);
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
	usart_set_baudrate(USART1, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USART1, coding->bDataBits + 1);
	else
		usart_set_databits(USART1, coding->bDataBits);

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USART1, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USART1, USART_STOPBITS_1_5);
		break;
	case 2:
		usart_set_stopbits(USART1, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USART1, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USART1, USART_PARITY_ODD);
		break;
	case 2:
		usart_set_parity(USART1, USART_PARITY_EVEN);
		break;
	}
}

static int ec_control_request(usbd_device *dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)dev;
	(void)len;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		cdcacm_set_modem_state(dev, req->wIndex, true, true);
		/* Ignore if not for GDB interface */
		if(req->wIndex != 0)
			return USBD_REQ_HANDLED;

		return USBD_REQ_HANDLED;
	case USB_CDC_REQ_SET_LINE_CODING:
		if(*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;

		switch(req->wIndex) {
		case 2:
			usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf);
			return USBD_REQ_HANDLED;
			break;
		case 0:
			return USBD_REQ_HANDLED; /* Ignore on GDB Port */
			break;
		default:
			return USBD_REQ_NOTSUPP;
		}
		break;
	}
	return USBD_REQ_NOTSUPP;
}




void exti15_10_isr(void)
{
	if (gpio_get(USB_VBUS_PORT, USB_VBUS_PIN)) {
		/* Drive pull-up high if VBUS connected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	} else {
		/* Allow pull-up to float if VBUS disconnected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
	}

	exti_reset_request(USB_VBUS_PIN);
}

static void usb_config_callback(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

//	usbd_ep_setup(usbd_dev, CDCACM_GDB_DATA_ENDPOINT, USB_ENDPOINT_ATTR_BULK, 64,
//			NULL);
	//TODO: Setup DAP Endpoint here
	//	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, jtag_data_rx_cb); //JTAG

	usbd_ep_setup(usbd_dev, CDCACM_UART_ENDPOINT, USB_ENDPOINT_ATTR_BULK,
			CDCACM_PACKET_SIZE / 2, NULL);
	usbd_ep_setup(usbd_dev, CDCACM_UART_DATA_ENDPOINT, USB_ENDPOINT_ATTR_BULK,
			CDCACM_PACKET_SIZE, serial_data_rx_cb); //Serial
	usbd_ep_setup(usbd_dev, CDCACM_UART_COMM_ENDPOINT, USB_ENDPOINT_ATTR_INTERRUPT,
			16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				ec_control_request);

	attached = 1;
}



char *serialno_read(char *s)
{
#if defined(STM32L0) || defined(STM32F3) || defined(STM32F4)
	volatile uint16_t *uid = (volatile uint16_t *)DESIG_UNIQUE_ID_BASE;
# if defined(STM32F4)
	int offset = 3;
# elif defined(STM32L0) || defined(STM32F4)
	int offset = 5;
#endif
	sprintf(s, "%04X%04X%04X",
            uid[1] + uid[5], uid[0] + uid[4], uid[offset]);
#else
	volatile uint32_t *unique_id_p = (volatile uint32_t *)0x1FFFF7E8;
	uint32_t unique_id = *unique_id_p +
			*(unique_id_p + 1) +
			*(unique_id_p + 2);
	int i;

	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < 8; i++) {
		s[7-i] = ((unique_id >> (4*i)) & 0xF) + '0';
	}
	for(i = 0; i < 8; i++)
		if(s[i] > '9')
			s[i] += 'A' - '9' - 1;
	s[8] = 0;

#endif
	return s;
}

void usb_init(void)
{
	exti15_10_isr();

	serialno_read(serial_no);

	usbd_dev_handler = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
				usb_strings, sizeof(usb_strings)/sizeof(char *),
				usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev_handler, usb_config_callback);
}
