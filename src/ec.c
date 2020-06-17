/*
 * This file is part of the open-ec (Embedded controller) project.
 *
 * Copyright (C) 2019 Zhiyuan Wan <h@iloli.bid>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <stdlib.h>

#include "usb.h"

#define FIFO_SIZE 256

#define SERIAL_IN_SINGLEBUF 1
#define UART_SEND_BLOCKING

extern usbd_device *usbd_dev_handler;
extern volatile uint8_t attached;

volatile uint8_t latency_timer[2] = {3, 3};

/* RX Fifo buffer */
static uint8_t buf_rx[FIFO_SIZE];
/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t buf_rx_in;
/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t buf_rx_out;

uint8_t dtr = 1, rts = 1;

volatile uint16_t timer_count = 0;
volatile uint16_t last_send = 0;

//TODO: Move platform specific code out from main.c
/* Here it starts */
static void rcc_clock_setup_in_hse_24mhz_out_72mhz(void)
{
    /* Set system clock to 72 MHz, base frequency = 24MHz */
	/* Enable internal high-speed oscillator. */
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);

	/* Select HSI as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

	/* Enable external high-speed oscillator 24MHz. */
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 72MHz Max. 72MHz */
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV8);  /* Set.  9MHz Max. 14MHz */
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);     /* Set. 36MHz Max. 36MHz */
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 72MHz Max. 72MHz */

	/*
	 * Sysclk runs with 72MHz -> 2 waitstates. (STM32 only)
	 * 0WS from 0-24MHz
	 * 1WS from 24-48MHz
	 * 2WS from 48-72MHz
	 */
	flash_set_ws(FLASH_ACR_LATENCY_2WS);

	/*
	 * Set the PLL multiplication factor to 6.
	 * 12MHz (external) * 6 (multiplier) = 72MHz
	 */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL6);

	/* Select HSE as PLL source. */
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

	/* Some board will have issues in high external clock input > 16MHz
	 * Set prescaler to 2 to avoid this issue and make system more stable
	 */
	rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK_DIV2); 

	/* Enable PLL oscillator and wait for it to stabilize. */
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	/* Select PLL as SYSCLK source. */
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

	/* Set the peripheral clock frequencies used */
	rcc_ahb_frequency = 72000000;
	rcc_apb1_frequency = 36000000;
	rcc_apb2_frequency = 72000000;
	
}

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_24mhz_out_72mhz();
	//rcc_clock_setup_in_hsi_out_48mhz();

	/* Enable clocks. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOA);
	//rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART1);
	
	SCB_VTOR = 0x08002000; 
}

static void gpio_setup(void)
{		

	/* LED 引脚 */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO2 | GPIO10 | GPIO11);
	/* USB 上拉 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
		      
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO15); //1.8V IO BOOT脚（DTR）
			
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO2);  //1.8V IO RST脚（RTS）

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO5 | GPIO7); //TCK TDI
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO14); //TMS
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO6); //TDO
			
	gpio_set(GPIOA, GPIO2 | GPIO5 | GPIO6 | GPIO7);
	gpio_set(GPIOB, GPIO10 | GPIO11 | GPIO14 | GPIO15 );
			
	/* 防止电源短路 */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO0 | GPIO1);
	//暂且用Bitbang模拟SESPM
}

uint8_t serial_recv_buf[64];
uint8_t serial_recv_len;
uint8_t serial_recv_i;

void serial_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	serial_recv_len = usbd_ep_read_packet(usbd_dev, CDCACM_UART_ENDPOINT, serial_recv_buf, 64);
	if (serial_recv_len)
	{
		int i;
		for(i = 0; i < serial_recv_len; i++)
			usart_send_blocking(USART1, serial_recv_buf[i]);
		gpio_set(GPIOB, GPIO2);
	}
}

static void interrupt_setup(void)
{
	asm("cpsid i");
	/* 开一个 1ms 的定时器 */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); /* 72MHz / 8 = 9MHz */
	/* 定时器每N次中断一次 */
	systick_set_reload(8999); /* 9000 / 9000 = 1kHz */
	systick_interrupt_enable();
	systick_counter_enable();

	nvic_set_priority(NVIC_USART1_IRQ, 0);//串口最高优先级
	nvic_set_priority(NVIC_USB_WAKEUP_IRQ, 1 << 4);
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 1 << 4);
	nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 1 << 4);
	nvic_set_priority(NVIC_SYSTICK_IRQ, 2 << 4);//systick最低优先级
	
	nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);


	/* 开接收中断 */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;
	USART_CR1(USART1) &= ~USART_CR1_TXEIE;

	__asm__("cpsie i"); 
}

void sys_tick_handler(void)
{
	timer_count ++;
	if(timer_count % 16 == 0)
		gpio_clear(GPIOB, GPIO2);
}


static void usb_packet_handler(void)
{
	/* if fifo empty, nothing further to do */
	if (buf_rx_in == buf_rx_out) {
		/* turn off LED, disable IRQ */
		gpio_clear(GPIOB, GPIO2);
	}
	else
	{
		uint8_t packet_buf[CDCACM_PACKET_SIZE];
		uint8_t packet_size = 0;
		uint32_t buf_out = buf_rx_out;

		/* copy from uart FIFO into local usb packet buffer */
		while (buf_rx_in != buf_out && packet_size < CDCACM_PACKET_SIZE)
		{
			packet_buf[packet_size++] = buf_rx[buf_out++];
			/* wrap out pointer */
			if (buf_out >= FIFO_SIZE)
			{
				buf_out = 0;
			}
		}
		/* advance fifo out pointer by amount written */
		buf_rx_out += usbd_ep_write_packet(usbd_dev_handler,
				CDCACM_UART_DATA_ENDPOINT, packet_buf, packet_size);
		buf_rx_out %= FIFO_SIZE;
	}
}
/* 串口开始 */
static void uart_setup(void)
{
	//启动IO口
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX); //TODO:奇偶校验，stop bit

	usart_enable(USART1);	
}

void usart1_isr(void) //串口中断
{
	if (((USART_SR(USART1) & USART_SR_RXNE) != 0) || 
		((USART_SR(USART1) & USART_SR_ORE) != 0) ||
		((USART_SR(USART1) & USART_SR_NE) != 0) ||
		((USART_SR(USART1) & USART_SR_FE) != 0)) {
//		circular_buf_put( serial_uart_in_cbuf, USART_DR(USART1) & USART_DR_MASK); //接收
		if (((buf_rx_in + 1) % FIFO_SIZE) != buf_rx_out)
		{
			/* insert into FIFO */
			buf_rx[buf_rx_in++] = (USART_DR(USART1) & USART_DR_MASK);

			/* wrap out pointer */
			if (buf_rx_in >= FIFO_SIZE)
			{
				buf_rx_in = 0;
			}
		}
	}
	gpio_set(GPIOB, GPIO2);
}

int main(void)
{
	volatile int i;
	
	clock_setup();
	gpio_setup();

	gpio_clear(GPIOA, GPIO8); //关USB上拉 
	
	usb_init();

	uart_setup();
		
	interrupt_setup();/* 开中断 */

	for (i = 0; i < 0x80000; i++)
		__asm__("nop");
	gpio_set(GPIOA, GPIO8);//开USB上拉
		
	while(1)
	{
		usbd_poll(usbd_dev_handler);
		if(attached)
			usb_packet_handler();
		__asm__("nop");
	}

	return 0;
}
