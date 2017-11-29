/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/can.h>

#include "general.h"
#include "cdcacm.h"

#define USBUART_TIMER_FREQ_HZ 1000000U /* 1us per tick */
#define USBUART_RUN_FREQ_HZ 5000U /* 200us (or 100 characters at 2Mbps) */

#define FIFO_SIZE 128

#define TYPE_SMALL_PACKET 0x3
#define TYPE_PACKET_NORMAL 0x0
#define TYPE_PACKET_START 0x1
#define TYPE_PACKET_STOP 0x2

#define PROCESS_LENGTH		1
#define PROCESS_SOURCE		2
#define PROCESS_DATAS		3
#define PROCESS_SEND_START	4
#define PROCESS_SEND_NORMAL	5
#define PROCESS_SEND_STOP	6
#define PROCESS_SEND_SMALL	7

#define CANID_TO_TYPE(canid) ((canid) >> 8)
#define CANID_TO_ID(canid) ((canid) & 0xff)

#define ASEBA_MAX_INNER_PACKET_SIZE 1048

typedef union {
    uint8_t u8[2];
    uint16_t u16;
} uint16_8_t;

static uint8_t can_rx_buf[ASEBA_MAX_INNER_PACKET_SIZE];
static uint32_t can_rx_pos = 0;

static uint8_t can_tx_buf[ASEBA_MAX_INNER_PACKET_SIZE];
static uint32_t can_tx_in = 0;
static uint32_t can_tx_out = 0; 
static uint32_t can_tx_count = 0;

#ifdef EPUCK2
extern uint32_t uartUsed;
#else
uint32_t uartUsed = USBUSART;
#endif /* EPUCK2 */

/* RX Fifo buffer */
static uint8_t buf_rx[FIFO_SIZE];
/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t buf_rx_in;
/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t buf_rx_out;

static void usbuart_run(void);

void usbuart_init(void)
{
	UART_PIN_SETUP();

#ifdef EPUCK2 //setup specific uart ports for e-puck2
	rcc_periph_clock_enable(USBUSART_ESP_CLK);

	/* Setup UART ESP parameters. */
	usart_set_baudrate(USBUSART_ESP, 38400);
	usart_set_databits(USBUSART_ESP, 8);
	usart_set_stopbits(USBUSART_ESP, USART_STOPBITS_1);
	usart_set_mode(USBUSART_ESP, USART_MODE_TX_RX);
	usart_set_parity(USBUSART_ESP, USART_PARITY_NONE);
	usart_set_flow_control(USBUSART_ESP, USART_FLOWCONTROL_NONE);
	/* Enable interrupts */
	USBUSART_ESP_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(USBUSART_ESP_IRQ, IRQ_PRI_USBUSART);

	rcc_periph_clock_enable(USBUSART_407_CLK);

	/* Setup UART 407 parameters. */
	usart_set_baudrate(USBUSART_407, 38400);
	usart_set_databits(USBUSART_407, 8);
	usart_set_stopbits(USBUSART_407, USART_STOPBITS_1);
	usart_set_mode(USBUSART_407, USART_MODE_TX_RX);
	usart_set_parity(USBUSART_407, USART_PARITY_NONE);
	usart_set_flow_control(USBUSART_407, USART_FLOWCONTROL_NONE);

	/* Enable interrupts */
	USBUSART_407_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(USBUSART_407_IRQ, IRQ_PRI_USBUSART);

	// if(uartUsed == USBUSART_ESP){
	// 	/* Finally enable the USART. */
	// 	usart_enable(USBUSART_ESP);
	// 	nvic_enable_irq(USBUSART_ESP_IRQ);

	// }else if(uartUsed == USBUSART_407){
	// 	/* Finally enable the USART. */
	// 	usart_enable(USBUSART_407);
	// 	nvic_enable_irq(USBUSART_407_IRQ);
	// }
#else //setup standard UART for other platforms
	rcc_periph_clock_enable(USBUSART_CLK);

	/* Setup UART parameters. */
	usart_set_baudrate(USBUSART, 38400);
	usart_set_databits(USBUSART, 8);
	usart_set_stopbits(USBUSART, USART_STOPBITS_1);
	usart_set_mode(USBUSART, USART_MODE_TX_RX);
	usart_set_parity(USBUSART, USART_PARITY_NONE);
	usart_set_flow_control(USBUSART, USART_FLOWCONTROL_NONE);
	/* Enable interrupts */
	USBUSART_CR1 |= USART_CR1_RXNEIE;
	nvic_set_priority(USBUSART_IRQ, IRQ_PRI_USBUSART);

	usart_enable(USBUSART);
	nvic_enable_irq(USBUSART_IRQ);
#endif /* EPUCK2 */

	/* Setup timer for running deferred FIFO processing */
	USBUSART_TIM_CLK_EN();
	timer_reset(USBUSART_TIM);
	timer_set_mode(USBUSART_TIM, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(USBUSART_TIM,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);
	timer_set_period(USBUSART_TIM,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);

	/* Setup update interrupt in NVIC */
	nvic_set_priority(USBUSART_TIM_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_enable_irq(USBUSART_TIM_IRQ);

	/* turn the timer on */
	timer_enable_counter(USBUSART_TIM);
}

/*
 * Runs deferred processing for usb uart rx, draining RX FIFO by sending
 * characters to host PC via CDCACM.  Allowed to read from FIFO in pointer,
 * but not write to it. Allowed to write to FIFO out pointer.
 */
static void usbuart_run(void)
{
	/* forcibly empty fifo if no USB endpoint */
	if (cdcacm_get_config() != 1)
	{
		buf_rx_out = buf_rx_in;
	}

	/* if fifo empty, nothing further to do */
	if (buf_rx_in == buf_rx_out) {
		/* turn off LED, disable IRQ */
		timer_disable_irq(USBUSART_TIM, TIM_DIER_UIE);
		gpio_clear(LED_PORT_UART, LED_UART);
	}
	else
	{
		uint8_t packet_buf[CDCACM_PACKET_SIZE];
		uint8_t packet_size = 0;
		uint8_t buf_out = buf_rx_out;

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
		buf_rx_out += usbd_ep_write_packet(usbdev,
				CDCACM_UART_ENDPOINT, packet_buf, packet_size);
		buf_rx_out %= FIFO_SIZE;
	}
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
	usart_set_baudrate(uartUsed, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(uartUsed, coding->bDataBits + 1);
	else
		usart_set_databits(uartUsed, coding->bDataBits);

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(uartUsed, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(uartUsed, USART_STOPBITS_1_5);
		break;
	case 2:
		usart_set_stopbits(uartUsed, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(uartUsed, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(uartUsed, USART_PARITY_ODD);
		break;
	case 2:
		usart_set_parity(uartUsed, USART_PARITY_EVEN);
		break;
	}
}

void aseba_can_transmit(uint8_t* data, uint16_t length, uint16_t source, uint32_t type){


	while(can_transmit(CAN1,
		 (((type) << 8) | (source)),     /* (EX/ST)ID: CAN ID */
		 false, /* IDE: CAN ID extended? */
		 false, /* RTR: Request transmit? */
		 length,     /* DLC: Data length */
		 data) == -1);

	// if (length <= 8)
	// {
	// 	while(can_transmit(CAN1,
	// 		 (((TYPE_SMALL_PACKET) << 8) | (source)),     /* (EX/ST)ID: CAN ID */
	// 		 false, /* IDE: CAN ID extended? */
	// 		 false, /* RTR: Request transmit? */
	// 		 length,     /* DLC: Data length */
	// 		 data) == -1);
	// }
	// else
	// {
	// 	size_t pos = 8;
		
	// 	while(can_transmit(CAN1,
	// 		 (((TYPE_PACKET_START) << 8) | (source)),     /* (EX/ST)ID: CAN ID */
	// 		 false, /* IDE: CAN ID extended? */
	// 		 false, /* RTR: Request transmit? */
	// 		 8,     /* DLC: Data length */
	// 		 data) == -1);

	// 	while (pos + 8 < length)
	// 	{
	// 		while(can_transmit(CAN1,
	// 			 (((TYPE_PACKET_NORMAL) << 8) | (source)),     /* (EX/ST)ID: CAN ID */
	// 			 false, /* IDE: CAN ID extended? */
	// 			 false, /* RTR: Request transmit? */
	// 			 8,     /* DLC: Data length */
	// 			 data + pos) == -1);
	// 		pos += 8;
	// 	}
	// 	while(can_transmit(CAN1,
	// 		 (((TYPE_PACKET_STOP) << 8) | (source)),     /* (EX/ST)ID: CAN ID */
	// 		 false, /* IDE: CAN ID extended? */
	// 		 false, /* RTR: Request transmit? */
	// 		 length - pos,     /* DLC: Data length */
	// 		 data + pos) == -1);
	// }
}

void usbuart_usb_out_cb(usbd_device *dev, uint8_t ep)
{
	(void)ep;

	char buf[CDCACM_PACKET_SIZE];
	int len = usbd_ep_read_packet(dev, CDCACM_UART_ENDPOINT,
					buf, CDCACM_PACKET_SIZE);

#if defined(BLACKMAGIC)
	/* Don't bother if uart is disabled.
	 * This will be the case on mini while we're being debugged.
	 */
	if(!(RCC_APB2ENR & RCC_APB2ENR_USART1EN))
		return;
#endif

	gpio_set(LED_PORT_UART, LED_UART);
	// for(int i = 0; i < len; i++)
	// 	usart_send_blocking(uartUsed, buf[i]);
	// gpio_clear(LED_PORT_UART, LED_UART);

	if(len && ((len + can_tx_count) <= ASEBA_MAX_INNER_PACKET_SIZE)){
		uint8_t i = 0;
		for(i = 0 ; i < len ; i++){
			can_tx_buf[can_tx_in++] = buf[i];
			//circular buffer
			if(can_tx_in >= ASEBA_MAX_INNER_PACKET_SIZE){
				can_tx_in = 0;
			}
		}
		can_tx_count += len;
	}
	

}

#ifdef USBUART_DEBUG
int usbuart_debug_write(const char *buf, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (buf[i] == '\n') {
			buf_rx[buf_rx_in++] = '\r';
			buf_rx_in %= FIFO_SIZE;
		}
		buf_rx[buf_rx_in++] = buf[i];
		buf_rx_in %= FIFO_SIZE;
	}
	/* enable deferred processing if we put data in the FIFO */
	timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	return len;
}
#endif

void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
	(void) ep;
}

/*
 * Read a character from the UART RX and stuff it in a software FIFO.
 * Allowed to read from FIFO out pointer, but not write to it.
 * Allowed to write to FIFO in pointer.
 */
void usbuart_isr(void){
	uint32_t err = USART_SR(uartUsed);
	char c = usart_recv(uartUsed);
	if (err & (USART_SR_ORE | USART_SR_FE))
		return;

	/* Turn on LED */
	gpio_set(LED_PORT_UART, LED_UART);

	/* If the next increment of rx_in would put it at the same point
	* as rx_out, the FIFO is considered full.
	*/
	if (((buf_rx_in + 1) % FIFO_SIZE) != buf_rx_out)
	{
		/* insert into FIFO */
		buf_rx[buf_rx_in++] = c;

		/* wrap out pointer */
		if (buf_rx_in >= FIFO_SIZE)
		{
			buf_rx_in = 0;
		}

		/* enable deferred processing if we put data in the FIFO */
		timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	}
}

void USBUSART_ESP_ISR(void)
{
	usbuart_isr();
}

void USBUSART_407_ISR(void)
{
	usbuart_isr();
}

void process_usbcan(void){

	static uint8_t state = PROCESS_LENGTH;
	static uint32_t i = 0;
	static uint16_8_t length, source; 
	static uint8_t data_buffer[8];
	static uint32_t data_sent = 0;
	static uint32_t count = 0;
	static uint32_t decrement = 0 ;

	count = can_tx_count;
	decrement = 0;

	if(state == PROCESS_LENGTH){
		if(count >= sizeof(length)){
			for(i = 0 ; i < sizeof(length) ; i++){
				length.u8[i] = can_tx_buf[can_tx_out++];
				decrement++;
				if(can_tx_out >= ASEBA_MAX_INNER_PACKET_SIZE){
					can_tx_out = 0;
				}
			}
			length.u16 += 2; // Aseba transmits length minus the type. 
			state = PROCESS_SOURCE;
		}
	}
	if(state == PROCESS_SOURCE){
		if(count >= sizeof(source)){
			for(i = 0 ; i < sizeof(source) ; i++){
				source.u8[i] = can_tx_buf[can_tx_out++];
				decrement++;
				if(can_tx_out >= ASEBA_MAX_INNER_PACKET_SIZE){
					can_tx_out = 0;
				}
			}
			state = PROCESS_DATAS;
		}
	}
	if(state == PROCESS_DATAS){
		if(length.u16 <= 8){
			state = PROCESS_SEND_SMALL;
		}else{
			state = PROCESS_SEND_START;
		}
	}
	if(state == PROCESS_SEND_SMALL){
		if(count >= length.u16){
			for(i = 0 ; i < length.u16 ; i++){
				data_buffer[i] = can_tx_buf[can_tx_out++];
				decrement++;
				if(can_tx_out >= ASEBA_MAX_INNER_PACKET_SIZE){
					can_tx_out = 0;
				}
			}
			aseba_can_transmit(data_buffer, length.u16, source.u16, TYPE_SMALL_PACKET);
			state = PROCESS_LENGTH;
		}
	}
	if(state == PROCESS_SEND_START){
		if(count >= 8){
			for(i = 0 ; i < 8 ; i++){
				data_buffer[i] = can_tx_buf[can_tx_out++];
				decrement++;
				if(can_tx_out >= ASEBA_MAX_INNER_PACKET_SIZE){
					can_tx_out = 0;
				}
			}
			data_sent = 8;
			aseba_can_transmit(data_buffer, 8, source.u16, TYPE_PACKET_START);
			if((length.u16 - data_sent) <= 8){
				state = PROCESS_SEND_STOP;
			}else{
				state = PROCESS_SEND_NORMAL;
			}
		}
	}
	if(state == PROCESS_SEND_NORMAL){
		if(count >= 8){
			for(i = 0 ; i < 8 ; i++){
				data_buffer[i] = can_tx_buf[can_tx_out++];
				decrement++;
				if(can_tx_out >= ASEBA_MAX_INNER_PACKET_SIZE){
					can_tx_out = 0;
				}
			}
			data_sent += 8;
			aseba_can_transmit(data_buffer, 8, source.u16, TYPE_PACKET_NORMAL);
			if((length.u16 - data_sent) <= 8){
				state = PROCESS_SEND_STOP;
			}
		}
	}
	if(state == PROCESS_SEND_STOP){
		if(count >= (length.u16 - data_sent)){
			for(i = 0 ; i < (length.u16 - data_sent) ; i++){
				data_buffer[i] = can_tx_buf[can_tx_out++];
				decrement++;
				if(can_tx_out >= ASEBA_MAX_INNER_PACKET_SIZE){
					can_tx_out = 0;
				}
			}
			data_sent += 8;
			aseba_can_transmit(data_buffer, (length.u16 - data_sent), source.u16, TYPE_PACKET_STOP);
			state = PROCESS_LENGTH;
		}
	}
	can_tx_count -= decrement;
}

void USBUSART_TIM_ISR(void)
{
	/* need to clear timer update event */
	timer_clear_flag(USBUSART_TIM, TIM_SR_UIF);

	/* process FIFO */
	if(0){
		usbuart_run();
	}
	process_usbcan();
}

void usbcan_run(void){

	/* send nothing if no USB endpoint */
	if (cdcacm_get_config() != 1)
	{
		return;
	}

	can_rx_pos += 4;
	if(can_rx_pos > CDCACM_PACKET_SIZE){
		uint16_t times_to_send = can_rx_pos / CDCACM_PACKET_SIZE;
		uint16_t rest_to_send = can_rx_pos % CDCACM_PACKET_SIZE;
		uint16_t i = 0;

		while(i < times_to_send){
			while(usbd_ep_write_packet(usbdev,CDCACM_UART_ENDPOINT,
			 &can_rx_buf[i * CDCACM_PACKET_SIZE], CDCACM_PACKET_SIZE) <= 0);
			i++;
		}
		if( can_rx_pos > CDCACM_PACKET_SIZE){
			while(usbd_ep_write_packet(usbdev,CDCACM_UART_ENDPOINT,
			 &can_rx_buf[i * CDCACM_PACKET_SIZE], rest_to_send) <= 0);
		}
	}else{
		while(usbd_ep_write_packet(usbdev,CDCACM_UART_ENDPOINT,
		 can_rx_buf, can_rx_pos) <= 0);
	}

	can_rx_pos = 0;
	
}

void CAN_RX_ISR(void)
{
	int8_t i = 0;
	uint32_t id;
	bool ext, rtr;
	uint8_t fmi, len, data[8];

	can_receive(CAN1, 0, true, &id, &ext, &rtr, &fmi, &len, data, NULL);

	uint16_8_t source, length;

	source.u16 = CANID_TO_ID(id);
	length.u16 = len - 2; /* Aseba transmits length minus the type. */

	if(CANID_TO_TYPE(id) == TYPE_SMALL_PACKET){

		for(i = 0 ; i < len ; i++){
			can_rx_buf[i + 4 + can_rx_pos] = data[i];
		}
		can_rx_pos += len;

		length.u16 = len - 2;  // Aseba transmits length minus the type. 

		can_rx_buf[0] = length.u8[0];
		can_rx_buf[1] = length.u8[1];	
		can_rx_buf[2] = source.u8[0];
		can_rx_buf[3] = source.u8[1];

		usbcan_run();

	}else if(CANID_TO_TYPE(id) == TYPE_PACKET_STOP){

		for(i = 0 ; i < len ; i++){
			can_rx_buf[i + 4 + can_rx_pos] = data[i];
		}
		can_rx_pos += len;

		length.u16 = can_rx_pos - 2; //Aseba transmits length minus the type.

		can_rx_buf[0] = length.u8[0];
		can_rx_buf[1] = length.u8[1];	
		can_rx_buf[2] = source.u8[0];
		can_rx_buf[3] = source.u8[1];

		usbcan_run();

	}else{//TYPE_PACKET_NORMAL or TYPE_PACKET_START //just fill the buffer

		for(i = 0 ; i < len ; i++){
			can_rx_buf[i + 4 + can_rx_pos] = data[i];
		}

		can_rx_pos += len;

	}

	

}

void usbcan_init(void)
{
	/* Enable peripheral clocks. */
	rcc_periph_clock_enable(RCC_CAN1);

	CAN_PIN_SETUP();

	/* NVIC setup. */
	nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
	nvic_set_priority(NVIC_CAN1_RX0_IRQ, IRQ_PRI_CAN_RX);

	/* Reset CAN. */
	can_reset(CAN1);

	/* CAN cell init.
     * Setting the bitrate to 1MBit. APB1 = 48MHz,
     * prescaler = 3 -> 16MHz time quanta frequency.
     * 1tq sync + 9tq bit segment1 (TS1) + 6tq bit segment2 (TS2) =
     * 16time quanta per bit period, therefor 16MHz/16 = 1MHz
     */
    if (can_init(CAN1,          // Interface
             false,             // Time triggered communication mode.
             true,              // Automatic bus-off management.
             false,             // Automatic wakeup mode.
             false,             // No automatic retransmission.
             false,             // Receive FIFO locked mode.
             false,             // Transmit FIFO priority.
             CAN_BTR_SJW_1TQ,   // Resynchronization time quanta jump width
             CAN_BTR_TS1_9TQ,  	// Time segment 1 time quanta width
             CAN_BTR_TS2_6TQ,   // Time segment 2 time quanta width
             3,                 // Prescaler
             false,             // Loopback
             false)) {          // Silent
    }

    can_filter_id_mask_32bit_init(
        CAN1,
        0,      // nr
        0,      // id
        0,      // mask
        0,      // fifo
        true    // enable
    ); // match any id

	/* Enable CAN RX interrupt. */
	can_enable_irq(CAN1, CAN_IER_FMPIE0);

	/* enable deferred processing if we put data in the FIFO */
	timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
}

#ifdef ENABLE_DEBUG
enum {
	RDI_SYS_OPEN = 0x01,
	RDI_SYS_WRITE = 0x05,
	RDI_SYS_ISTTY = 0x09,
};

int rdi_write(int fn, const char *buf, size_t len)
{
	(void)fn;
	if (debug_bmp)
		return len - usbuart_debug_write(buf, len);

	return 0;
}

struct ex_frame {
	union {
		int syscall;
		int retval;
	};
	const int *params;
	uint32_t r2, r3, r12, lr, pc;
};

void debug_monitor_handler_c(struct ex_frame *sp)
{
	/* Return to after breakpoint instruction */
	sp->pc += 2;

	switch (sp->syscall) {
	case RDI_SYS_OPEN:
		sp->retval = 1;
		break;
	case RDI_SYS_WRITE:
		sp->retval = rdi_write(sp->params[0], (void*)sp->params[1], sp->params[2]);
		break;
	case RDI_SYS_ISTTY:
		sp->retval = 1;
		break;
	default:
		sp->retval = -1;
	}

}

asm(".globl debug_monitor_handler\n"
    ".thumb_func\n"
    "debug_monitor_handler: \n"
    "    mov r0, sp\n"
    "    b debug_monitor_handler_c\n");

#endif
