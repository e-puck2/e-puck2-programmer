/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"
#include "version.h"

#include <setjmp.h>

// Define the identification's names of the device
#define BOARD_IDENT "Black Magic Probe (e-puck2), (Firmware " FIRMWARE_VERSION ")"
#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
#define DFU_IDENT   "Black Magic Firmware Upgrade (e-puck2)"
#endif

/* Important pin mappings for e-puck2 implementation:
 *
 * PA15	(Green LED 	: Running/Idle)
 * PA13	(Red LED    : Error)
 * PA14	(Blue LED   : Serial)
 * There is no BootlLoader active LED because this mode is intrinsic to the uC
 *
 * JTAG is not usable on this platform then pins can be reduced
 * TRACESWO is not usable on this platform then pins can be reduced
 *
 * nTRST	:	To be checked but not needed
 * SRST_OUT	:	To be checked but seems not needed
 * TDI		:	To be checked but not needed
 * TMS		:	PA10/SWDIO
 * TCK		:	PA5/SWCLK
 * TDO		:	To be checked but not needed (input for TRACESWO)
 * nSRST	:	PB0
 *
 * DFU mode selection : Boot0 with a jumper
 */

/* Hardware definitions... */
#define NOT_USED	0
#define JTAG_PORT GPIOA
#define TDI_PORT	NOT_USED
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	NOT_USED
#define TDI_PIN		NOT_USED
#define TMS_PIN		GPIO10
#define TCK_PIN		GPIO5
#define TDO_PIN		NOT_USED

#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	NOT_USED
#define TRST_PIN	NOT_USED
#define SRST_PORT	GPIOB
#define SRST_PIN	GPIO0

#define LED_PORT		GPIOA
#define LED_PORT_UART	GPIOA
#define LED_UART		GPIO14
#define LED_IDLE_RUN	GPIO15
#define LED_ERROR		GPIO13
#define LED_BOOTLOADER	NOT_USED

#define TMS_SET_MODE() \
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, \
	                GPIO_PUPD_NONE, TMS_PIN);
#define SWDIO_MODE_FLOAT() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, \
	                GPIO_PUPD_NONE, SWDIO_PIN);

#define SWDIO_MODE_DRIVE() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, \
	                GPIO_PUPD_NONE, SWDIO_PIN);


#define USB_DRIVER      stm32f107_usb_driver
#define USB_IRQ         NVIC_OTG_FS_IRQ
#define USB_ISR         otg_fs_isr

/* Interrupt priorities.  Low numbers are high priority.
 * For now USART2 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(2 << 4)

#ifndef PLATFORM_HAS_NO_SERIAL
#define IRQ_PRI_USBUSART	(1 << 4)
#define IRQ_PRI_USBUSART_TIM	(3 << 4)
#endif

#define IRQ_PRI_TRACE		(0 << 4)

#ifndef PLATFORM_HAS_NO_SERIAL
#define USBUSART USART2
#define USBUSART_CR1 USART2_CR1
#define USBUSART_IRQ NVIC_USART2_IRQ
#define USBUSART_CLK RCC_USART2
#define USBUSART_TX_PORT GPIOA
#define USBUSART_TX_PIN  GPIO2
#define USBUSART_RX_PORT GPIOA
#define USBUSART_RX_PIN  GPIO3
#define USBUSART_ISR usart2_isr
#define USBUSART_TIM TIM4
#define USBUSART_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM4)
#define USBUSART_TIM_IRQ NVIC_TIM4_IRQ
#define USBUSART_TIM_ISR tim4_isr

#define UART_PIN_SETUP() do { \
	gpio_mode_setup(USBUSART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_TX_PIN); \
	gpio_mode_setup(USBUSART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_RX_PIN); \
	gpio_set_af(USBUSART_TX_PORT, GPIO_AF7, USBUSART_TX_PIN); \
	gpio_set_af(USBUSART_RX_PORT, GPIO_AF7, USBUSART_RX_PIN); \
    } while(0)
#endif

#ifdef PLATFORM_HAS_TRACESWO
#define TRACE_TIM TIM3
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ   NVIC_TIM3_IRQ
#define TRACE_ISR   tim3_isr
#endif

#define DEBUG(...)

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT, LED_ERROR, state);}

static inline int platform_hwversion(void)
{
	return 0;
}

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif
