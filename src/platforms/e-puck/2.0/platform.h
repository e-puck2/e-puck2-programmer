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

#if defined(PLATFORM_HAS_COMMANDS)
#define COMMANDS_OPTION "C"
#else
#define COMMANDS_OPTION "c"
#endif

#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
#error "NO_DFU option must be defined for this platform !!"
#endif

#ifndef PLATFORM_HAS_NO_JTAG
#error "NO_JTAG option must be defined for this platform !!"
#endif

#if defined(PLATFORM_HAS_NO_SERIAL)
#define SERIAL_OPTION "s"
#else
#define SERIAL_OPTION "S"
#endif

#if defined(PLATFORM_HAS_TRACESWO)
#error "TRACESWO option is not usable yet on this platform !!"
#endif

#define PLATFORM_OPTIONS	\
				COMMANDS_OPTION		\
				SERIAL_OPTION			\
/*			TRACESWO_OPTION 	\*/

//#pragma message "Platform options : " PLATFORM_OPTIONS

// Define the identification's names of the device
#define BOARD_IDENT "Black Magic Probe (e-puck2)-(Options " PLATFORM_OPTIONS ")-(Firmware " FIRMWARE_VERSION ")"
#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
#define DFU_IDENT   "Black Magic Firmware Upgrade (e-puck2)"
#endif

/* Important pin mappings for e-puck2 implementation:
 *
 * PA15	(Green LED 	: Running/Idle)
 * PB13	(Red LED    : Error)
 * PB15	(Blue LED   : Serial)
 * There is no BootlLoader active LED because this mode is intrinsic to the uC
 *
 * JTAG is not usable on this platform then pins can be reduced
 * TRACESWO is not usable on this platform then pins can be reduced
 *
 * nTRST	:	To be checked but not needed
 * SRST_OUT	:	To be checked but seems not needed
 * TDI		:	To be checked but not needed
 * TMS		:	PA8/SWDIO
 * TCK		:	PA5/SWCLK
 * TDO		:	To be checked but not needed (input for TRACESWO)
 * nSRST	:	PB4
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
#define TMS_PIN		GPIO8
#define TCK_PIN		GPIO5
#define TDO_PIN		NOT_USED

#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	NOT_USED
#define TRST_PIN	NOT_USED
#define SRST_PORT	GPIOB
#define SRST_PIN	GPIO4

#define LED_PORT_ERROR		GPIOB
#define LED_PORT_UART		GPIOB
#define LED_PORT			GPIOA  //IDLE_RUN


#define LED_ERROR		GPIO13
#define LED_UART		GPIO15
#define LED_IDLE_RUN	GPIO15
#define LED_BOOTLOADER	NOT_USED

#define PWR_ON_PORT		GPIOA
#define PWR_ON_PIN		GPIO6

#define PWR_ON_BTN_PORT GPIOA
#define PWR_ON_BTN_PIN GPIO4
#define PWR_ON_BTN_EXTI EXTI4
#define PWR_ON_BTN_EXTI_ISR	exti4_isr
#define PWR_ON_BTN_EXTI_IRQ NVIC_EXTI4_IRQ
#define PWR_ON_BTN_TIM TIM2
#define PWR_ON_BTN_TIM_ISR tim2_isr
#define TURN_ON_TIME 10
#define TURN_OFF_TIME 100
#define ROBOT_OFF 0
#define ROBOT_ON 1

#define VBUS_PORT	GPIOA
#define VBUS_PIN	GPIO9

#define USB_CHARGE_PORT	GPIOB
#define USB_CHARGE_PIN	GPIO5

#define USB_500_PORT	GPIOB
#define USB_500_PIN 	GPIO3

#define EN_ESP32_PORT	GPIOC
#define EN_ESP32_PIN	GPIO13

#define GPIO0_ESP32_PORT	GPIOB
#define GPIO0_ESP32_PIN		GPIO1

#define I2C_HUB			I2C1
#define I2C_RCC_HUB		RCC_I2C1
#define I2C_PORT		GPIOB
#define I2C_SDA_PIN		GPIO7
#define I2C_SDA_AF		GPIO_AF4
#define I2C_SCL_PIN		GPIO6
#define I2C_SCL_AF		GPIO_AF4

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
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, !(state==1));}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT_ERROR, LED_ERROR, !(state==1));}

static inline int platform_hwversion(void)
{
	return 0;
}
//theses diefines exist because we can not include the file containing it here
//aka. flash_common_f24.h
#define FLASH_ACR_ICE_COPY				(1 << 9)
#define FLASH_ACR_DCE_COPY				(1 << 10)
#define FLASH_ACR_LATENCY_3WS_COPY		0x03

void platform_set_en_esp32(bool assert);
bool platform_get_en_esp32(void);

void platform_set_gpio0_esp32(bool assert);
bool platform_get_gpio0_esp32(void);

void platform_pwr_on(bool on_state);
bool platform_pwr_on_btn_pressed(void);

bool platform_vbus(void);

void platform_set_usb_charge(bool assert);
bool platform_get_usb_charge(void);

void platform_set_usb_500(bool assert);
bool platform_get_usb_500(void);

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif
