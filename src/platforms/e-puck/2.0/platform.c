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

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"
#include "morse.h"

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <../USB251XB/USB251XB.h>
#include <../USB251XB/SMBus.h>

#pragma message "Platform options : " PLATFORM_OPTIONS

jmp_buf fatal_error_jmpbuf;
extern uint32_t _ebss;

void platform_init(void)
{
#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
	volatile uint32_t *magic = (uint32_t *) &_ebss;
	/* Check the USER button*/
#endif
	rcc_periph_clock_enable(RCC_GPIOA);		// Necessary for other GPIOA
#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
	if (gpio_get(GPIOA, GPIO0) ||
	   ((magic[0] == BOOTMAGIC0) && (magic[1] == BOOTMAGIC1))) {
		magic[0] = 0;
		magic[1] = 0;
		/* Assert blue LED as indicator we are in the bootloader */
		rcc_periph_clock_enable(RCC_GPIOD);
		gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
						GPIO_PUPD_NONE, LED_BOOTLOADER);
		gpio_set(LED_PORT, LED_BOOTLOADER);
		/* Jump to the built in bootloader by mapping System flash.
		   As we just come out of reset, no other deinit is needed!*/
		rcc_periph_clock_enable(RCC_SYSCFG);
		SYSCFG_MEMRM &= ~3;
		SYSCFG_MEMRM |=  1;
		scb_reset_core();
	}
#endif

	/* If ON/OFF button pressed then maintain the power supply. */
	if (platform_pwr_on_btn()) {
		gpio_mode_setup(POWERFUNC_PORT, GPIO_MODE_OUTPUT,
				GPIO_PUPD_NONE, PWR_ON_BTN_PIN);
		platform_pwr_on(true);
	}
	/* Else the power supply will stay ON only as long as the ON/OFF button
	   will stay pressed, then will die !! */

	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_48MHZ]);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
//	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_CRC);

	SMBus_init();

	USB251XB_init(USB2512B);

	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOC_OSPEEDR &=~0xF30;
	GPIOC_OSPEEDR |= 0xA20;
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			TMS_PIN | TCK_PIN | TDI_PIN);

	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT,
			GPIO_PUPD_NONE,
			TDO_PIN);

	gpio_set(SRST_PORT, SRST_PIN);
	gpio_set_output_options(SRST_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,SRST_PIN);
	gpio_mode_setup(SRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SRST_PIN);

	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE,
			LED_UART | LED_IDLE_RUN | LED_ERROR | LED_BOOTLOADER);

	gpio_clear(EN_ESP32_PORT, EN_ESP32_PIN);
	gpio_set_output_options(EN_ESP32_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_2MHZ,EN_ESP32_PIN);
	gpio_mode_setup(EN_ESP32_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN_ESP32_PIN);

	platform_timing_init();
#ifndef PLATFORM_HAS_NO_SERIAL
	usbuart_init();
#endif
	cdcacm_init();
}

void platform_srst_set_val(bool assert)
{
	volatile int i;
	if (assert) {
		gpio_clear(SRST_PORT, SRST_PIN);
		for(i = 0; i < 10000; i++) asm("nop");
	} else {
		gpio_set(SRST_PORT, SRST_PIN);
	}
}

bool platform_srst_get_val(void)
{
	return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

void platform_set_en_esp32(bool assert)
{
	if (assert)
		gpio_set(EN_ESP32_PORT, EN_ESP32_PIN);
	else
		gpio_clear(EN_ESP32_PORT, EN_ESP32_PIN);
}

bool platform_get_en_esp32(void)
{
	return gpio_get(EN_ESP32_PORT, EN_ESP32_PIN) == 1;
}

void platform_pwr_on(bool on_state)
{
	if (on_state)
		gpio_set(POWERFUNC_PORT, PWR_ON_BTN_PIN);
	else
		gpio_clear(POWERFUNC_PORT, PWR_ON_BTN_PIN);
}

bool platform_pwr_on_btn(void)
{
	/* Return true if button pressed else false. */
	return !gpio_get(POWERFUNC_PORT, PWR_ON_PIN);
}

bool platform_vbus(void)
{
	/* Return true if button pressed else false. */
	return !gpio_get(VBUS_PORT, VBUS_PIN);
}

void platform_set_usb_charge(bool assert)
{
	if (assert)
		gpio_set(USB_CHARGE_PORT, USB_CHARGE_PIN);
	else
		gpio_clear(USB_CHARGE_PORT, USB_CHARGE_PIN);
}

bool platform_get_usb_charge(void)
{
	return gpio_get(USB_CHARGE_PORT, USB_CHARGE_PIN) == 1;
}

void platform_set_usb_500(bool assert)
{
	if (assert)
		gpio_set(USB_500_PORT, USB_500_PIN);
	else
		gpio_clear(USB_500_PORT, USB_500_PIN);
}

bool platform_get_usb_500(void)
{
	return gpio_get(USB_500_PORT, USB_500_PIN) == 1;
}

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}

#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
void platform_request_boot(void)
{
	uint32_t *magic = (uint32_t *) &_ebss;
	magic[0] = BOOTMAGIC0;
	magic[1] = BOOTMAGIC1;
	scb_reset_system();
}
#endif
