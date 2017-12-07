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
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/otg_fs.h>

#include <../USB251XB/USB251XB.h>
#include <../USB251XB/SMBus.h>

const struct rcc_clock_scale hse_24mhz_to_96mhz_413_epuck = {
	 /* 96MHz */
	.pllm = 12,
	.plln = 96,
	.pllp = 2,
	.pllq = 4,
	.pllr = 0,
	.hpre = RCC_CFGR_HPRE_DIV_NONE,
	.ppre1 = RCC_CFGR_PPRE_DIV_2,
	.ppre2 = RCC_CFGR_PPRE_DIV_NONE,
	//power_save = 1 if ahb_frequency <= 84MHz
	//cf. 5.4.1 PWR power control register (PWR_CR) of the
	//stm32f413 reference manual
	.power_save = 0,
	.flash_config = FLASH_ACR_ICE_COPY | FLASH_ACR_DCE_COPY |
			FLASH_ACR_LATENCY_3WS_COPY,
	.ahb_frequency = 96000000,
	.apb1_frequency = 48000000,
	.apb2_frequency = 96000000,
};

//#pragma message "Platform options : " PLATFORM_OPTIONS

jmp_buf fatal_error_jmpbuf;
extern uint32_t _ebss;
uint16_t pwrBtnCounter = 0;
uint8_t pwrBtnState = ROBOT_OFF;
uint8_t hub_state = NOT_CONFIGURED;
//CAN has priority over uart. So if canUsed is set to true, the usbuart is disabled
uint32_t uartUsed = USBUSART_ESP;
bool canUsed = false; 

void PWR_ON_BTN_TIM_ISR(void) {
	/* need to clear timer update event */
	timer_clear_flag(PWR_ON_BTN_TIM, TIM_SR_UIF);
	pwrBtnCounter++;
	if((pwrBtnState==ROBOT_OFF) && (pwrBtnCounter>=TURN_ON_TIME)) {
		pwrBtnState = ROBOT_ON;
		timer_disable_counter(PWR_ON_BTN_TIM);
		platform_pwr_on(true);
	}
	if((pwrBtnState==ROBOT_ON) && (pwrBtnCounter>=TURN_OFF_TIME)) {
		pwrBtnState = ROBOT_OFF;
		timer_disable_counter(PWR_ON_BTN_TIM);
		platform_pwr_on(false);
	}
//	gpio_toggle(LED_PORT_ERROR, LED_ERROR);
}

void PWR_ON_BTN_EXTI_ISR(void) {
	if(exti_get_flag_status(PWR_ON_BTN_EXTI)) {
		exti_reset_request(PWR_ON_BTN_EXTI);
		if(!platform_pwr_on_btn_pressed()) { // Button released.
			timer_disable_counter(PWR_ON_BTN_TIM);
//			gpio_set(LED_PORT_UART, LED_UART);
		} else { // Button pressed.
			pwrBtnCounter = 0;
			timer_enable_counter(PWR_ON_BTN_TIM);
//			gpio_clear(LED_PORT_UART, LED_UART);
		}
	}
}

void VBUS_EXTI_ISR(void) {
	if(exti_get_flag_status(VBUS_EXTI)) {
		//wait a few moments to be sure the interruption was not triggered
		//by a glitch and then test the GPIO
		//also wait for the USB HUB to be running
		platform_delay(100);
		if(platform_get_vbus()){
			if(hub_state == NOT_CONFIGURED){
				// //reinit SMBus because of BUSY flag being kept high
				// //when certain glitches appear on the I2C bus.
				SMBus_init();
				USB251XB_init(USB2512B);
				hub_state = CONFIGURED;
			}
		}else{
			hub_state = NOT_CONFIGURED;
		}
		exti_reset_request(VBUS_EXTI);
	 }
}

void setup_vbus_detection(void){

	gpio_mode_setup(VBUS_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, VBUS_PIN);

	// Configure the EXTI subsystem.
	exti_select_source(VBUS_EXTI, VBUS_PORT);
	exti_set_trigger(VBUS_EXTI, EXTI_TRIGGER_BOTH);
	exti_enable_request(VBUS_EXTI);

	exti_reset_request(VBUS_EXTI);
	nvic_set_priority(VBUS_EXTI_IRQ, VBUS_EXTI_ISR_PRI);
	// Enable EXTI interrupt.
	nvic_enable_irq(VBUS_EXTI_IRQ);

	//test if vbus is present on power on to configure the hub
	//because the interruption can not be triggered on power on
	if(platform_get_vbus()){
		if(hub_state == NOT_CONFIGURED){
			//wait for the USB HUB to be running
			platform_delay(100);
			// //reinit SMBus because of BUSY flag being kept high
			// //when certain glitches appear on the I2C bus.
			SMBus_init();
			USB251XB_init(USB2512B);
			hub_state = CONFIGURED;
		}
	}
}

void setup_pwr_button() {
	// Enable EXTI9_5 interrupt (power button connected to PA7).
	nvic_enable_irq(PWR_ON_BTN_EXTI_IRQ);

	gpio_mode_setup(PWR_ON_BTN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PWR_ON_BTN_PIN);

	// Configure the EXTI subsystem.
	exti_select_source(PWR_ON_BTN_EXTI, PWR_ON_BTN_PORT);
	exti_set_trigger(PWR_ON_BTN_EXTI, EXTI_TRIGGER_BOTH);
	exti_enable_request(PWR_ON_BTN_EXTI);

	// Configure the timer to count how much time the button is pressed.
	// Enable TIM2 clock.
	rcc_periph_clock_enable(RCC_TIM2);

	// Reset TIM2 peripheral to defaults.
	timer_reset(PWR_ON_BTN_TIM);

	// Timer mode: internal clock source, no divider, alignment edge, direction up.
	timer_set_mode(PWR_ON_BTN_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	// Timer2 clock source is APB1 x 2. Set the prescaler to have the timer run at 1 KHz.
	timer_set_prescaler(PWR_ON_BTN_TIM, ((rcc_apb1_frequency * 2) / 1000)-1);

	// An interrupt every 10 ms.
	timer_set_period(PWR_ON_BTN_TIM, 10);

	// Enable TIM2 interrupt.
	nvic_enable_irq(NVIC_TIM2_IRQ);
	timer_enable_irq(PWR_ON_BTN_TIM, TIM_DIER_UIE);

	// Needed if no USB connexion : Test if PowerOn button is just pressed then power up the system
  if(platform_pwr_on_btn_pressed()) {
		pwrBtnCounter = 0;
		timer_enable_counter(PWR_ON_BTN_TIM);
	}

}

void platform_init(void)
{
	rcc_osc_bypass_enable(RCC_HSE);
	//rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_48MHZ]);
	rcc_clock_setup_hse_3v3(&hse_24mhz_to_96mhz_413_epuck);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_SYSCFG);

	gpio_set(LED_PORT_ERROR, LED_ERROR);
	gpio_mode_setup(LED_PORT_ERROR, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_ERROR);

	gpio_set(LED_PORT_UART, LED_UART);
	gpio_mode_setup(LED_PORT_UART, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_UART);

	gpio_set(LED_PORT, LED_IDLE_RUN);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_IDLE_RUN);

	gpio_clear(PWR_ON_PORT, PWR_ON_PIN);
	gpio_set_output_options(PWR_ON_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_2MHZ,PWR_ON_PIN);
	gpio_mode_setup(PWR_ON_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_ON_PIN);
	setup_pwr_button();

	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	// Configure the programming pins to "fast speed".
	GPIOA_OSPEEDR &= ~0x00030C00; // Reset PA5 (SWCLK), PA8 (SWDIO).
	GPIOA_OSPEEDR |= 0x00020800; // PA5 (SWCLK), PA8 (SWDIO) to fast speed.
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,  SWDIO_PIN | SWCLK_PIN);

/* Can be needed for TRACESWO but not yet. */
	//gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, TDO_PIN);

	gpio_set(SRST_PORT, SRST_PIN);
	gpio_set_output_options(SRST_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ,SRST_PIN);
	gpio_mode_setup(SRST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SRST_PIN);

	gpio_set(USB_CHARGE_PORT, USB_CHARGE_PIN);
	gpio_set_output_options(USB_CHARGE_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,USB_CHARGE_PIN);
	gpio_mode_setup(USB_CHARGE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, USB_CHARGE_PIN);

	gpio_set(USB_500_PORT, USB_500_PIN);
	gpio_set_output_options(USB_500_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_2MHZ,USB_500_PIN);
	gpio_mode_setup(USB_500_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, USB_500_PIN);

	gpio_set(EN_ESP32_PORT, EN_ESP32_PIN);
	gpio_set_output_options(EN_ESP32_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,EN_ESP32_PIN);
	gpio_mode_setup(EN_ESP32_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EN_ESP32_PIN);

	gpio_set(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN);
	gpio_set_output_options(GPIO0_ESP32_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_2MHZ,GPIO0_ESP32_PIN);
	gpio_mode_setup(GPIO0_ESP32_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0_ESP32_PIN);

	platform_timing_init();
	platform_delay(5000);
#ifndef PLATFORM_HAS_NO_SERIAL
	usbuart_init();
#endif
	usbcan_init();
	cdcacm_init();
	
	setup_vbus_detection();

	/* To correct the Pull-UP on D+ management
	Done in the cdcacm_init code by call of stm32f107_usbd_init :
	// Enable VBUS sensing in device mode and power down the PHY.
	OTG_FS_GCCFG |= OTG_FS_GCCFG_VBUSBSEN | OTG_FS_GCCFG_PWRDWN;
	But for the STM32F413, FS_GCCFG is not exactly the same as the F103 one.
	Then we correct the bad OTG_FS_GCCFG_VBUSBSEN bit
	*/
	OTG_FS_GCCFG &= ~OTG_GCCFG_VBUSBSEN;
	/* and use the right one */
	OTG_FS_GCCFG |= (1<<21);
	/* Disconnect/Reconnect the USB device with a delay in order to respect the bigger delay of 1,025 ms (see Table 217 of F413 Ref. Man.) */
	OTG_FS_DCTL |= OTG_DCTL_SDIS;
	platform_delay(2);
	OTG_FS_DCTL &= ~OTG_DCTL_SDIS;
}

void platform_switch_monitor_to(uint8_t choice){
	if(choice == 0){//case ESP
		//disable UART 407
		usart_disable(USBUSART_407);
		nvic_disable_irq(USBUSART_407_IRQ);

		//disable CAN ASEBA
		can_disable_irq(CAN_USED, CAN_IER_FMPIE0);

		canUsed = false;
		uartUsed = USBUSART_ESP;

		//enable UART ESP
		usart_enable(USBUSART_ESP);
		nvic_enable_irq(USBUSART_ESP_IRQ);

	}else if(choice == 1){//case MAIN
		//disable UART ESP
		usart_disable(USBUSART_ESP);
		nvic_disable_irq(USBUSART_ESP_IRQ);

		//disable CAN ASEBA
		can_disable_irq(CAN_USED, CAN_IER_FMPIE0);
	
		canUsed = false;
		uartUsed = USBUSART_407;

		//enable UART 407
		usart_enable(USBUSART_407);
		nvic_enable_irq(USBUSART_407_IRQ);

	}else if(choice == 2){//case ASEBA
		//disable UART 407
		usart_disable(USBUSART_407);
		nvic_disable_irq(USBUSART_407_IRQ);

		//disable UART ESP
		usart_disable(USBUSART_ESP);
		nvic_disable_irq(USBUSART_ESP_IRQ);

		canUsed = true;

		//enable CAN ASEBA
		can_enable_irq(CAN_USED, CAN_IER_FMPIE0);
	}
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
	return gpio_get(EN_ESP32_PORT, EN_ESP32_PIN) != 0;
}

void platform_set_gpio0_esp32(bool assert)
{
	if (assert)
		gpio_set(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN);
	else
		gpio_clear(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN);
}

bool platform_get_gpio0_esp32(void)
{
	return gpio_get(GPIO0_ESP32_PORT, GPIO0_ESP32_PIN) != 0;
}

void platform_pwr_on(bool on_state)
{
	if (on_state)
		gpio_set(PWR_ON_PORT, PWR_ON_PIN);
	else
		gpio_clear(PWR_ON_PORT, PWR_ON_PIN);
}

bool platform_pwr_on_btn_pressed(void)
{
	/* Return true if button pressed else false. */
	return gpio_get(PWR_ON_PORT, PWR_ON_BTN_PIN) == 0;
}

bool platform_vbus_hub(void)
{
	return gpio_get(VBUS_HUB_PORT, VBUS_HUB_PIN) != 0;
}

bool platform_get_vbus(void)
{
	return gpio_get(VBUS_PORT, VBUS_PIN) != 0;
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
	return gpio_get(USB_CHARGE_PORT, USB_CHARGE_PIN) != 0;
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
	return gpio_get(USB_500_PORT, USB_500_PIN) != 0;
}

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}
