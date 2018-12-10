/**
 * @file	platform.c
 * @brief  	Used to make the link between the blackmagic files and the e-puck2_programmer files.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "gdb_packet.h"

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void platform_srst_set_val(bool assert)
{
	if (assert) {
		gpio_clear(SRST_PORT, SRST_PIN);
		chThdSleepMilliseconds(1);
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
	if(communicationGetActiveMode() == UART_ESP_PASSTHROUGH){
		if (assert)
			gpio_set(GPIOC, GPIOC_ESP32_EN);
		else
			gpio_clear(GPIOC, GPIOC_ESP32_EN);
	}else{
		gdb_out("Must be in mode 2 to perform this action\n");
	}
}

bool platform_get_en_esp32(void)
{
	return gpio_get(GPIOC, GPIOC_ESP32_EN) != 0;
}

void platform_set_gpio0_esp32(bool assert)
{
	if(communicationGetActiveMode() == UART_ESP_PASSTHROUGH){
		if (assert)
			gpio_set(GPIOB, GPIOB_ESP_GPIO0);
		else
			gpio_clear(GPIOB, GPIOB_ESP_GPIO0);
	}else{
		gdb_out("Must be in mode 2 to perform this action\n");
	}
	
}

bool platform_get_gpio0_esp32(void)
{
	return gpio_get(GPIOB, GPIOB_ESP_GPIO0) != 0;
}

bool platform_vbus_hub(void)
{
	return gpio_get(GPIOA, GPIOA_USB_VBUS) != 0;
}

bool platform_get_vbus(void)
{
	return gpio_get(GPIOC, GPIOC_VBUS_DET) != 0;
}

void platform_set_usb_charge(bool assert)
{
	if (assert)
		gpio_set(GPIOB, GPIOB_CHRG_USB_CHARGE);
	else
		gpio_clear(GPIOB, GPIOB_CHRG_USB_CHARGE);
}

bool platform_get_usb_charge(void)
{
	return gpio_get(GPIOB, GPIOB_CHRG_USB_CHARGE) != 0;
}

void platform_set_usb_500(bool assert)
{
	if (assert)
		gpio_set(GPIOB, GPIOB_CHRG_USB_500);
	else
		gpio_clear(GPIOB, GPIOB_CHRG_USB_500);
}

bool platform_get_usb_500(void)
{
	return gpio_get(GPIOB, GPIOB_CHRG_USB_500) != 0;
}

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}
