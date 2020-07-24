/**
 * @file	platform.h
 * @brief  	Used to make the link between the blackmagic files and the ChibiOS project.
 * 
 * @written by  		Eliot Ferragni
 * @creation date		15.02.2019
 * @last modification	27.02.2019
 */

#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <ch.h>
#include <hal.h>

#include "gdb.h"
#include "communications.h"
#include "power_button.h"

/**
 * Blackmagic wrappers
 */


#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"
#include "debug.h"

/* Uncomment to define a custom section area in the ram to place
   the working area of the GDB thread. For example ".ram0" 
   replace .to_be_defined by the section you want to use */

//#define BLACKMAGIC_CUSTOM_RAM_SECTION ".to_be_defined"

/* Uncomment to use a second interface to communicate with GDB
the behavior is to try to read or write to the second interface provided if the USB
isn't connected. It means the USB has always the priority*/

#define USE_SECOND_GDB_INTERFACE

/* 	Uncomment if you want to turn on the target when a swdp_scan is done.
	It is useless if you don't have a physical way to do it */

#define POWER_ON_WHEN_SWDP_SCAN

/*  Uncomment if you want to add custom commands to GDB. The commands are defined in the
	platform_commands.h */

//#define PLATFORM_HAS_CUSTOM_COMMANDS

/* USB related settings. Go in pair with usbcfg.c/.h */
#define GDB_USB_INTERFACE_NB	GDB_INTERFACE				// USB interface number (interface_name_t)
#define GDB_USB_INTERFACE		USB_GDB						// USB interface (SerialUSBDrivers)
#define GDB_DEVICE_NAME			USB_DEVICE_NAME				// Name of GDB device
#define GDB_VENDOR_NAME			USB_VENDOR_NAME				// Name of vendor 
#define GDB_USB_DATA_SIZE		USB_DATA_SIZE				// USB packet size
#define gdbIsUSBConfigured		isUSBConfigured				// Wrapper to the function to tell if usb is configured
#define gdbGetControlLineState	getControlLineState			// Wrapper to the functions to get control line state

#ifdef USE_SECOND_GDB_INTERFACE
//Replace with the Serial or USB Serial object you want to use (for example SD2 for UART2) 
#define GDB_2ND_INTERFACE	UART_ESP

/* the two functions below are to be filled in platform.c */	

/**
 * @brief 	Tell if the second interface usable for GDB is active
 * @details In the current implementation, it is possible to have a second interface such as UART
 * 			for example to communicate with GDB when the USB isn't connected. 
 * 			This function is used to know if we want to use the second interface or not.
 * @return true if connected, false otherwise
 */		
bool platform_is_second_gdb_interface_active(void);

/**
 * @brief 	Tell if the second interface usable for GDB is connected
 * @details In the current implementation, it is possible to have a second interface such as UART
 * 			for example to communicate with GDB when the USB isn't connected.
 * @return true if connected, false otherwise
 */		
bool platform_is_second_gdb_interface_connected(void);
#endif /* USE_SECOND_GDB_INTERFACE */

#ifdef POWER_ON_WHEN_SWDP_SCAN

/* 	the function below need to be filled to turn on the target.
	you ca safely add a chThdSleep() inside in order to be sure the target has booted */

/**
 * @brief Function to turn on the target
 */
void platform_turn_on_target_on_swdp_scan(void);
#endif /* POWER_ON_WHEN_SWDP_SCAN */


#define SWDIO_PORT 	GPIOA
#define SWCLK_PORT 	GPIOA
#define SWDIO_PIN	GPIOA_SWD_407_DIO
#define SWCLK_PIN	GPIOA_SWD_407_CLK

#define SRST_PORT	GPIOB
#define SRST_PIN	GPIOB_SWD_407_RESET


#define TMS_SET_MODE() {palSetLineMode(LINE_SWD_407_DIO, PAL_MODE_OUTPUT_PUSHPULL);}

#define SWDIO_MODE_FLOAT() {palSetLineMode(LINE_SWD_407_DIO, PAL_MODE_INPUT);}

#define SWDIO_MODE_DRIVE() {palSetLineMode(LINE_SWD_407_DIO, PAL_MODE_OUTPUT_PUSHPULL);}

#define SET_RUN_STATE(state)	{gdbSetFlag(state ? RUNNING_FLAG : IDLE_FLAG);};
#define SET_PROGRAMMING_STATE()	{gdbSetFlag(PROGRAMMING_FLAG);};
#define SET_IDLE_STATE(state)	{};
#define SET_ERROR_STATE(state)	{gdbSetFlag(ERROR_FLAG);};

static inline int platform_hwversion(void)
{
	return 0;
}

void platform_set_en_esp32(bool assert);
bool platform_get_en_esp32(void);

void platform_set_gpio0_esp32(bool assert);
bool platform_get_gpio0_esp32(void);

bool platform_vbus_hub(void);
bool platform_get_vbus(void);

void platform_set_usb_charge(bool assert);
bool platform_get_usb_charge(void);

void platform_set_usb_500(bool assert);
bool platform_get_usb_500(void);

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif /* __PLATFORM_H */