/**
 * @file	platform.h
 * @brief  	Used to make the link between the blackmagic files and the e-puck2_programmer files.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
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

#ifndef PLATFORM_HAS_NO_DFU_BOOTLOADER
#error "NO_DFU option must be defined for this platform !!"
#endif

#ifndef PLATFORM_HAS_NO_JTAG
#error "NO_JTAG option must be defined for this platform !!"
#endif

#if defined(PLATFORM_HAS_COMMANDS)
#define COMMANDS_OPTION "C"
#else
#define COMMANDS_OPTION "c"
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

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"
#include "version.h"

#define NOT_USED	0
#define JTAG_PORT 	GPIOA
#define TDI_PORT	NOT_USED
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	NOT_USED
#define TDI_PIN		NOT_USED
#define TMS_PIN		GPIOA_SWD_407_DIO
#define TCK_PIN		GPIOA_SWD_407_CLK
#define TDO_PIN		NOT_USED

#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	NOT_USED
#define TRST_PIN	NOT_USED
#define SRST_PORT	GPIOB
#define SRST_PIN	GPIOB_SWD_407_RESET

#define LED_PORT_ERROR		GPIOB //RED_LED
#define LED_PORT_UART		GPIOB //BLUE_LED
#define LED_PORT			GPIOA //GREEN_LED


#define LED_ERROR		GPIOB_LED_RED	
#define LED_UART		GPIOB_LED_BLUE	
#define LED_IDLE_RUN	GPIOA_LED_GREEN	
#define LED_BOOTLOADER	NOT_USED

#define TMS_SET_MODE() {palSetLineMode(LINE_SWD_407_DIO, PAL_MODE_OUTPUT_PUSHPULL);}

#define SWDIO_MODE_FLOAT() {palSetLineMode(LINE_SWD_407_DIO, PAL_MODE_INPUT);}

#define SWDIO_MODE_DRIVE() {palSetLineMode(LINE_SWD_407_DIO, PAL_MODE_OUTPUT_PUSHPULL);}

#define DEBUG(...)

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