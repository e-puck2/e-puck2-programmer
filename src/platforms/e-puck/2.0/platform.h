#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <ch.h>
#include <hal.h>

#include "gdb.h"
#include "communications.h"

/**
 * Blackmagic wrappers
 */

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


/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif /* __PLATFORM_H */