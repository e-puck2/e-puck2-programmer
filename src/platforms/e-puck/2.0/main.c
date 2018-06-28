#include <stdio.h>
#include <string.h>

#include "main.h"

#include <usb_hub.h>
#include <power_button.h>
#include <leds.h>
#include "gdb.h"
#include "uc_usage.h"
#include "leds_states.h"
#include "battery_measurement.h"
#include "aseba_can_interface.h"
#include "aseba_bridge.h"
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    230400,
	    0,
	    0,
	    0,
	};

	sdStart(&SD2, &ser_cfg); // UART3.
}

int main(void) {

	/**
	 * Special function to handle the turn on if we pressed the button without
	 * the usb cable plugged. Called before everything to catch the button pressed.
	 */
	powerButtonStartSequence();

	/*
	* System initializations.
	* - HAL initialization, this also initializes the configured device drivers
	*   and performs the board-specific initializations.
	* - Kernel initialization, the main() function becomes a thread and the
	*   RTOS is active.
	*/
	halInit();
	chSysInit();

	/**
	 * Init the events objects. Better to do it before any modules that use them since
	 * they are global.
	 */

	chEvtObjectInit(&power_event);
	chEvtObjectInit(&battery_info_event);
	chEvtObjectInit(&gdb_status_event);

	/**
	* Starts the leds states thread. Must be the first module because other modules
	* use it.
	*/
	ledsStatesStart();

	/*
	* Starts the handling of the power button
	*/
	powerButtonStart();

	batteryMesurementStart();

	serial_start();

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	/*
	* Starts the thread managing the USB hub
	*/
	usbHubStart();

	/*
	* Starts the GDB system
	*/
	gdbStart();

	aseba_can_start(0);
	aseba_bridge(&SDU2);

	while (true) {
		// static systime_t time_before = 0;
		// static systime_t time = 0;
		// time_before = time;
		// time = chVTGetSystemTime();

		// chprintf((BaseSequentialStream *) &SDU1,"hello 1 %d\n",time-time_before);
		chThdSleepMilliseconds(100);
		//printUcUsage((BaseSequentialStream *) &SDU2);
	}
}
