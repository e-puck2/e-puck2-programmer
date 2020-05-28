/**
 * @file	main.c
 * @brief  	Main file of the e-puck2_programmer firmware used by the onboard programmer of the
 * 			e-puck2 educational robot.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#include <stdio.h>
#include <string.h>

#include "main.h"

#include <usb_hub.h>
#include <power_button.h>
#include "gdb.h"
#include "uc_usage.h"
#include "leds_states.h"
#include "battery_measurement.h"
#include "communications.h"

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
	chEvtObjectInit(&communications_event);

	/**
	* Starts the leds states thread. Must be the first module because other modules
	* use it.
	*/
	ledsStatesStart();

	/*
	* Starts the handling of the power button
	*/
	powerButtonStart();

	/**
	 * Starts the battery measurement thread
	 */
	batteryMesurementStart();

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	/*
	* Starts the thread managing the USB hub
	*/
	usbHubStart();

	/**
	 * Starts the communication thread
	 */
	communicationsStart();

	/*
	* Starts the GDB system
	*/
	gdbStart();

	while (true) {
		chThdSleepMilliseconds(300);
		//printUcUsage((BaseSequentialStream *) &UART_ESP);
	}
}
