#include <stdio.h>
#include <string.h>

#include <platform.h>

#include <usb_hub.h>
#include <power_button.h>
#include <leds.h>

static THD_WORKING_AREA(test_thd_wa, 256);
static THD_FUNCTION(test_thd, arg)
{
	(void) arg;

	while(1){
		static systime_t time_before = 0;
		static systime_t time = 0;
		time_before = time;
		time = chVTGetSystemTime();
		chprintf((BaseSequentialStream *) &SDU2,"hello 2 %d\n",time-time_before);
		/* Sleep for some time. */
		chThdSleepMilliseconds(1);
	}
}




int main(void) {

	/**
	 * @brief 	Special function to handle the turn on if we pressed the button without
	 * 			the usb cable plugged. Called before everything to catch the button pressed.
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

	/*
	* Starts the handling of the power button
	*/
	powerButtonStart();

	/*
	*	Starts the PWM managing the leds
	*/
	ledInit();

	/*
	* Initializes two serial-over-USB CDC drivers and starts and connects the USB.
	*/
	usbSerialStart();

	/*
	* Starts the thread managing the USB hub
	*/
	usbHubStart();

	chThdCreateStatic(test_thd_wa, sizeof(test_thd_wa), NORMALPRIO, test_thd, NULL);

	while (true) {
		// static systime_t time_before = 0;
		// static systime_t time = 0;
		// time_before = time;
		// time = chVTGetSystemTime();

		// chprintf((BaseSequentialStream *) &SDU1,"hello 1 %d\n",time-time_before);
		chThdSleepMilliseconds(10);
		static int16_t value = 0;
		static int8_t coeff = 10;
		setLed(RED_LED, (value)/2);
		setLed(BLUE_LED, 1000-value);
		value+=coeff;
		if(value>1000 || value < 1){
			coeff *=-1;
			value+=2*coeff;
		}
	}
}
