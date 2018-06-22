

#include <usb_hub.h>
#include <i2c_smbus.h>
#include <USB251XB.h>

static THD_WORKING_AREA(usb_hub_thd_wa, 128);
static THD_FUNCTION(usb_hub_thd, arg)
{
	(void) arg;

	chRegSetThreadName("USB Hub Management");

	bool hub_state = NOT_CONFIGURED;

	//Vbus detection init. Used to configure the USB Hub when we detect an USB cable

	/* Enabling events on both edges of the button line.*/
	palEnableLineEvent(LINE_VBUS_DET, PAL_EVENT_MODE_BOTH_EDGES);
	//init the first time the hub
	USB251XB_init(USB2512B);
	hub_state = CONFIGURED;

	while(1){
		//waiting until an event on the line is detected
		palWaitLineTimeout(LINE_VBUS_DET, TIME_INFINITE);
		//wait a few moments to be sure the interruption was not triggered
		//by a glitch and then test the GPIO
		//also wait for the USB HUB to be running
		chThdSleepMilliseconds(DEBOUNCE_TIME_VBUS_DET_MS);

		if(palReadLine(LINE_VBUS_DET)){
			if(hub_state == NOT_CONFIGURED){
				USB251XB_init(USB2512B);
				hub_state = CONFIGURED;
			}
		
		}else{
			hub_state = NOT_CONFIGURED;
		}
	}
}

void usbHubStart(void){
	//SMBUS init
	i2c_smbus_start();

	chThdCreateStatic(usb_hub_thd_wa, sizeof(usb_hub_thd_wa), NORMALPRIO, usb_hub_thd, NULL);
}