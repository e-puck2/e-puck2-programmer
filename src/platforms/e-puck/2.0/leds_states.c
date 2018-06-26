
#include "leds_states.h"
#include "power_button.h"

//values to control the Red and Green leds
static uint16_t leds_values[] = {LED_NO_POWER, LED_MID_POWER};

#define POWER_EVENT 		EVENT_MASK(0)
#define BATTERY_INFO_EVENT	EVENT_MASK(1)
#define GDB_STATUS_EVENT 	EVENT_MASK(2)

static THD_WORKING_AREA(leds_states_thd_wa, 256);
static THD_FUNCTION(leds_states_thd, arg)
{
	(void) arg;

	chRegSetThreadName("Leds states");


	eventmask_t events;
	eventflags_t flags;

	event_listener_t power_event_listener;
	//event_listener_t battery_info_event_listener;
	//event_listener_t gdb_status_event_listener;


	chEvtRegisterMask(&power_event, &power_event_listener, POWER_EVENT);
	//chEvtRegisterMask(&battery_info_event, &battery_info_event_listener, BATTERY_INFO_EVENT);
	//chEvtRegisterMask(&gdb_status_event, &gdb_status_event_listener, GDB_STATUS_EVENT);

	while (true) {
		
		events = chEvtWaitAnyTimeout(POWER_EVENT, TIME_IMMEDIATE);

		if (events & POWER_EVENT) {
			flags = chEvtGetAndClearFlags(&power_event_listener);
			if(flags & POWER_ON_FLAG){
				setLed(RED_LED, leds_values[RED_LED]);
				setLed(GREEN_LED, leds_values[GREEN_LED]);
			}else if(flags & POWER_OFF_FLAG){
				setLed(RED_LED,LED_NO_POWER); 
				setLed(GREEN_LED,LED_NO_POWER);
			}

		}else if(events & BATTERY_INFO_EVENT){
			//flags = chEvtGetAndClearFlags(&battery_info_event_listener);
		}

		chThdSleepMilliseconds(10);
	}
}

void ledsStatesStart(void){

	/*
	*	Starts the PWM managing the leds
	*/
	ledInit();

	/**
	* Starts the leds states thread
	*/
	chThdCreateStatic(leds_states_thd_wa, sizeof(leds_states_thd_wa), NORMALPRIO, leds_states_thd, NULL);
}

