
#include "leds_states.h"
#include "power_button.h"
#include "gdb.h"

//values to control the Red and Green leds
static uint16_t leds_values[] = {LED_NO_POWER, LED_MID_POWER};

#define POWER_EVENT 		EVENT_MASK(0)
#define BATTERY_INFO_EVENT	EVENT_MASK(1)
#define GDB_STATUS_EVENT 	EVENT_MASK(2)

static THD_WORKING_AREA(leds_states_thd_wa, 1024);
static THD_FUNCTION(leds_states_thd, arg)
{
	(void) arg;

	chRegSetThreadName("Leds states");

	static uint8_t running_state = false;

	static uint8_t toggle_state = 0;

	static systime_t time = 0;

	eventmask_t events;
	eventflags_t flags;

	event_listener_t power_event_listener;
	//event_listener_t battery_info_event_listener;
	event_listener_t gdb_status_event_listener;


	chEvtRegisterMask(&power_event, &power_event_listener, POWER_EVENT);
	//chEvtRegisterMask(&battery_info_event, &battery_info_event_listener, BATTERY_INFO_EVENT);
	chEvtRegisterMask(&gdb_status_event, &gdb_status_event_listener, GDB_STATUS_EVENT);

	while (true) {
		
		events = chEvtWaitOneTimeout(ALL_EVENTS, TIME_IMMEDIATE);
		//power events come from the power_button module
		if (events & POWER_EVENT) {
			flags = chEvtGetAndClearFlags(&power_event_listener);
			//turns on the leds
			if(flags & POWER_ON_FLAG){
				setLed(RED_LED, leds_values[RED_LED]);
				setLed(GREEN_LED, leds_values[GREEN_LED]);
			}
			//turns off the leds
			else if(flags & POWER_OFF_FLAG){
				setLed(RED_LED, LED_NO_POWER); 
				setLed(GREEN_LED, LED_NO_POWER);
			}

		}else if(events & BATTERY_INFO_EVENT){
			//flags = chEvtGetAndClearFlags(&battery_info_event_listener);
		}
		//gdb status events come from gdb_main
		else if(events & GDB_STATUS_EVENT){
			flags = chEvtGetAndClearFlags(&gdb_status_event_listener);
			//during IDLE, we simply have the leds on
			if(flags & IDLE_FLAG){
				toggle_state = false;
				running_state = false;
				setLed(RED_LED, leds_values[RED_LED]);
				setLed(GREEN_LED, leds_values[GREEN_LED]);
			}
			//during running state, we make the leds blink
			else if(flags & RUNNING_FLAG){
				running_state = true;
				toggle_state = false;
			}
			//during the programming state, we make the leds blink at each flashwrite
			else if(flags & PROGRAMMING_FLAG){
				//flag set when we are programming the target.
				//since it is called at every flash write order, we need to blink the leds here
				setLed(RED_LED, toggle_state ? leds_values[RED_LED] : LED_NO_POWER);
				setLed(GREEN_LED, toggle_state ? leds_values[GREEN_LED] : LED_NO_POWER);
				toggle_state = !toggle_state;
			}
			//does nothing for now
			if(flags & ERROR_FLAG){
			}
		}

		//condition to blink the leds at the specified frequency when the target is in run mode
		if(running_state){
			if((time + BLINK_TIME_RUNNING_STATE) < chVTGetSystemTime()){
				time = chVTGetSystemTime();
				toggle_state = !toggle_state;
			}
			setLed(RED_LED, toggle_state ? leds_values[RED_LED] : LED_NO_POWER);
			setLed(GREEN_LED, toggle_state ? leds_values[GREEN_LED] : LED_NO_POWER);
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

