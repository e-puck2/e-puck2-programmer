
#include "leds_states.h"
#include "power_button.h"
#include "battery_measurement.h"
#include "gdb.h"

//values used to control the Red, Green and Blue leds
static uint16_t leds_values[] = {LED_NO_POWER, LED_QUARTER_POWER, LED_MAX_POWER};

#define POWER_EVENT 			EVENT_MASK(0)
#define BATTERY_INFO_EVENT		EVENT_MASK(1)
#define GDB_STATUS_EVENT 		EVENT_MASK(2)
#define COMMUNICATIONS_EVENT 	EVENT_MASK(3)

static THD_WORKING_AREA(leds_states_thd_wa, 1024);
static THD_FUNCTION(leds_states_thd, arg)
{
	(void) arg;

	chRegSetThreadName("Leds states");

	uint8_t running_state = false;
	uint8_t communicating_state = false;
	uint8_t low_power_state = false;

	systime_t time_run = 0;
	systime_t time_communication = 0;

	eventmask_t events;
	eventflags_t flags;

	event_listener_t battery_info_event_listener;
	event_listener_t power_event_listener;
	event_listener_t gdb_status_event_listener;
	event_listener_t communications_event_listener;

	chEvtRegisterMask(&battery_info_event, &battery_info_event_listener, BATTERY_INFO_EVENT);
	chEvtRegisterMask(&power_event, &power_event_listener, POWER_EVENT);
	chEvtRegisterMask(&gdb_status_event, &gdb_status_event_listener, GDB_STATUS_EVENT);
	chEvtRegisterMask(&communications_event, &communications_event_listener, COMMUNICATIONS_EVENT);

	while (true) {
		

						//////////EVENTS/////////

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
			flags = chEvtGetAndClearFlags(&battery_info_event_listener);
			if(flags & MIN_VOLTAGE_FLAG){
				//red blinking
				low_power_state = true;
				leds_values[RED_LED] = LED_QUARTER_POWER;
				leds_values[GREEN_LED] = LED_NO_POWER;
			}else if(flags & VERY_LOW_VOLTAGE_FLAG){
				//red blinking
				low_power_state = true;
				leds_values[RED_LED] = LED_QUARTER_POWER;
				leds_values[GREEN_LED] = LED_NO_POWER;
			}else if(flags & LOW_VOLTAGE_FLAG){
				//red
				low_power_state = false;
				leds_values[RED_LED] = LED_QUARTER_POWER;
				leds_values[GREEN_LED] = LED_NO_POWER;
			}else if(flags & GOOD_VOLTAGE_FLAG){
				//orange
				low_power_state = false;
				leds_values[RED_LED] = LED_QUARTER_POWER;
				leds_values[GREEN_LED] = LED_QUARTER_POWER;
			}else if(flags & MAX_VOLTAGE_FLAG){
				//green
				low_power_state = false;
				leds_values[RED_LED] = LED_NO_POWER;
				leds_values[GREEN_LED] = LED_QUARTER_POWER;
			}

			//updates the leds if the robot is ON if it is not blinking
			//(except for the blinking of the programming which is particular)
			if(powerButtonGetPowerState() == POWER_ON && !low_power_state && !running_state){
				setLed(RED_LED, leds_values[RED_LED]);
				setLed(GREEN_LED, leds_values[GREEN_LED]);
			}
		}
		//gdb status events come from gdb_main
		else if(events & GDB_STATUS_EVENT){
			flags = chEvtGetAndClearFlags(&gdb_status_event_listener);
			//during IDLE, we simply have the leds on
			if(flags & IDLE_FLAG){
				running_state = false;
				setLed(RED_LED, leds_values[RED_LED]);
				setLed(GREEN_LED, leds_values[GREEN_LED]);
			}
			//during running state, we make the leds blink
			else if(flags & RUNNING_FLAG){
				running_state = true;
			}
			//during the programming state, we make the leds blink at each flashwrite
			else if(flags & PROGRAMMING_FLAG){
				//flag set when we are programming the target.
				//since it is called at every flash write order, we need to blink the leds here
				toggleLed(RED_LED, leds_values[RED_LED]);
				toggleLed(GREEN_LED, leds_values[GREEN_LED]);
			}
			//does nothing for now
			if(flags & ERROR_FLAG){
			}
		}
		//communications events come from communications and aseba_bridge
		else if(events & COMMUNICATIONS_EVENT){
			flags = chEvtGetAndClearFlags(&communications_event_listener);

			//a communications is active
			if(flags & ACTIVE_COMMUNICATION_FLAG){
				communicating_state = true;
			}
			//no more communications
			else if(flags & NO_COMMUNICATION_FLAG){
				communicating_state = false;
			}
		}

							///////RED and GREEN Leds////////

		//condition to blink the leds at the specified frequency when the target is in run mode
		//or when it is in low power state
		if((running_state || low_power_state) && (powerButtonGetPowerState() == POWER_ON)){
			if(time_run < chVTGetSystemTime()){
				time_run = chVTGetSystemTime() + TIME_MS2I(BLINK_TIME);
				toggleLed(RED_LED,leds_values[RED_LED]);
				toggleLed(GREEN_LED,leds_values[GREEN_LED]);
			}
		}

							////////BLUE Led ///////////

		//condition to blink the blue led at the specified frequency when a communication is active
		if(communicating_state && (powerButtonGetPowerState() == POWER_ON)){
			if(time_communication < chVTGetSystemTime()){
				time_communication = chVTGetSystemTime() + TIME_MS2I(COMMUNICATION_BLINK_TIME);
				toggleLed(BLUE_LED,leds_values[BLUE_LED]);
			}
		}
		//we turn off the blue led after COMMUNICATION_BLINK_TIME if no more active communication
		else{
			if(time_communication < chVTGetSystemTime()){
				setLed(BLUE_LED, LED_NO_POWER);
			}
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

