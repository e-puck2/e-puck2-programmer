
#include "leds_states.h"

typedef enum{
	ERROR_STATE = 0,
	RUNNING_STATE,
	PROGRAMMING_STATE,
	WAITING_STATE,
	POWER_ON_STATE,
	POWER_OFF_STATE,
}gdb_states_t;

typedef enum{
	LED_BLINKING = 0,
	LED_FADING_UP,
	LED_FADING_DOWN,
	LED_ON,
	LED_OFF,
}led_states_t;

static const uint8_t power_on_seq[] = {LED_FADING_UP, };

static THD_WORKING_AREA(leds_states_thd_wa, 256);
static THD_FUNCTION(leds_states_thd, arg)
{
	(void) arg;

	chRegSetThreadName("Leds states");

	uint8_t gdb_state = WAITING_STATE;
	uint8_t leds_state = LED_OFF;

	while(1){
		static int16_t value = 0;
		static int8_t coeff = 10;
		setLed(RED_LED, (value)/2);
		setLed(BLUE_LED, 1000-value);
		value+=coeff;
		if(value>1000 || value < 1){
			coeff *=-1;
			value+=2*coeff;
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

