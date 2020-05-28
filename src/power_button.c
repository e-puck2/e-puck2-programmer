/**
 * @file	power_button.c
 * @brief  	Controls the power button. Contains the functions to trun ON and OFF
 * 			the robot. Sends events when a power on or power off occurs.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	19.06.2018
 */

#include "main.h"
#include "power_button.h"

static virtual_timer_t power_timer;
static uint8_t power_state = POWER_OFF;

/////////////////////////////////////////PRIVATE FUNCTIONS/////////////////////////////////////////

/**
 * @brief 	Callback called when the virtual timer finishes to count.
 * 			Used to turn ON or OFF the robot when pressing the button for
 * 			the good duration
 * @param par Tells if we need to turn ON or OFF
 */
void powerButtonCb(void* par){
	uint8_t choice = (uint32_t)par;

	chSysLockFromISR();
	powerButtonTurnOnOffI(choice);
	chSysUnlockFromISR();
	
}

//Event source used to send events to other threads
event_source_t power_event;

static uint8_t power_on_reset = false;

static THD_WORKING_AREA(power_button_thd_wa, 128);
static THD_FUNCTION(power_button_thd, arg)
{

	(void) arg;
	chRegSetThreadName("Power Button management");

	/* Enabling events on both edges of the button line.*/
	palEnableLineEvent(LINE_PWR_ON_BTN, PAL_EVENT_MODE_BOTH_EDGES);

	if(power_on_reset){
		powerButtonTurnOnOff(POWER_ON);
	}

	while(1){
		//waiting until an event on the line is detected
		palWaitLineTimeout(LINE_PWR_ON_BTN, TIME_INFINITE);
		//if the button is pressed, we begin to count
		if(isPowerButtonPressed()){
			if(power_state == POWER_OFF){
				//set the timer to turn ON the robot after the good time
				chVTSet(&power_timer, TIME_MS2I(POWER_BUTTON_DURATION_MS_TO_TURN_ON),
                           powerButtonCb, (void*)POWER_ON);
			}else if(power_state == POWER_ON){
				//set the timer to turn OFF the robot after the good time
				chVTSet(&power_timer, TIME_MS2I(POWER_BUTTON_DURATION_MS_TO_TURN_OFF),
                           powerButtonCb, (void*)POWER_OFF);
			}
		}else{
			//stops the timer (prevent the callback to be called)
			chVTReset(&power_timer);
		}
	}
}

/////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void powerButtonStart(void){
	chVTObjectInit(&power_timer);

	chThdCreateStatic(power_button_thd_wa, sizeof(power_button_thd_wa), NORMALPRIO, power_button_thd, NULL);
}

void powerButtonStartSequence(void){

	/* 	we don't need to configure the GPIOs because it is done in _early_init which
		is called before the main */

	if(isPowerButtonPressed()){
		//turn on the robot if we pressed the button and there is no USB connexion
		//since we are before the chSysInit(), we simply store into a variable that we 
		//need to power on the robot. It will be applied in the thread
		power_on_reset = true;
	}
}

uint8_t isPowerButtonPressed(void){
	return !palReadLine(LINE_PWR_ON_BTN);
}

uint8_t powerButtonGetPowerState(void){
	return power_state;
}

void powerButtonTurnOnOff(uint8_t state){
	osalSysLock();
	powerButtonTurnOnOffI(state);
	osalSysUnlock();
}

void powerButtonTurnOnOffI(uint8_t state){
	if(state == POWER_ON){
		power_state = POWER_ON;
		palSetLine(LINE_PWR_ON_OUT);
		//setLedI(GREEN_LED, LED_MID_POWER);
		chEvtBroadcastFlagsI(&power_event, POWER_ON_FLAG);
	}else{
		power_state = POWER_OFF;
		palClearLine(LINE_PWR_ON_OUT);
		//setLedI(GREEN_LED, LED_NO_POWER);
		chEvtBroadcastFlagsI(&power_event, POWER_OFF_FLAG);
	}
}
