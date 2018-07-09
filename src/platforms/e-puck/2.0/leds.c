/**
 * @file	leds.c
 * @brief  	Functions to manage the RGB led connected to the programmer
 * 
 * @written by  	Eliot Ferragni
 * @creation date	19.06.2018
 */

#include "main.h"
#include "leds.h"

#define PWM_CLOCK_FREQUENCY		100000		//100kHz
#define PWM_PERIOD 				1000		//=> resolution = 1000 and 
											//   PWM frequency = PWM_CLOCK_FREQUENCY/PWM_PERIOD = 100Hz

static uint8_t pwm_status = NOT_CONFIGURED;

static uint16_t leds_values[NB_LEDS] = {0};

/////////////////////////////////////////PRIVATE FUNCTIONS/////////////////////////////////////////

/**
 * @brief Turns OFF the three leds. Called each time the PWM resets its counter
 * 
 * @param pwmp PWM driver (not used)
 */
static void clearLedsCb(PWMDriver *pwmp) {

	(void)pwmp;
	palSetLine(LINE_LED_RED);
	palSetLine(LINE_LED_GREEN);
	palSetLine(LINE_LED_BLUE);

}

/**
 * @brief 	Turns ON the red led. Called when the counter of the PWM becomes 
 * 			higher than the period of the channel.
 * 
 * @param pwmp PWM driver (not used)
 */
static void redLedCb(PWMDriver *pwmp) {

	(void)pwmp;
	palClearLine(LINE_LED_RED);
}

/**
 * @brief 	Turns ON the green led. Called when the counter of the PWM becomes 
 * 			higher than the period of the channel.
 * 
 * @param pwmp PWM driver (not used)
 */
static void greenLedCb(PWMDriver *pwmp) {

	(void)pwmp;
	palClearLine(LINE_LED_GREEN);
}

/**
 * @brief 	Turns ON the blue led. Called when the counter of the PWM becomes 
 * 			higher than the period of the channel.
 * 
 * @param pwmp PWM driver (not used)
 */
static void blueLedCb(PWMDriver *pwmp) {

	(void)pwmp;
	palClearLine(LINE_LED_BLUE);
}

static PWMConfig pwmLedCfg = {
	PWM_CLOCK_FREQUENCY,
	PWM_PERIOD,
	clearLedsCb,
	{
		{PWM_OUTPUT_DISABLED, redLedCb},
		{PWM_OUTPUT_DISABLED, greenLedCb},
		{PWM_OUTPUT_DISABLED, blueLedCb},
		{PWM_OUTPUT_DISABLED, NULL}
	},
	0,
	0
};

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void ledInit(void){
	pwmStart(&PWM_LED, &pwmLedCfg);
	pwmEnablePeriodicNotification(&PWM_LED);
	
	//enables and disables otherwise we have a kernel panic when trying to disable
	//if it has never been enabled. This occurs if for example the first value we set with setLed() 
	//is 0.
	pwmEnableChannel(&PWM_LED, RED_LED, PWM_PERIOD);
	pwmEnableChannelNotification(&PWM_LED, RED_LED);
	pwmDisableChannelNotification(&PWM_LED, RED_LED);

	pwmEnableChannel(&PWM_LED, GREEN_LED, PWM_PERIOD);
	pwmEnableChannelNotification(&PWM_LED, GREEN_LED);
	pwmDisableChannelNotification(&PWM_LED, GREEN_LED);

	pwmEnableChannel(&PWM_LED, BLUE_LED, PWM_PERIOD);
	pwmEnableChannelNotification(&PWM_LED, BLUE_LED);
	pwmDisableChannelNotification(&PWM_LED, BLUE_LED);

	pwm_status = CONFIGURED;

	//sets the init values. 
	//For example if someone tried to set a led before the pwm was configured
	setLed(RED_LED, leds_values[RED_LED]);
	setLed(GREEN_LED, leds_values[GREEN_LED]);
	setLed(BLUE_LED, leds_values[BLUE_LED]);
}

void toggleLed(led_name_t led, uint16_t value){
	if(led>=NB_LEDS){
		return;
	}

	if(leds_values[led] != PWM_PERIOD){
		setLed(led, LED_NO_POWER);
	}else{
		setLed(led, value);
	}
}

void setLed(led_name_t led, uint16_t value){
	osalSysLock();
	setLedI(led, value);
	osalSysUnlock();
}

void setLedI(led_name_t led, uint16_t value){
	if(led>=NB_LEDS){
		return;
	}
	if(pwm_status == CONFIGURED){

		if(value>PWM_PERIOD){
			value = PWM_PERIOD;
		}

		//final duty-cycle is (100% - value)
		//because the callback of each led turns it ON.
		value = PWM_PERIOD - value;

		if(value < 1){
			value = 1;
		}

		if(value < PWM_PERIOD){
			pwmEnableChannelI(&PWM_LED, led, value);
			pwmEnableChannelNotificationI(&PWM_LED, led);
		}else{
			pwmDisableChannelNotificationI(&PWM_LED, led);

		}
		
	}

	//stores the value
	leds_values[led] = value;
}