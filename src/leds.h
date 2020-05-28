/**
 * @file	leds.h
 * @brief  	Functions to manage the RGB led connected to the programmer. Not inteded to be used by the user
 * 			because the RGB led is already completely used by leds_states.c
 * 
 * @written by  	Eliot Ferragni
 * @creation date	19.06.2018
 */

#ifndef LEDS_H
#define LEDS_H

#include "main.h"

//List of the LEDs present on the RGB LED
typedef enum {
	RED_LED = 0,
	GREEN_LED,
	BLUE_LED,
	NB_LEDS,
} led_name_t;


#define LED_MAX_POWER		1000
#define LED_MID_POWER		500
#define LED_QUARTER_POWER	250
#define LED_MIN_POWER		1
#define LED_NO_POWER		0

/**
 * @brief Init the PWM to handle the three leds
 */
void ledInit(void);

/**
 * @brief Toggles the selected led with the value given.
 * 
 * @param led 		Led to update. See led_name_t
 * @param value 	New value to give in the case the led was off. 
 * 					;If the led was on, it will simply turn it off.
 */
void toggleLed(led_name_t led, uint16_t value);

/**
 * @brief 		Sets the valu of the given led
 * 
 * @param led 	Led to update. See led_name_t
 * @param value New value to give to the led. 
 * 				Range is 0-1000 (0 = completely off and 1000 = completely on)
 */
void setLed(led_name_t led, uint16_t value);

/**
 * @brief 		Sets the valu of the given led. To be called from an interrupt context
 * 
 * @param led 	Led to update. See led_name_t
 * @param value New value to give to the led. 
 * 				Range is 0-1000 (0 = completely off and 1000 = completely on)
 */
void setLedI(led_name_t led, uint16_t value);

#endif  /* LEDS_H */