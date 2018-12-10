/**
 * @file	power_button.h
 * @brief  	Controls the power button. Contains the functions to trun ON and OFF
 * 			the robot. Sends events when a power on or power off occurs.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	19.06.2018
 */

#ifndef POWER_BUTTON_H
#define POWER_BUTTON_H

#include "main.h"

#define POWER_ON				1
#define POWER_OFF				0

#define POWER_BUTTON_DURATION_MS_TO_TURN_ON		10
#define POWER_BUTTON_DURATION_MS_TO_TURN_OFF	500

//Event source used to send events to other threads
extern event_source_t power_event;

//Possible flags of the power event
#define POWER_ON_FLAG	(1<<0)
#define POWER_OFF_FLAG	(1<<1)

/**
 * @brief 	Function used to turn ON the robot if the button is pressed
 * 			and if the USB connexion is not plugged->when we want to turn on the robot, without the USB.
 * 			
 * 			Need to be called before everything in the main (even halInit) in order to catch 
 * 			correctly the button pressed and not miss it because the system init took to much time.
 */	
void powerButtonStartSequence(void);

/**
 * @brief 	Starts the thread managing the button to turn on and off
 * 			the robot.
 */	
void powerButtonStart(void);

/**
 * @brief Checks if the power button is pressed
 * 
 * @return true of false
 */
uint8_t isPowerButtonPressed(void);

/**
 * @brief Returns the power state of the robot
 * 
 * @return POWER_ON or POWER_OFF
 */
uint8_t powerButtonGetPowerState(void);

/**
 * @brief Turns ON or OFF the robot
 * @param state POWER_ON or POWER_OFF
 */
void powerButtonTurnOnOff(uint8_t state);

/**
 * @brief Turns ON or OFF the robot. To be called from an interrupt context
 * @param state POWER_ON or POWER_OFF
 */
void powerButtonTurnOnOffI(uint8_t state);

#endif  /* POWER_BUTTON_H */