#ifndef POWER_BUTTON_H
#define POWER_BUTTON_H

#include "main.h"

#define POWER_ON				1
#define POWER_OFF				0

#define POWER_BUTTON_DURATION_MS_TO_TURN_ON		10
#define POWER_BUTTON_DURATION_MS_TO_TURN_OFF	500

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