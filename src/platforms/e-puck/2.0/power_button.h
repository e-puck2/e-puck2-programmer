#ifndef POWER_BUTTON_H
#define POWER_BUTTON_H

#include <platform.h>

#define POWER_ON				1
#define POWER_OFF				0

#define POWER_BUTTON_DURATION_MS_TO_TURN_ON		10
#define POWER_BUTTON_DURATION_MS_TO_TURN_OFF	500

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

#endif  /* POWER_BUTTON_H */