#ifndef LEDS_STATES_H
#define LEDS_TATES_H

#include "main.h"
#include "leds.h"

#define BLINK_TIME					100	//used when the target is running or when the robot is in low power state
#define COMMUNICATION_BLINK_TIME	10	//used when a communication is active

/**
* Starts the leds states thread
* Manages the red green and blue leds.
*/
void ledsStatesStart(void);

#endif  /* LEDS_STATES_H */