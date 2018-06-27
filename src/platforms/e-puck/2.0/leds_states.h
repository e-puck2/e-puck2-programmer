#ifndef LEDS_STATES_H
#define LEDS_TATES_H

#include "main.h"
#include "leds.h"

#define BLINK_TIME		100	//used when the target is running or when the robot is in low power state

/**
* Starts the leds states thread
* Manages the red and green leds. The blue is left for the translator thread
*/
void ledsStatesStart(void);

#endif  /* LEDS_STATES_H */