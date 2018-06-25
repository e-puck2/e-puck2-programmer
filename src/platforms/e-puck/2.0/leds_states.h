#ifndef LEDS_STATES_H
#define LEDS_TATES_H

#include "main.h"
#include "leds.h"

/**
* Starts the leds states thread
* Manages the red and green leds. The blue is left for the translator thread
*/
void ledsStatesStart(void);

#endif  /* LEDS_STATES_H */