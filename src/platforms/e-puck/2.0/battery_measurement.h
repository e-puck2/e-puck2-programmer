/**
 * @file	battery_measurement.h
 * @brief  	Functions to measure the battery voltage and change the battery state accordingly
 * 			Uses the event system to signal a new state
 * 
 * @written by  	Eliot Ferragni
 * @creation date	26.06.2018
 */

#ifndef BATTERY_MEASUREMENT_H
#define BATTERY_MEASUREMENT_H

#include "main.h"


/* BATTERY---[ R1 ]--*--- measure
                     |
                     |
                   [ R2 ]
                     |
                    GND
*/

#define RESISTOR_R1             220 //kohm
#define RESISTOR_R2             330 //kohm

#define MAX_VOLTAGE				4.2f	//volt GREEN
#define GOOD_VOLTAGE			3.50f	//volt ORANGE
#define LOW_VOLTAGE				3.40f	//volt RED
#define VERY_LOW_VOLTAGE		3.30f	//volt RED BLINKING
#define MIN_VOLTAGE				3.20f	//volt RED BLINKING + QUICK TURNOFF

#define BATTERY_LOW_TIME_MS		10000 	//time before shutting down the system when in very low voltage
#define CHANGE_STATE_TIME_MS	3000	//time before changing the battery state 

//Event source used to send events to other threads
extern event_source_t battery_info_event;

#define MAX_VOLTAGE_FLAG		(1<<0)
#define GOOD_VOLTAGE_FLAG		(1<<1)
#define LOW_VOLTAGE_FLAG		(1<<2)
#define VERY_LOW_VOLTAGE_FLAG	(1<<3)
#define MIN_VOLTAGE_FLAG		(1<<4)

/**
 * @brief Starts the battery measurement thread
 */
void batteryMesurementStart(void);

#endif  /* BATTERY_MEASUREMENT_H */