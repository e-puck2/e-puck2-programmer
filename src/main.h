/**
 * @file	main.h
 * @brief  	Main file of the e-puck2_programmer firmware used by the onboard programmer of the
 * 			e-puck2 educational robot.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#ifndef MAIN_H
#define MAIN_H

#include <ch.h>
#include <hal.h>

#include <usbcfg.h>
#include <shell.h>
#include <chprintf.h>
#include "platform.h"

#define CONFIGURED		1
#define NOT_CONFIGURED	0

#define UART_ESP		SD2
#define UART_407		SD4
#define USB_GDB			SDU1
#define USB_SERIAL		SDU2

#define PWM_LED			PWMD3
#define I2C_SMBUS		I2CD1
#define ADC_BATT		ADCD1
#define CAN_ASEBA		CAND1

#endif  /* MAIN_H */