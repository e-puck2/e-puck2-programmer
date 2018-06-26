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

#define PWM_LED			PWMD3
#define I2C_SMBUS		I2CD1
#define ADC_BATT		ADCD1

#endif  /* MAIN_H */