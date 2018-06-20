#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <ch.h>
#include <hal.h>

#include <usbcfg.h>
#include <shell.h>
#include <chprintf.h>

#define CONFIGURED		1
#define NOT_CONFIGURED	0

#define PWM_LED			PWMD3
#define I2C_SMBUS		I2CD1

#endif /* __PLATFORM_H */