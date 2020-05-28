/**
 * @file	usb_hub.h
 * @brief  	Functions to handle the configuration of the USB Hub when needed.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	19.06.2018
 */

#ifndef USB_HUB_H
#define USB_HUB_H

#include "main.h"

#define DEBOUNCE_TIME_VBUS_DET_MS	100

/**
 * @brief 	Starts the thread which configures the  
 * 			USB Hub each time a cable connection is detected.
 */	
void usbHubStart(void);

#endif  /* USB_HUB_H */