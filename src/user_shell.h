/**
 * @file	user_shell.h
 * @brief  	File containing functions and definitions to handle the shell provided by ChibiOS
 * 
 * @written by  	Eliot Ferragni
 * @creation date	12.03.2019
 */

#ifndef USER_SHELL_H
#define USER_SHELL_H

#include "usbcfg.h"
#define SHELL_PORT		USB_SERIAL


/**
 * @brief Spawns the shell
 */
void spawn_shell(void);

#endif /* USER_SHELL_H */