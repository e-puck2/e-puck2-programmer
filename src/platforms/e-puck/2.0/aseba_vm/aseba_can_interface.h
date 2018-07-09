/**
 * @file    aseba_can_interface.h
 * @brief   USB Serial to CAN Aseba bridge functions
 * 
 * @source          adapted from aseba_can_interface.c of e-puck2_main-processor project
 * @written by      Eliot Ferragni
 * @creation date   28.06.2018
 */

#ifndef ASEBA_CAN_INTERFACE_H
#define ASEBA_CAN_INTERFACE_H

#include "vm.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void aseba_can_start(uint16 id);

void aseba_can_lock(void);
void aseba_can_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* ASEBA_CAN_INTERFACE_H */
