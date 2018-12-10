/**
 * @file    aseba_bridge.h
 * @brief   USB Serial to CAN Aseba bridge functions
 * 
 * @source          adapted from aseba_bridge.c of e-puck2_main-processor project
 * @written by      Eliot Ferragni
 * @creation date   28.06.2018
 */

#ifndef ASEBA_BRIDGE_H
#define ASEBA_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch.h"

void aseba_bridge(void *stream);

/**
 * @brief Resumes the aseba bridge threads
 */
void resumeAsebaBridge(void);

/**
 * @brief Pauses the aseba bridge threads
 */
void pauseAsebaBridge(void);
/** Returns true if the board is running in bridge mode. */
bool aseba_is_bridge(void);

#ifdef __cplusplus
}
#endif
#endif
