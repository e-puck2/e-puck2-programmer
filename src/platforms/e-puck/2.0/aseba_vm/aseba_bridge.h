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
