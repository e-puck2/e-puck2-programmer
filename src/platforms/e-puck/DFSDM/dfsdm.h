/** @file dfsdm.h
 * DFSDM microphone driver
 *
 * This module implements a simple 2-channels DFSDM microphone driver.
 *
 * The driver operates using DMA in circular mode, which means that while one
 * buffer is processed by the user's code, the other half is used by the DMA to
 * store the samples. This allows real time processing of the samples without
 * data loss. It is similar to what is done for the ADCs drivers by ChibiOS.
 *
 * This driver is tailored to the hardware present on the STM32F7 discovery
 * board, but the implementation should be relatively easy to adapt to another
 * target / use case.
 */
#ifndef DFSDM_H
#define DFSDM_H

#include <stdint.h>
#include <unistd.h>

/** Callback type for data received
 *
 * @param [in] drv Pointer provided by the user in the DFSDM config.
 * @param [in] buffer A pointer to the buffer holding the samples.
 * @param [in] n number of samples in the buffer.
 */
typedef void (*dfsdmcallback_t)(void *drv, int32_t *buffer, size_t n);

/** Callback type for DMA errors.
 *
 * @param [in] drv Pointer provided by the user in the DFSDM config.
 */
typedef void (*dfsdmerrorcallback_t)(void *drv);

/** Configuration for one of the two DFSDM microophone channels. */
typedef struct {
    /** Callback called when samples_len/2 samples have been read. */
    dfsdmcallback_t end_cb;

    /** Callback in case of DMA error. */
    dfsdmerrorcallback_t error_cb;

    /** Argument passed to the callbacks. */
    void *cb_arg;

    /** Pointer to a buffer to hold the samples. */
    int32_t *samples;

    /** Length of the samples buffer, in number of elements. */
    size_t samples_len;
} DFSDM_config_t;

/** Configure the hardware peripherals. */
void dfsdm_start(void);

/** Starts the continous acquisition. */
void dfsdm_start_conversion(DFSDM_config_t *left_config, DFSDM_config_t *right_config);

/** Stops the continous acquisition. */
void dfsdm_stop_conversion(void);

#endif
