/**
 * @file	i2c_smbus.h
 * @brief  	Functions to communicate with the SMBus protocol with ChibiOS
 * 
 * @source			adapted from i2c_bus.c of e-puck2_main-processor project
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

#define I2C_MAX_SEQUENTIAL_WRITE	20	//bytes

#define I2C_ERROR 					1
/**
 * @brief Starts the I2C interface
 */
void i2c_smbus_start(void);

/**
 * @brief Stops the I2C module	
 */
void i2c_smbus_stop(void);

/**
 * @brief 	Gets the last I2C error
 * 
 * @return	The last error
 */
i2cflags_t get_last_i2c_error(void);

/**
 * @brief 		Reads a register bigger than 8bits over I2C
 * 
 * @param addr	8bits address of the peripherical to write to
 * @param reg	8bits address of the register to read
 * @param buf	Pointer to a buffer containing the values to send
 * @param len	Length of the requested write
 * 
 * @return		The error code. msg_t format
 */
int8_t write_reg_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief 		Reads a register bigger than 8bits over I2C
 * 
 * @param addr	8bits address of the peripherical to read from
 * @param reg	8bits address of the register to read
 * @param buf	Pointer to a buffer used to store the values read
 * @param len	Length of the requested read. the buf must be this size or greater
 * 
 * @return		The error code. msg_t format
 */
int8_t read_reg_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
