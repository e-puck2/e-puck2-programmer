#ifndef SMBUS_H
#define SMBUS_H

/**
 * @file    SMBus.h
 * @brief   Functions to configure SMBus host over I2C
 * 
 * @author  Eliot Ferragni
 */



#include "general.h"


//////////////////// PROTOTYPES PUBLIC FUNCTIONS /////////////////////

/**
 * @brief [Init the SMBus interface with I2C1]
 */
void SMBus_init(void);
/**
 * @brief [SMBus write functions]
 * @details [Transmit data through SMBus protocole]
 * 
 * @param i2c_addr [I2C addresse in 8bit of the peripherical to talk to]
 * @param reg [Address of the register to access]
 * @param nbDatas [Number of datas to transmit in Byte]
 * @param datas [Pointer to the datas to transmit]
 */
void SMBus_write(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas);
/**
 * @brief [SMBus read functions]
 * @details [Receive datas through SMBus protocole]
 * 
 * @param i2c_addr [I2C addresse in 8bit of the peripherical to talk to]
 * @param reg [Address of the register to access]
 * @param nbDatas [Number of datas to receive in Byte]
 * @param datas [Pointer to save the datas]
 */
void SMBus_read(uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas);

#endif /* SMBUS_H*/