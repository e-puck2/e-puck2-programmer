/**
 * @file	i2c_smbus.c
 * @brief  	Functions to communicate with the SMBus protocol with ChibiOS
 * 
 * @source			adapted from i2c_bus.c of e-puck2_main-processor project
 * @written by  	Eliot Ferragni
 * @creation date	18.06.2018
 */

#include "main.h"
#include "i2c_smbus.h"

static i2cflags_t errors = 0;
static systime_t timeout = TIME_MS2I(4); // 4 ms

static uint8_t txbuf[I2C_MAX_SEQUENTIAL_WRITE+2];
static uint8_t rxbuf[I2C_MAX_SEQUENTIAL_WRITE+1];

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void i2c_smbus_start(void) {

	if(I2C_SMBUS.state != I2C_STOP) {
		return;
	}

	/*
	 * I2C configuration structure for camera, IMU and distance sensor.
	 * Set it to 400kHz fast mode
	 */
	static const I2CConfig i2c_cfg1 = {
		.op_mode = OPMODE_SMBUS_HOST,
		.clock_speed = 100000,
		.duty_cycle = STD_DUTY_CYCLE
	};

	//simulate 16 clock pulses to unblock potential I2C periph blocked
	//take control of the pin
	palSetLineMode(LINE_I2C1_SCL , PAL_MODE_OUTPUT_OPENDRAIN );
	//16 clock pulses
	for(uint8_t i = 0 ; i < 32 ; i++){
		palToggleLine(LINE_I2C1_SCL);
		chThdSleepMilliseconds(1);
	}
	//make sure the output is high
	palSetLine(LINE_I2C1_SCL);
	//give the control of the pin to the I2C machine
	palSetLineMode(LINE_I2C1_SCL , PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
	i2cStart(&I2C_SMBUS, &i2c_cfg1);
}

void i2c_smbus_stop(void) {
	i2cStop(&I2C_SMBUS);
}

i2cflags_t get_last_i2c_error(void) {
	return errors;
}

int8_t write_reg_multi(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {

	if(len > I2C_MAX_SEQUENTIAL_WRITE){
		return I2C_ERROR;
	}

	txbuf[0] = reg;
	txbuf[1] = len;

	for(uint8_t i = 0 ; i < len ; i++){
		txbuf[i+2] = buf[i];
	}

	i2cAcquireBus(&I2C_SMBUS);
	if(I2C_SMBUS.state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(&I2C_SMBUS, addr>>1, txbuf, len+2, rxbuf, 0, timeout);
		if (status != MSG_OK){
			errors = i2cGetErrors(&I2C_SMBUS);
			if(I2C_SMBUS.state == I2C_LOCKED){
				i2c_smbus_stop();
				i2c_smbus_start();
			}
			i2cReleaseBus(&I2C_SMBUS);
			return status;
		}
	}
	i2cReleaseBus(&I2C_SMBUS);

	return MSG_OK;
}


int8_t read_reg_multi(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {

	if(len > I2C_MAX_SEQUENTIAL_WRITE){
		return I2C_ERROR;
	}

	txbuf[0] = reg;

	i2cAcquireBus(&I2C_SMBUS);
	if(I2C_SMBUS.state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(&I2C_SMBUS, addr>>1, txbuf, 1, rxbuf, len+1, timeout);
		if (status != MSG_OK){
			errors = i2cGetErrors(&I2C_SMBUS);
			if(I2C_SMBUS.state == I2C_LOCKED){
				i2c_smbus_stop();
				i2c_smbus_start();
			}
			i2cReleaseBus(&I2C_SMBUS);
			return status;
		}
	}

	//the first value is the number of bytes of the transaction. => we don't read it
	for(uint8_t i = 0 ; i < len ; i++){
		buf[i] = rxbuf[i+1];
	}

	i2cReleaseBus(&I2C_SMBUS);

	return MSG_OK;
}
