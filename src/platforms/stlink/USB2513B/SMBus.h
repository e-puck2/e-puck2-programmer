#include "general.h"

#define I2C_SMBUS_HOST_MODE		0x000A



void SMBus_init(void);
void i2c_smbus_host_mode(uint32_t i2c);
void SMBus_write(uint32_t i2c, uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas);
void SMBus_read(uint32_t i2c, uint8_t i2c_addr, uint8_t reg, uint8_t nbDatas, uint8_t* datas);