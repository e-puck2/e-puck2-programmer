/**
 * @file    USB251XB.c
 * @brief   Functions to configure the USB251XB USB2 Hub over SMBus.
 *
 * @author  Eliot Ferragni
 */


#include <../USB251XB/USB251XB.h>
#ifdef EPUCK2_CHIBIOS
	#include <hal.h>
	#include <i2c_smbus.h>
#else
	#include <../USB251XB/SMBus.h>
#endif

//comment to be in FS mode
#define HS_MODE_SELECTED	1


//////  I2C ADDRESS //////////
#define USB251XB_ADDR							0x58 //(8bit)

////// REGISTERS MAP /////////						 //(8bits)
#define VENDOR_ID_LSB_REG						0x00
#define VENDOR_ID_MSB_REG						0x01
#define PRODUCT_ID_LSB_REG						0x02
#define PRODUCT_ID_MSB_REG						0x03
#define DEVICE_ID_LSB_REG						0x04
#define DEVICE_ID_MSB_REG						0x05
#define CONF_DATA_BYTE_1_REG					0x06
#define CONF_DATA_BYTE_2_REG					0x07
#define CONF_DATA_BYTE_3_REG					0x08
#define NON_REM_DEVICES_REG						0x09
#define PORT_DISABLE_SELF_REG					0x0A
#define PORT_DISABLE_BUS_REG					0x0B
#define MAX_POWER_SELF_REG						0x0C
#define MAX_POWER_BUS_REG						0x0D
#define HUB_CONTROLLER_MAX_CURRENT_SELF_REG		0x0E
#define HUB_CONTROLLER_MAX_CURRENT_BUS_REG		0x0F
#define POWER_ON_TIME_REG						0x10
#define LANGUAGE_ID_HIGH_REG					0x11
#define LANGUAGE_ID_LOW_REG						0x12
#define MANUFACTURER_STRING_LENGTH_REG			0x13
#define PRODUCT_STRING_LENGTH_REG				0x14
#define SERIAL_STRING_LENGTH_REG				0x15
#define MANUFACTURER_STRING_REG 				0x16 //0x16 to 0x53
#define PRODUCT_STRING_REG 						0x54 //0x54 to 0x91
#define SERIAL_STRING_REG 						0x92 //0x92 to 0xCF
#define BATTERY_CHARGING_ENABLE_REG				0xD0
#define RSVD_1_REG								0xE0
#define RSVD_2_REG								0xF5
#define BOOST_UP_REG							0xF6
#define RSVD_3_REG								0xF7
#define BOOST_40_REG							0xF8
#define RSVD_4_REG								0xF9
#define PORT_SWAP_REG							0xFA
#define PORT_MAP_12_REG						0xFB
#define PORT_MAP_34_REG						0xFC
#define RSVD_5_REG								0xFD
#define RSVD_6_REG								0xFE
#define STATUS_COMMAND_REG				0xFF

///////	CONFIGURATION ///////////
#define VENDOR_ID_LSB						0x24
#define VENDOR_ID_MSB						0x04
#define PRODUCT_ID_LSB_BASE			0x12	// Base value for the USB2512
#define PRODUCT_ID_MSB					0x25
#define DEVICE_ID_LSB						0xB3
#define DEVICE_ID_MSB						0x0B

#if HS_MODE_SELECTED == 1
//(default = 0x9B) Self-powered, High_speed, MTT, EOP disable, individual sensing, individual switching
#define CONF_DATA_BYTE_1					0x1B //modified
#else
//(default = 0x9B) Self-powered, Full_speed, MTT, EOP disable, individual sensing, individual switching
#define CONF_DATA_BYTE_1					0x3B //modified
#endif

//no dynamic auto-switching, overcurrent delay = 8ms, not compound device
#define CONF_DATA_BYTE_2					0x20
//standard port mapping mode, string support disabled
#define CONF_DATA_BYTE_3					0x02
//No devices are Non-Removable
#define NON_REM_DEVICES						0x00
//all port are enabled in self powered operation
#define PORT_DISABLE_SELF					0x00
//all port are enabled in bus powered operation
#define PORT_DISABLE_BUS					0x00
//2mA max in self mode (2mA increment)
#define MAX_POWER_SELF						0x01
//(default = 0x32) 500mA max in bus mode (2mA increment)
#define MAX_POWER_BUS						0xFA //modified
//2mA max for only the hub in self mode(2mA increment)
#define HUB_CONTROLLER_MAX_CURRENT_SELF		0x01
//100mA max for only the hub in bus mode(2mA increment)
#define HUB_CONTROLLER_MAX_CURRENT_BUS		0x32
//100ms until a port has power
#define POWER_ON_TIME						0x32
#define LANGUAGE_ID_HIGH					0x00
#define LANGUAGE_ID_LOW						0x00
#define MANUFACTURER_STRING_LENGTH			0x00
#define PRODUCT_STRING_LENGTH				0x00
#define SERIAL_STRING_LENGTH				0x00
#define MANUFACTURER_STRING 				0x00
#define PRODUCT_STRING 						0x00
#define SERIAL_STRING 						0x00
//no battery charging support
#define BATTERY_CHARGING_ENABLE				0x00
#define RSVD_1								0x00
#define RSVD_2								0x00
//no boost
#define BOOST_UP							0x00
#define RSVD_3								0x00
//no boost
#define BOOST_40							0x00
#define RSVD_4								0x00
//no swap of D+/D-
#define PORT_SWAP							0x00
//no port map
#define PORT_MAP_12							0x00
//no port map
#define PORT_MAP_34							0x00
#define RSVD_5								0x00
#define RSVD_6								0x00
#define STATUS_COMMAND 						0x00

//////////// STATUS COMMAND ORDER ////////////

//Reset the registers to defaults settings
#define STATUS_COMMAND_RESET_BIT			(1<<1)
//Activate the USB interface and write protect the registers 0x00 to 0xFE
#define STATUS_COMMAND_USB_ATTACH_BIT		(1<<0)


//////////////////// PRIVATE FUNCTIONS ////////////////////////
/**
 * @brief 		Writes a register over I2C
 * 
 * @param addr	8bits address of the peripherical to write to
 * @param reg	8bits address of the register to write
 * @param buf	Pointer to a buffer containing the values to send
 * @param size	Length of the requested write
 * 
 * @return		The error code. msg_t format
 */
int8_t USB251XB_write(uint8_t addr, uint8_t reg, uint8_t size, uint8_t* buf){
#ifdef EPUCK2_CHIBIOS
	return write_reg_multi(addr, reg, buf, size);
#else
	SMBus_write(addr, reg, size, buf);
	return 0;
#endif
}
/**
 * @brief 		Reads registers over I2C
 * 
 * @param addr	8bits address of the peripherical to read from
 * @param reg	8bits address of the register to read
 * @param buf	Pointer to a buffer used to store the values read
 * @param size	Length of the requested read. The buf must be this size or greater
 * 
 * @return		The error code. msg_t format
 */
int8_t USB251XB_read(uint8_t addr, uint8_t reg, uint8_t size, uint8_t* buf){
#ifdef EPUCK2_CHIBIOS
	return read_reg_multi(addr, reg, buf, size);
#else
	SMBus_write(addr, reg, size, buf);
	return 0;
#endif
}


//////////////////// PUBLIC FUNCTIONS /////////////////////////

void USB251XB_reset(void){

	uint8_t temp_reg = 0;
	USB251XB_read(USB251XB_ADDR, STATUS_COMMAND_REG, 1, &temp_reg);
	temp_reg |= STATUS_COMMAND_RESET_BIT;
	USB251XB_write(USB251XB_ADDR, STATUS_COMMAND_REG, 1, &temp_reg);
}

void USB251XB_usb_attach(void){
	uint8_t temp_reg = 0;
	USB251XB_read(USB251XB_ADDR, STATUS_COMMAND_REG, 1, &temp_reg);
	temp_reg |= STATUS_COMMAND_USB_ATTACH_BIT;
	USB251XB_write(USB251XB_ADDR, STATUS_COMMAND_REG, 1, &temp_reg);
}

void USB251XB_usb_detach(void){
	uint8_t temp_reg = 0;
	USB251XB_read(USB251XB_ADDR, STATUS_COMMAND_REG, 1, &temp_reg);
	temp_reg &= ~STATUS_COMMAND_USB_ATTACH_BIT;
	USB251XB_write(USB251XB_ADDR, STATUS_COMMAND_REG, 1, &temp_reg);
}

void USB251XB_init(t_USB251XB choice_of_USB251XB){

	USB251XB_usb_detach();
	USB251XB_reset();

	uint8_t temp_reg[17] = {	VENDOR_ID_LSB,
								VENDOR_ID_MSB,
								PRODUCT_ID_LSB_BASE + choice_of_USB251XB,
								PRODUCT_ID_MSB,
								DEVICE_ID_LSB,
								DEVICE_ID_MSB,
								CONF_DATA_BYTE_1,
								CONF_DATA_BYTE_2,
								CONF_DATA_BYTE_3,
								NON_REM_DEVICES,
								PORT_DISABLE_SELF,
								PORT_DISABLE_BUS,
								MAX_POWER_SELF,
								MAX_POWER_BUS,
								HUB_CONTROLLER_MAX_CURRENT_SELF,
								HUB_CONTROLLER_MAX_CURRENT_BUS,
								POWER_ON_TIME};

	USB251XB_write(USB251XB_ADDR, VENDOR_ID_LSB_REG, 17, temp_reg);

	USB251XB_usb_attach();

}
