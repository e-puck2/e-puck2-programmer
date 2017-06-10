#ifndef USB2513B_H
#define USB2513B_H

/**
 * @file    USB2513B.h
 * @brief   Functions to configure the USB2513B USB2 Hub over SMBus.
 * 
 * @author  Eliot Ferragni
 */



//////////////////// PROTOTYPES PUBLIC FUNCTIONS /////////////////////

/**
 * @brief [reset the registers to the default config]
 */
void USB2513B_reset(void);
/**
 * @brief [Set the configuration of the chip]
 */
void USB2513B_init(void);
//Tell the chip to attach to USB. Also write protect the registers
/**
 * @brief [Tell the chip to attach to USB. Also write protect the registers]
 */
void USB2513B_usb_attach(void);

//Tell the chip to attach to USB. Also write protect the registers
/**
 * @brief [Tell the chip to detach from USB. Also write protect the registers]
 */
void USB2513B_usb_detach(void);



#endif /* USB2513B_H*/