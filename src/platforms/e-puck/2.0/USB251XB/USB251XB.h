#ifndef USB251XB_H
#define USB251XB_H

/**
 * @file    USB251XB.h
 * @brief   Functions to configure the USB251XB USB2 Hub over SMBus.
 *
 * @author  Eliot Ferragni
 */

typedef enum {
  USB2512B,
  USB2513B,
  USB2514B
} t_USB251XB;

//////////////////// PROTOTYPES PUBLIC FUNCTIONS /////////////////////

/**
 * @brief [reset the registers to the default config]
 */
void USB251XB_reset(void);
/**
 * @brief [Set the configuration of the chip]
 */
void USB251XB_init(t_USB251XB choice_of_USB251XB);
//Tell the chip to attach to USB. Also write protect the registers
/**
 * @brief [Tell the chip to attach to USB. Also write protect the registers]
 */
void USB251XB_usb_attach(void);

//Tell the chip to attach to USB. Also write protect the registers
/**
 * @brief [Tell the chip to detach from USB. Also write protect the registers]
 */
void USB251XB_usb_detach(void);



#endif /* USB251XB_H*/
