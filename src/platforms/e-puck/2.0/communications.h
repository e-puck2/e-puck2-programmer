#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

//functionning modes of the communications thread
//the USB side is always the USB_SERIAL comm port
typedef enum{
	UART_407_PASSTHROUGH = 0,
	UART_ESP_PASSTHROUGH,
	ASEBA_CAN_TRANSLATOR,
	NB_COMM_MODES,
}comm_modes_t;

#define DEFAULT_COMM_MODE			UART_ESP_PASSTHROUGH

//Event source used to send events to other threads
extern event_source_t communications_event;

//Possible flags of the communications event
#define ACTIVE_COMMUNICATION_FLAG		(1<<0)	
#define NO_COMMUNICATION_FLAG			(1<<1)

/**
 * @brief Starts the communications thread
 * 
 * @details Handles the uart407 <-> USB translator, the uartESP <-> USB 
 * 			and the Aseba CAN <-> USB translator
 */
void communicationsStart(void);

/**
 * @brief Switches the communication mode to the one given in parameter
 * @param mode 			Mode chosen
 * @param writeToflash 	Choose to write or not the mode in the flash
 */
void communicationsSwitchModeTo(comm_modes_t mode, uint8_t writeToflash);

/**
 * @brief Returns the active communication mode
 * @return The active communication mode. See comm_modes_t
 */
comm_modes_t communicationGetActiveMode(void);

#endif  /* COMMUNICATIONS_H */