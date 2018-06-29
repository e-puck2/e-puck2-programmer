#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

//functionning modes of the communications thread
//the USB side is always the USB_SERIAL comm port
typedef enum{
	UART_407_PASSTHROUGH = 0,
	UART_ESP_PASSTHROUGH,
	ASEBA_CAN_TRANSLATOR
}comm_modes_t;


#define DEFAULT_COMM_MODE	UART_ESP_PASSTHROUGH


/**
 * @brief Starts the communications thread
 * 
 * @details Handles the uart407 <-> USB translator, the uartESP <-> USB 
 * 			and the Aseba CAN <-> USB translator
 */
void communicationsStart(void);

#endif  /* COMMUNICATIONS_H */