#include "main.h"
#include "communications.h"
#include "aseba_can_interface.h"
#include "aseba_bridge.h"

static uint8_t uart_to_usb_should_pause	= false;
static BSEMAPHORE_DECL(uart_to_usb_pause, true);

static SerialDriver* uart_used = NULL;
static comm_modes_t active_mode = DEFAULT_COMM_MODE;

#define BUFFER_SIZE	64
static uint8_t rx_buffer[BUFFER_SIZE];

/**
 * @brief Resumes the aseba bridge threads
 */
void resumeUartToUSBThread(void){
	uart_to_usb_should_pause = false;
	chBSemSignal(&uart_to_usb_pause);
}

/**
 * @brief Pauses the aseba bridge threads
 */
void pauseUartToUSBThread(void){
	uart_to_usb_should_pause = true;
}

static THD_WORKING_AREA(uart_to_usb_thd_wa, 128);
static THD_FUNCTION(uart_to_usb_thd, arg)
{
	(void) arg;

	chRegSetThreadName("UART <-> USB");

	uint8_t nb_read = 0;

	while(1){
		if(uart_to_usb_should_pause){
			chBSemWait(&uart_to_usb_pause);
		}else{
			nb_read = chnReadTimeout((BaseChannel*)uart_used, rx_buffer, BUFFER_SIZE, TIME_MS2I(1));
			if(nb_read){
				chnWriteTimeout((BaseChannel*)&USB_SERIAL, rx_buffer, nb_read, TIME_MS2I(1));
			}

			nb_read = chnReadTimeout((BaseChannel*)&USB_SERIAL, rx_buffer, BUFFER_SIZE, TIME_MS2I(1));
			if(nb_read){
				chnWriteTimeout((BaseChannel*)uart_used, rx_buffer, nb_read, TIME_MS2I(1));
			}
		}
	}
}

void communicationsStart(void){

	static const SerialConfig ser_cfg_esp = {
	    .speed = 230400,
	    .cr1 = 0,
	    .cr2 = 0,
	    .cr3 = 0,
	};

	static const SerialConfig ser_cfg_407 = {
	    .speed = 115200,
	    .cr1 = 0,
	    .cr2 = 0,
	    .cr3 = 0,
	};

	/**
	 * Configures the two serial over uart drivers
	 */
	sdStart(&UART_ESP, &ser_cfg_esp);
	sdStart(&UART_407, &ser_cfg_407);

	/**
	 * Configures the can for Aseba and the threads of the Aseba Bridge
	 */
	aseba_can_start(0);
	aseba_bridge(&USB_SERIAL);

	/**
	 * Sets the communication mode to the default one
	 */
	communicationsSwitchModeTo(active_mode, false);

	/**
	 * Starts the communications thread
	 */
	chThdCreateStatic(uart_to_usb_thd_wa, sizeof(uart_to_usb_thd_wa), NORMALPRIO, uart_to_usb_thd, NULL);
}

void communicationsSwitchModeTo(comm_modes_t mode, uint8_t writeToflash){
	if(mode == ASEBA_CAN_TRANSLATOR){
		pauseUartToUSBThread();
		resumeAsebaBridge();
		active_mode = ASEBA_CAN_TRANSLATOR;
	}
	else{
		pauseAsebaBridge();
		if(mode == UART_407_PASSTHROUGH){
			uart_used = &UART_407;
			active_mode = UART_407_PASSTHROUGH;
		}else if(mode == UART_ESP_PASSTHROUGH){
			uart_used = &UART_ESP;
			active_mode = UART_ESP_PASSTHROUGH;
		}
		resumeUartToUSBThread();
	}
}

comm_modes_t communicationGetActiveMode(void){
	return active_mode;
}