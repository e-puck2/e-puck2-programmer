/**
 * @file	communications.c
 * @brief  	Functions to manage the three different communications modes
 * 			available using the second USB virtual com port (Serial Monitor)
 * 			Sends events to signal the state of the communications
 * 
 * @written by  	Eliot Ferragni
 * @creation date	29.06.2018
 */

#include "main.h"
#include "communications.h"
#include "aseba_can_interface.h"
#include "aseba_bridge.h"
#include "flash_common_f24.h"

#define CONFIG_SECTOR	15			//corresponds to the last sector of the flash for the 413
#define PATTERN_FLASH	0xABCDEC	//arbitrary patern to detect if the block has already been written

//flash variables. used in writeModeToFlash() and findLastModeWrittenToFlash()
extern uint32_t _config_start; //sector 15	//defined in the .ld file
extern uint32_t _config_end;				//defined in the .ld file

static uint32_t config_start = (uint32_t)&_config_start;
static uint32_t config_end = (uint32_t)&_config_end;
static uint32_t config_addr;

//Event source used to send events to other threads
event_source_t communications_event;

//used to pause or not the thread
static uint8_t uart_usb_should_pause	= false;
static BSEMAPHORE_DECL(uart_to_usb_pause, true);
static BSEMAPHORE_DECL(usb_to_uart_pause, true);

//used to store the active mode
static SerialDriver* uart_used = NULL;
static comm_modes_t active_mode = DEFAULT_COMM_MODE;

/////////////////////////////////////////PRIVATE FUNCTIONS/////////////////////////////////////////

/**
 * @brief Resumes the aseba bridge threads
 */
void resumeUartToUSBThreads(void){
	uart_usb_should_pause = false;
	chBSemSignal(&uart_to_usb_pause);
	chBSemSignal(&usb_to_uart_pause);
}

/**
 * @brief Pauses the aseba bridge threads
 */
void pauseUartToUSBThreads(void){
	uart_usb_should_pause = true;
}

/**
 * @brief 	Writes to the flash the mode given in parameter.
 * 			Also erases the flash if it is full or if we found nothing on it
 * 
 * @param choice. See comm_modes_t
 */
void writeModeToFlash(comm_modes_t choice){
	flash_unlock();

	//erases the flash if we are at the end or if we found nothing on it
	if( (config_addr == 0) || (config_addr >= (config_end - sizeof(uint32_t)) ) ){
		flash_erase_sector(CONFIG_SECTOR, FLASH_CR_PROGRAM_X32);
		config_addr = config_start;
	}

	//writes the choice and the pattern on the flash 
	flash_program_word(config_addr, PATTERN_FLASH | (uint32_t)choice);

	//increment for the next write
	config_addr += sizeof(uint32_t);

	flash_lock();
}

/**
 * @brief Search in the flash the last mode written
 * @return 	The last mode written in the flash. If no mode has been found, or if the flash is
 * 			full, config_addr is set to 0.
 */
comm_modes_t findLastModeWrittenToFlash(void){
	uint32_t* block = (uint32_t*)config_start;
	uint32_t* last = NULL;
	uint32_t last_value = 0;
	comm_modes_t choice = DEFAULT_COMM_MODE; //if nothing found on the flash, then the default choice is used

	//the pointer will be inceremented by sizeof(uint32_t) so we need to divide the length by the same value
	uint32_t length = (config_end - config_start)/sizeof(uint32_t);
	
	//checks the flash to find variables with the pattern.
	//continues until it founds the last pattern written or if at the end of the flash sector.
	for(uint32_t i = 0 ; i < length ; i++){
		if((block[i] & 0xFFFFFFFC) == PATTERN_FLASH){
			last = &block[i];
			last_value = block[i];	
		}else{
			break;
		}
	}

	if(last == NULL){
		config_addr = 0;
	}else{
		//we take only the 2 last bits => only 4 choices availables with this procedure (0,1,2,3)
		choice = last_value & 0x03; //if we do this in the loop, it goes into an unhandled exception...
		//chprintf((BaseSequentialStream *) &SDU2,"choice = %d\n", choice);
		//sets the config_addr to the next writtable address
		config_addr = (uint32_t)last + sizeof(uint32_t);
	}

	return choice;
}

static THD_WORKING_AREA(uart_to_usb_thd_wa, 128);
static THD_FUNCTION(uart_to_usb_thd, arg)
{
	(void) arg;

	chRegSetThreadName("UART -> USB");

	uint8_t c[1] = {0};
	uint8_t nb_read = 0;
	uint32_t nb_times_read = 0;

	while(1){
		if(uart_usb_should_pause){
			chBSemWait(&uart_to_usb_pause);
		}else{
			nb_read = chnReadTimeout((BaseChannel*)uart_used, c, 1, TIME_MS2I(10));
			if(nb_read){
				nb_times_read++;
				if(nb_times_read > 1)
					chEvtBroadcastFlags(&communications_event, ACTIVE_COMMUNICATION_FLAG);
				
				if((communicationGetActiveMode() == UART_407_PASSTHROUGH) && getControlLineState(SERIAL_INTERFACE, CONTROL_LINE_DTR))
					chnWriteTimeout((BaseChannel*)&USB_SERIAL, c, 1, TIME_INFINITE);
				else if(communicationGetActiveMode() == UART_ESP_PASSTHROUGH)
					chnWriteTimeout((BaseChannel*)&USB_SERIAL, c, 1, TIME_INFINITE);
			}else{
				nb_times_read = 0;
				chEvtBroadcastFlags(&communications_event, NO_COMMUNICATION_FLAG);
			}
		}
	}
}

static THD_WORKING_AREA(usb_to_uart_thd_wa, 128);
static THD_FUNCTION(usb_to_uart_thd, arg)
{
	(void) arg;

	chRegSetThreadName("USB -> UART");

	uint8_t c[1] = {0};
	uint8_t nb_read = 0;

	while(1){
		if(uart_usb_should_pause){
			chBSemWait(&usb_to_uart_pause);
		}else{
			nb_read = chnReadTimeout((BaseChannel*)&USB_SERIAL, c, 1, TIME_MS2I(10));
			if(nb_read){
				chEvtBroadcastFlags(&communications_event, ACTIVE_COMMUNICATION_FLAG);
				chnWriteTimeout((BaseChannel*)uart_used, c, 1, TIME_INFINITE);
			}else{
				chEvtBroadcastFlags(&communications_event, NO_COMMUNICATION_FLAG);
			}
		}
	}
}

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

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
	active_mode = findLastModeWrittenToFlash();
	communicationsSwitchModeTo(active_mode, false);

	/**
	 * Starts the communications threads
	 */
	chThdCreateStatic(uart_to_usb_thd_wa, sizeof(uart_to_usb_thd_wa), NORMALPRIO, uart_to_usb_thd, NULL);
	chThdCreateStatic(usb_to_uart_thd_wa, sizeof(usb_to_uart_thd_wa), NORMALPRIO, usb_to_uart_thd, NULL);
}

void communicationsSwitchModeTo(comm_modes_t mode, uint8_t writeToflash){
	if(mode >= NB_COMM_MODES){
		mode = DEFAULT_COMM_MODE;
		active_mode = DEFAULT_COMM_MODE;
	}
	if(mode == ASEBA_CAN_TRANSLATOR){
		pauseUartToUSBThreads();
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
		resumeUartToUSBThreads();
	}

	if(writeToflash){
		writeModeToFlash(mode);
	}

}

comm_modes_t communicationGetActiveMode(void){
	return active_mode;
}

//we get the gpio0 pin status of the ESP32. 0 is for connected and 1 for not connected.
uint8_t communicationIsBluetoothConnected(void){
	return !palReadLine(LINE_ESP_GPIO0);
}