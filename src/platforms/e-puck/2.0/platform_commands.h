#if defined(PLATFORM_COMMANDS_DEFINE)
/********************************************/
/* Begining of platform dedicated commands. */
/********************************************/
static bool cmd_en_esp32(target *t, int argc, const char **argv);
static bool cmd_pwr_on_btn(target *t, int argc, const char **argv);
static bool cmd_vbus(target *t, int argc, const char **argv);
static bool cmd_usb_charge(target *t, int argc, const char **argv);
static bool cmd_usb_500(target *t, int argc, const char **argv);
static void cmd_dfsdm(target *t, int argc, const char **argv);
/***************************************/
/* End of platform dedicated commands. */
/***************************************/
#undef PLATFORM_COMMANDS_DEFINE
#endif

#if defined(PLATFORM_COMMANDS_LIST)
/****************************************************/
/* Begining of List of platform dedicated commands. */
/* IMPORTANT : Each line MUST finish with "\"       */
/****************************************************/
	{"en_esp32", (cmd_handler)cmd_en_esp32, "(ON|OFF|) Set the EN_ESP32 pin or return the state of this one" }, \
	{"pwr_on_btn", (cmd_handler)cmd_pwr_on_btn, "(ON|SHUTDOWN|) Power On or Shutdown the robot or return the state of Power On button" }, \
	{"vbus", (cmd_handler)cmd_vbus, "Return the state of VBus" }, \
	{"usb_charge", (cmd_handler)cmd_usb_charge, "(ON|OFF|) Set the USB_CHARGE pin or return the state of this one" }, \
	{"usb_500", (cmd_handler)cmd_usb_500, "(ON|OFF|) Set the USB_500 pin or return the state of this one" }, \
	{"dfsdm", (cmd_handler)cmd_dfsdm, "Usage: dfsdm left|right|top|bottom"}, \
/***********************************************/
/* End of List of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_LIST
#endif

#if defined(PLATFORM_COMMANDS_CODE)
#include <../DFSDM/dfsdm.h>
#include "cdcacm.h"
#include <libopencm3/cm3/nvic.h>
#include "gdb_packet.h" 
/****************************************************/
/* Begining of Code of platform dedicated commands. */
/****************************************************/
static bool cmd_en_esp32(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("EN_ESP32 state: %s\n",
			 platform_get_en_esp32() ? "ON" : "OFF");
	else
		platform_set_en_esp32(strcmp(argv[1], "ON") == 0);
	return true;
}

static bool cmd_pwr_on_btn(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Power On button: %s\n",
			 platform_pwr_on_btn_pressed() ? "Pressed" : "Released");
	else if (strcmp(argv[1], "SHUTDOWN") == 0)
		platform_pwr_on(false);
	else if (strcmp(argv[1], "ON") == 0)
		platform_pwr_on(true);
	return true;
}

static bool cmd_vbus(target *t, int argc, const char **argv)
{
	(void)t;
	(void)argv;
	if (argc == 1)
		gdb_outf("VBus: %s\n", platform_vbus() ? "ON" : "OFF");
	return true;
}

static bool cmd_usb_charge(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("USB_CHARGE state: %s\n",
			 platform_get_usb_charge() ? "ON" : "OFF");
	else
		platform_set_usb_charge(strcmp(argv[1], "ON") == 0);
	return true;
}

static bool cmd_usb_500(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("USB_500 state: %s\n",
			 platform_get_usb_500() ? "ON" : "OFF");
	else
		platform_set_usb_500(strcmp(argv[1], "ON") == 0);
	return true;
}

static int32_t audio_buffer[AUDIO_BUFFER_SIZE] = {0};
static DFSDM_config_t* mic_used;

/*microphones config rules :
* 1) 	The DMA, DMA stream and DMA channel must be set accordingly with the DMA request mapping
* 		to be able to work with the DFSDM filter chosen
* 
* 2) 	The DSFDM channel is dependant on the hardware conection of the microphone
* 3)	Two microphones can be connected to one input, then each channel can be connected
* 		to its line or the its line+1. It should be configured accordingly with the hardware conenction
* 4)	The edge specifiy which microphone connected to the line has to be read
* 5) 	Beware of the hardware limitations of the uC. DFSDM1 has 3 inputs, 4 channels and 2 filters
* 		and DFSDM2 had 4 inputs, 8 channels and 4 filters
* 		
* 		If one filter can only compute on channel at at time and one input can connect two microphones
* 		
* 		Actual connection for e-puck 2 first prototype :
* 		
* 		Left and Right mics connected to DFSDM1 input 1 (channel 0 and 1 can read)
* 		Top and Bottom mics connected to DFSDM1 input 2 (channel 1 and 2 can read)
* 		
*/
static DFSDM_config_t left_cfg = {
    .end_cb = dfsdm_data_callback,
    .error_cb = dfsdm_err_cb,
    .cb_arg = (void*) 1,
    .samples = audio_buffer,
    .samples_len = AUDIO_BUFFER_SIZE,
    .dma = DMA2,
    .dma_stream = DMA_STREAM0,
    .dma_channel = DMA_SxCR_CHSEL_7,
    .dma_priority = STM32_DFSDM_MICROPHONE_DMA_PRIORITY,
    .dfsdm_channel = DFSDM1_Channel0,
    .dfsdm_input =	DFSDM_NEXT,
    .dfsdm_edge = DFSDM_FALLING_EDGE,
    .dfsdm_filter = DFSDM1_Filter0,
};

static DFSDM_config_t right_cfg = {
    .end_cb = dfsdm_data_callback,
    .error_cb = dfsdm_err_cb,
    .cb_arg = (void*) 1,
    .samples = audio_buffer,
    .samples_len = AUDIO_BUFFER_SIZE,
    .dma = DMA2,
    .dma_stream = DMA_STREAM1,
    .dma_channel = DMA_SxCR_CHSEL_3,
    .dma_priority = STM32_DFSDM_MICROPHONE_DMA_PRIORITY,
    .dfsdm_channel = DFSDM1_Channel1,
    .dfsdm_input =	DFSDM_SELF,
    .dfsdm_edge = DFSDM_RISING_EDGE,
    .dfsdm_filter = DFSDM1_Filter1,
};

static DFSDM_config_t bottom_cfg = {
    .end_cb = dfsdm_data_callback,
    .error_cb = dfsdm_err_cb,
    .cb_arg = (void*) 1,
    .samples = audio_buffer,
    .samples_len = AUDIO_BUFFER_SIZE,
    .dma = DMA2,
    .dma_stream = DMA_STREAM0,
    .dma_channel = DMA_SxCR_CHSEL_7,
    .dma_priority = STM32_DFSDM_MICROPHONE_DMA_PRIORITY,
    .dfsdm_channel = DFSDM1_Channel2,
    .dfsdm_input =	DFSDM_SELF,
    .dfsdm_edge = DFSDM_FALLING_EDGE,
    .dfsdm_filter = DFSDM1_Filter0,
};

static DFSDM_config_t top_cfg = {
    .end_cb = dfsdm_data_callback,
    .error_cb = dfsdm_err_cb,
    .cb_arg = (void*) 1,
    .samples = audio_buffer,
    .samples_len = AUDIO_BUFFER_SIZE,
    .dma = DMA2,
    .dma_stream = DMA_STREAM0,
    .dma_channel = DMA_SxCR_CHSEL_7,
    .dma_priority = STM32_DFSDM_MICROPHONE_DMA_PRIORITY,
    .dfsdm_channel = DFSDM1_Channel2,
    .dfsdm_input =	DFSDM_SELF,
    .dfsdm_edge = DFSDM_RISING_EDGE,
    .dfsdm_filter = DFSDM1_Filter0,
};

static void cmd_dfsdm(target *t, int argc, const char **argv)
{
    (void) argc;
    (void) argv;
    (void) t;
    dfsdm_data_ready = false;

    if (argc != 2) {
    	gdb_out("Usage: dfsdm left|right|top|down\n");
        return;
    }

    /* We use the callback arg to store which microphone is used. */
    if (!strcmp(argv[1], "left")) {
    	mic_used = &left_cfg;
    }else if (!strcmp(argv[1], "right")) {
    	mic_used = &right_cfg;
    }else if (!strcmp(argv[1], "top")){
    	mic_used = &top_cfg;
    }else if (!strcmp(argv[1], "bottom")){
    	mic_used = &bottom_cfg;
    }

    dfsdm_start_conversion(mic_used, DFSDM_ONESHOT);

    /* High pass filter params */
    const float tau = 1 / 20.; /* 1 / cutoff */
    const float dt = 1./44e3;  /* Sampling period */

    const float alpha = tau / (tau + dt);

    int32_t x_prev = 0, y = 0, x;
    while (true) {
        if(dfsdm_data_ready){
        	dfsdm_data_ready = false;
	    	uint16_t i;
	        /* First order high pass filtering is used to remove the DC component
	         * of the signal. */

	        for (i = 0; i < (AUDIO_BUFFER_SIZE); i++) {
	            x = mic_used->samples[i];
	            y = alpha * y + alpha * (x - x_prev);
	            x_prev = x;
	            mic_used->samples[i] = y;
	        }

	        /* here, we don't send half a buffer at every interruption of the DMa because 
	           we don't reach a sufficient speed to send them before the buffer is modified
	           again so when the buffer is full, the DMA stream is disabled and the full buffer
	           is sent */
	        while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,"Done !\r\n", 8) <= 0);
	        uint16_t nb = 0;
	        uint8_t* pointeur = (uint8_t*) mic_used->samples;
		    while (nb<((((AUDIO_BUFFER_SIZE)*4)/50))){
		    	while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, &pointeur[nb*50], 50) <= 0);
		    	nb+=1;
		    }
		    return;
        }
    }
}

/***********************************************/
/* End of Code of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
