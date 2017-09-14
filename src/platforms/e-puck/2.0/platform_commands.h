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
	{"dfsdm", (cmd_handler)cmd_dfsdm, "Usage: dfsdm left|right"}, \
/***********************************************/
/* End of List of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_LIST
#endif

#if defined(PLATFORM_COMMANDS_CODE)
#include <../DFSDM/dfsdm.h>
#include "cdcacm.h"
#include <libopencm3/cm3/nvic.h>
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

static int32_t left_buffer[AUDIO_BUFFER_SIZE];
static DFSDM_config_t left_cfg = {
    .end_cb = dfsdm_data_callback,
    .error_cb = dfsdm_err_cb,
    .samples = left_buffer,
    .samples_len = AUDIO_BUFFER_SIZE
};

static int32_t right_buffer[AUDIO_BUFFER_SIZE];
static DFSDM_config_t right_cfg = {
    .end_cb = dfsdm_data_callback,
    .error_cb = dfsdm_err_cb,
    .samples = right_buffer,
    .samples_len = AUDIO_BUFFER_SIZE
};

static void cmd_dfsdm(target *t, int argc, const char **argv)
{
    (void) argc;
    (void) argv;
    (void) t;
    dfsdm_data_ready = false;

    if (argc != 2) {
        while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,"Usage: dfsdm left|right", 23) <= 0);
        return;
    }

    /* We use the callback arg to store which microphone is used. */
    if (!strcmp(argv[0], "left")) {
        left_cfg.cb_arg = (void*) 1;
        right_cfg.cb_arg = (void*) 0;
    } else {
        left_cfg.cb_arg = (void*) 0;
        right_cfg.cb_arg = (void*) 1;
    }

    dfsdm_start_conversion(&left_cfg, &right_cfg);

	while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT,"Done !\r\n", 8) <= 0);

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

	        for (i = 0; i < (AUDIO_BUFFER_SIZE / 2); i++) {
	            x = samples[i];
	            y = alpha * y + alpha * (x - x_prev);
	            x_prev = x;
	            samples[i] = y;
	            while(usbd_ep_write_packet(usbdev, CDCACM_UART_ENDPOINT, (uint8_t*)&y, 1) <= 0);
	        }
	        nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
	        nvic_enable_irq(NVIC_DMA2_STREAM1_IRQ);
        }
    }
}

/***********************************************/
/* End of Code of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
