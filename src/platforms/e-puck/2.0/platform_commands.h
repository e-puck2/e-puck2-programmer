#if defined(PLATFORM_COMMANDS_DEFINE)
/********************************************/
/* Begining of platform dedicated commands. */
/********************************************/
static bool cmd_en_esp32(target *t, int argc, const char **argv);
static bool cmd_gpio0_esp32(target *t, int argc, const char **argv);
static bool cmd_pwr_on_btn(target *t, int argc, const char **argv);
static bool cmd_vbus(target *t, int argc, const char **argv);
static bool cmd_usb_charge(target *t, int argc, const char **argv);
static bool cmd_usb_500(target *t, int argc, const char **argv);
static bool cmd_reset_F407(target *t, int argc, const char **argv);
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
	{"esp32_en", (cmd_handler)cmd_en_esp32, "(ON|OFF|) Set the EN_ESP32 pin or return the state of this one" }, \
	{"esp32_gpio0", (cmd_handler)cmd_gpio0_esp32, "(ON|OFF|) Set the GPIO0_ESP32 pin or return the state of this one" }, \
	{"pwr_on_btn", (cmd_handler)cmd_pwr_on_btn, "(ON|SHUTDOWN|) Power On or Shutdown the robot or return the state of Power On button" }, \
	{"vbus", (cmd_handler)cmd_vbus, "Return the state of VBus" }, \
	{"usb_charge", (cmd_handler)cmd_usb_charge, "(ON|OFF|) Set the USB_CHARGE pin or return the state of this one" }, \
	{"usb_500", (cmd_handler)cmd_usb_500, "(ON|OFF|) Set the USB_500 pin or return the state of this one" }, \
	{"reset_F407", (cmd_handler)cmd_reset_F407, "(ON|OFF|) Force the reset of F407" }, \

/***********************************************/
/* End of List of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_LIST
#endif

#if defined(PLATFORM_COMMANDS_CODE)
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

static bool cmd_gpio0_esp32(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("GPIO0_ESP32 state: %s\n",
			 platform_get_gpio0_esp32() ? "ON" : "OFF");
	else
		platform_set_gpio0_esp32(strcmp(argv[1], "ON") == 0);
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

static bool cmd_reset_F407(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Reset F407 state: %s\n",
			gpio_get(SRST_PORT, SRST_PIN) ? "OFF" : "ON");
	else if (strcmp(argv[1], "ON") == 0)
		gpio_clear(SRST_PORT, SRST_PIN);
 	else
		gpio_set(SRST_PORT, SRST_PIN);
	return true;
}

/***********************************************/
/* End of Code of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
