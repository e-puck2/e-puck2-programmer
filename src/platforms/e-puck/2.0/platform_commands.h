#if defined(PLATFORM_COMMANDS_DEFINE)
/********************************************/
/* Begining of platform dedicated commands. */
/********************************************/
static bool cmd_en_esp32(target *t, int argc, const char **argv);
static bool cmd_pwr_on_btn(target *t, int argc, const char **argv);
/*
static bool cmd_get_blue_led_val(void);
static bool cmd_set_green_led_val(target *t, int argc, const char **argv);
static bool cmd_get_green_led_val(void);
*/
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
	{"pwr_on_btn", (cmd_handler)cmd_pwr_on_btn, "(|SHUTDOWN) Return the state of Power On button OR Shutdown the system" }, \
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
		gdb_outf("Blue led state: %s\n",
			 !platform_get_en_esp32() ? "ON" : "OFF");
	else
		platform_set_en_esp32(strcmp(argv[1], "ON"));
	return true;
}

static bool cmd_pwr_on_btn(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Power On button: %s\n",
			 !platform_pwr_on_btn() ? "Pressed" : "Released");
	else
		platform_pwr_on(!strcmp(argv[1], "SHUTDOWN"));
	return true;
}

/***********************************************/
/* End of Code of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
