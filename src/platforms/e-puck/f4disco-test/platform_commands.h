#if defined(PLATFORM_COMMANDS_DEFINE)
/*******************************************
* Begining of platform dedicated commands. *
********************************************/
static bool cmd_set_blue_led_val(target *t, int argc, const char **argv);
static bool cmd_get_blue_led_val(void);
static bool cmd_set_green_led_val(target *t, int argc, const char **argv);
static bool cmd_get_green_led_val(void);
/**************************************
* End of platform dedicated commands. *
**************************************/
#undef PLATFORM_COMMANDS_DEFINE
#endif

#if defined(PLATFORM_COMMANDS_LIST)
/***************************************************
* Begining of List of platform dedicated commands. *
***************************************************/
	{"set_blue_led_val", (cmd_handler)cmd_set_blue_led_val, "Set the blue LED state: (ON|OFF)" },
	{"get_blue_led_val", (cmd_handler)cmd_get_blue_led_val, "Get the blue LED state" },
	{"set_green_led_val", (cmd_handler)cmd_set_green_led_val, "Set the green LED state: (ON|OFF)" },
	{"get_green_led_val", (cmd_handler)cmd_get_green_led_val, "Get the green LED state" },
/**********************************************
* End of List of platform dedicated commands. *
**********************************************/
#undef PLATFORM_COMMANDS_LIST
#endif

#if defined(PLATFORM_COMMANDS_CODE)
/***************************************************
* Begining of Code of platform dedicated commands. *
***************************************************/
static bool cmd_set_blue_led_val(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Blue led state: %s\n",
			 !platform_get_blue_led_val() ? "ON" : "OFF");
	else
		platform_set_blue_led_val(strcmp(argv[1], "ON"));
	return true;
}

static bool cmd_get_blue_led_val(void)
{
	gdb_outf("Blue led state: %s\n",
		 !platform_get_blue_led_val() ? "ON" : "OFF");
	return true;
}
static bool cmd_set_green_led_val(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Green led state: %s\n",
			 !platform_get_green_led_val() ? "ON" : "OFF");
	else
		platform_set_green_led_val(strcmp(argv[1], "ON"));
	return true;
}

static bool cmd_get_green_led_val(void)
{
	gdb_outf("Green led state: %s\n",
		 !platform_get_green_led_val() ? "ON" : "OFF");
	return true;
}
/**********************************************
* End of Code of platform dedicated commands. *
**********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
