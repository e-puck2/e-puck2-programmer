#if defined(PLATFORM_COMMANDS_DEFINE)
/*******************************************
* Begining of platform dedicated commands. *
********************************************/
static bool cmd_blue(target *t, int argc, const char **argv);
static bool cmd_green(target *t, int argc, const char **argv);
static bool cmd_orange(target *t, int argc, const char **argv);
static bool cmd_red(target *t, int argc, const char **argv);
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
	{"blue", (cmd_handler)cmd_blue, "Set the blue LED state: (ON|OFF)" }, \
	{"green", (cmd_handler)cmd_green, "Set the green LED state: (ON|OFF)" }, \
	{"red", (cmd_handler)cmd_red, "Set the red LED state: (ON|OFF)" }, \
	{"orange", (cmd_handler)cmd_orange, "Set the orange LED state: (ON|OFF)" }, \
/***********************************************/
/* End of List of platform dedicated commands. */
/***********************************************/
#undef PLATFORM_COMMANDS_LIST
#endif

#if defined(PLATFORM_COMMANDS_CODE)
/****************************************************/
/* Begining of Code of platform dedicated commands. */
/****************************************************/
static bool cmd_blue(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Blue led state: %s\n",
			 platform_get_blue() ? "ON" : "OFF");
	else
		platform_set_blue((strcmp(argv[1], "ON"))==0);
	return true;
}

static bool cmd_green(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Green led state: %s\n",
			 platform_get_green() ? "ON" : "OFF");
	else
		platform_set_green((strcmp(argv[1], "ON"))==0);
	return true;
}

static bool cmd_red(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Red led state: %s\n",
			 platform_get_red() ? "ON" : "OFF");
	else
		platform_set_red((strcmp(argv[1], "ON"))==0);
	return true;
}

static bool cmd_orange(target *t, int argc, const char **argv)
{
	(void)t;
	if (argc == 1)
		gdb_outf("Orange led state: %s\n",
			 platform_get_orange() ? "ON" : "OFF");
	else
		platform_set_orange((strcmp(argv[1], "ON"))==0);
	return true;
}

/***********************************************/
/* End of Code of platform dedicated commands. */
/**********************************************/
#undef PLATFORM_COMMANDS_CODE
#endif
