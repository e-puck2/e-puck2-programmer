#include <stdio.h>
#include <string.h>

#include "main.h"

void platform_srst_set_val(bool assert)
{
	if (assert) {
		gpio_clear(SRST_PORT, SRST_PIN);
		chThdSleepMilliseconds(1);
	} else {
		gpio_set(SRST_PORT, SRST_PIN);
	}
}

bool platform_srst_get_val(void)
{
	return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}
