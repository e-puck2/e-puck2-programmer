
#include "platform.h"

#include "general.h"
#include "gdb_if.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "morse.h"

static THD_WORKING_AREA(test_thd_wa, 20480);
static THD_FUNCTION(test_thd, arg)
{
	(void) arg;

	while(1){
		// static systime_t time_before = 0;
		// static systime_t time = 0;
		// time_before = time;
		// time = chVTGetSystemTime();
		// chprintf((BaseSequentialStream *) &SDU2,"hello 2 %d\n",time-time_before);
		/* Sleep for some time. */
		volatile struct exception e;
		TRY_CATCH(e, EXCEPTION_ALL) {
			gdb_main();
		}
		if (e.type) {
			gdb_putpacketz("EFF");
			target_list_free();
			morse("TARGET LOST.", 1);
		}		
		chThdSleepMilliseconds(1);
	}
}



void gdbStart(void){
	/**
	 * Starts a Systick emulation for GDB
	 */
	platform_timing_init();

	/**
	 * Starts the GDB thread
	 */
	chThdCreateStatic(test_thd_wa, sizeof(test_thd_wa), NORMALPRIO, test_thd, NULL);
}