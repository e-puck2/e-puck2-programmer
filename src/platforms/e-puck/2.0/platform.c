#include <stdio.h>
#include <string.h>

#include <platform.h>

#include <usb_hub.h>

void panic_handler(const char *reason)
{
    (void)reason;

    palClearLine(LINE_LED_GREEN);
    palClearLine(LINE_LED_RED);
    palClearLine(LINE_LED_BLUE);
  
    while (true) {

    }
}


static THD_WORKING_AREA(test_thd_wa, 256);
static THD_FUNCTION(test_thd, arg)
{
    (void) arg;

    while(1){
      static systime_t time_before = 0;
      static systime_t time = 0;
      time_before = time;
      time = chVTGetSystemTime();
      chprintf((BaseSequentialStream *) &SDU2,"hello 2 %d\n",time-time_before);
      /* Sleep for some time. */
      chThdSleepMilliseconds(1);
    }
}




int main(void) {
  

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
  * Starts the thread managing the USB hub
  */
  usb_hub_start();
  /*
   * Initializes two serial-over-USB CDC drivers.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg1);
  sduObjectInit(&SDU2);
  sduStart(&SDU2, &serusbcfg2);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg1.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg1.usbp, &usbcfg);
  usbConnectBus(serusbcfg1.usbp);

  chThdCreateStatic(test_thd_wa, sizeof(test_thd_wa), NORMALPRIO, test_thd, NULL);

  while (true) {
      static systime_t time_before = 0;
      static systime_t time = 0;
      time_before = time;
      time = chVTGetSystemTime();

      chprintf((BaseSequentialStream *) &SDU1,"hello 1 %d\n",time-time_before);
      palToggleLine(LINE_LED_RED);
      chThdSleepMilliseconds(1);
  }
}
