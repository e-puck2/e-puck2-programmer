#include <stdio.h>
#include <string.h>

#include <ch.h>
#include <hal.h>

#include <shell.h>
#include <chprintf.h>

#include <platform.h>

#include <usbcfg.h>
#include <i2c_smbus.h>
#include <USB251XB.h>

void panic_handler(const char *reason)
{
    (void)reason;

    palClearLine(LINE_LED_GREEN);
    palClearLine(LINE_LED_RED);
    palClearLine(LINE_LED_BLUE);
  
    while (true) {

    }
}

static THD_WORKING_AREA(usb_hub_thd_wa, 128);
static THD_FUNCTION(usb_hub_thd, arg)
{
  (void) arg;
  bool hub_state = NOT_CONFIGURED;

  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(LINE_VBUS_DET, PAL_EVENT_MODE_BOTH_EDGES);
  //init the first time the hub
  USB251XB_init(USB2512B);
  hub_state = CONFIGURED;

  while(1){
    //waiting until an event on the line is detected
    palWaitLineTimeout(LINE_VBUS_DET, TIME_INFINITE);
    //wait a few moments to be sure the interruption was not triggered
    //by a glitch and then test the GPIO
    //also wait for the USB HUB to be running
    chThdSleepMilliseconds(DEBOUNCE_TIME_VBUS_DET_MS);

    if(palReadLine(LINE_VBUS_DET)){
      if(hub_state == NOT_CONFIGURED){
        USB251XB_init(USB2512B);
        hub_state = CONFIGURED;
      }
      
    }else{
      hub_state = NOT_CONFIGURED;
    }
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

  //SMBUS init
  i2c_smbus_start();

  //Vbus detection init. Used to configure the USB Hub when we detect an USB cable
  chThdCreateStatic(usb_hub_thd_wa, sizeof(usb_hub_thd_wa), NORMALPRIO, usb_hub_thd, NULL);
  
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
