Specific to the e-puck2 mobile robot:

This platform use 2 STM32F4xx uCs
1. An STM32F413 to implement the low-level microcontroller,
   including BlackMagic Probe modified software with dedicated other features
   like the power management and USB HUB configuration.
2. An STM32F407 as main uC for the mobile robotic platform. This one is program-
   med from the first one.

System vs BMP Bootloader
========================
We use the STM32F413 internal DFU bootloader and there is no code to provide and
no control except the Boot0 pin (see below).
=======
System vs BMP Bootloader
========================
For the BMP bootloader, flashing was not reliable. So we use the system
bootloader unconditional.

Connections:
====================

> Boot0: Pin Boot0 to force system bootloader entry AFTER reset.
The e-puck2 must be OFF and the Boot0 jumper short-circuited then the USB must
be connected to a PC with DFU util tools in order to reprogram the BlackMagic
Probe firmware.

Not implemented yet but could be done later:
1. An eventual Black Magic Trace Capture: It needs 1 link between the 2 uCs
(SWO on the 407)

> PA0: TDI - if needed but try to avoid to use it, that's the case yet
> PA10: TMS/SWDIO
> PA5: TCK/SWCLK
> PA1: TDO/TRACESWO - if needed but try to avoid to use it, that's the case yet

> PB4: TRST - if needed but try to avoid to use it, that's the case yet
> PB0: SRST - RESET of STM32F407

> PA14/Blue Led: Indicator of serial activity
> PA13/Red Led:  Error indicator
> PA15/Green Led: Idle/Run indicator

USB connection detection.
> PA9: VBUS (USB_PRESENT or PRT_PROG)

Charger management. For the moment only GDB dedicated functions will allow
to check the input/output states and manage the output states.
Later an autonomous state machine will do the job autonomously.
> PB5: USB_CHARGE output
> PB6: USB_500 output

GDB dedicated functions :
=================================
Use "mon help" to list the dedicated function for the platform when connected with GDB
New and specific functions are added via platform_commands.h file.


ToDo : 
====================

1) Extension connector management. Not implemented yet.
    PB12: Extension system enable (EXT_SYSEN) output to control the supplies
          of futur extention boards.


New in version 2 of the code (july 2018) :
====================================

1) Code completely rewritten using ChibiOS
2) Now we can control the brigthness of the three leds
3) The status of GDB is showed using the leds (programming, running, idle)
4) Now, when the bluetooth is connected, the blue led is ON
5) The Aseba Can translor works better now
6) In general, everything works smoother and better, also the code has a better organization and is easier to modify
7) No more dependencies with LibOpencm3, therefore we can compile the whole project without the need to compile first
   libopencm3 (no more Python support needed)
8) The robot is automatically turned on if we try to program it when it is off.
9) To ensure good behavior of the blinking of the blue LED while communicating with one of the three modes
   for the Serial monitor, we need now to be connected with DTR high when using a serial terminal on a computer most programs do it automatically)

States of the RGB Led
====================================
0) The RGB led is used to indicate if the robot is ON or OFF (no matter the color)
-> Leds ON      = Robot is ON
-> Leds OFF     = Robot is OFF

1) The Red and Green colors are used to indicate the battery level and the status of GDB
  a) Battery Voltage
  ->Green         = Battery between 4.2 and 3.5V
  ->Orange        = Battery between 3.5 and 3.4V
  ->Red           = Battery between 3.4 and 3.3V
  ->Red blinking  = Battery below 3.3V
  b) GDB status
  ->Blinks        = Programming with GDB (blinks more or less fast depending on the communication speed)
  ->Blinks        = Running the program with GDB (Blinks at regular speed)
  ->Solid         = Program paused or disconnected from GDB
2)The Blue color is used to indicate the status of the bluetooth and the status of the communication of the USB Serial
-> Blinking       = A communication is active for one of the three mode of the Serial monitor
                    (UART_407_PASSTHROUGH, UART_ESP_PASSTHROUGH or ASEBA_CAN_TRANSLATOR)
-> ON             = The bluetooth is connected for the GDB or UART channel
-> OFF            = The bluetooth is disconnected
