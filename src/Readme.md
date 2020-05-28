Specific to the e-puck2 mobile robot:
-------------------------------------

This platform use 2 STM32F4xx uCs
1. An STM32F413 to implement the low-level microcontroller,
   including BlackMagic Probe modified software with dedicated other features
   like the power management and USB HUB configuration.
2. An STM32F407 as main uC for the mobile robotic platform. This one is programmed from the first one.

System vs BMP Bootloader
------------------------
We use the STM32F413 internal DFU bootloader to program it.

GDB dedicated functions :
-------------------------
Use "mon help" to list the dedicated function for the platform when connected with GDB
New and specific functions are added via platform_commands.h file.