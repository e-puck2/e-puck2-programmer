# e-puck2 Programmer (based on Black Magic Probe)
e-puck2 firmware for the built-in programmer/debugger of the robot. It is used to manage the power, some low-level controls of the robot, battery checking, USB charging, to program the ESP32, to act as a USB to ASEBA CAN or UART to USB translator and to run an embedded gdb server (based on Blackmagic firmware) to program and debug the STM32F407 microcontroller of the robot.

More details on the wiki of the robot :
[Wiki e-puck2](http://www.gctronic.com/doc/index.php/e-puck2)

## Major revisions

### New in version 2.2 of the code (e-puck-2.0_E_V2.2) :
1. Blackmagic has been updated from version **v1.6.1-249** to version **1.6.2-rc0** (1 year of commits)
2. ChibiOS has been updated from version **18.2 trunk** to version **20.3.x stable** (nearly 2 years of commits)

### New in version 2.1 of the code (e-puck-2.0_E_V2.1) :
1. Deep reorganization of the folder's structure. Now blackmagic is used as a submodule, like ChibiOS.
2. Updated the readme

### New in version 2 of the code (e-puck-2.0_E merged with branch port-to-chibios of july 2018) :
1. Code completely rewritten using ChibiOS
2. Now we can control the brightness of the three leds
3. The status of GDB is showed using the leds (programming, running, idle)
4. Now, when the bluetooth is connected, the blue led is ON
5. The Aseba Can translator works better now
6. In general, everything works smoother and better, also the code has a better organization and is easier to modify
7. No more dependencies with LibOpencm3, therefore we can compile the whole project without the need to compile first
   libopencm3 (no more Python support needed)
8. The robot is automatically turned on if we try to program it when it is off.
9. To ensure good behavior of the blinking of the blue LED while communicating with one of the three modes
   for the Serial monitor, we need now to be connected with DTR high when using a serial terminal on a computer most programs do it automatically)

## States of the RGB Led
### RGB led
The RGB led is used to indicate if the robot is **ON** or **OFF** (no matter the color)
- Leds **ON**				-> Robot is ON
- Leds **OFF**				-> Robot is OFF

### Red and Green leds
The **Red** and **Green** colors are used to indicate the **battery level** and the **status of GDB**
1. Battery Voltage
- **Green**					-> Battery between 4.2 and 3.5V
- **Orange**				-> Battery between 3.5 and 3.4V
- **Red**					-> Battery between 3.4 and 3.3V
- **Red blinking**			-> Battery below 3.3V
2. GDB status
- **Blinks**				-> Programming with GDB (blinks more or less fast depending on the communication speed)
- **Blinks**				-> Running the program with GDB (Blinks at regular speed)
- **Solid**					-> Program paused or disconnected from GDB

### Blue led
The Blue color is used to indicate the status of the bluetooth and the status of the communication of the USB Serial

- **Blinking**				-> A communication is active for one of the three modes of the Serial (UART_407_PASSTHROUGH, UART_ESP_PASSTHROUGH or ASEBA_CAN_TRANSLATOR)
- **ON**					-> The bluetooth is connected for the GDB or UART channel
- **OFF**					-> The bluetooth is disconnected

## How to compile
To compile the firmware, you need to have **arm-none-eabi** toolchain installed. 

The version tested for this firmware is the **arm-none-eabi 7 2017 q4 major**.

Once ready, simply **cd** into the folder **e-puck2-programmer/src** and run ``make``.
This will compile the firmware and you will find the result in **src/build** under the name ``e-puck2-programmer.bin/.elf/.hex``.