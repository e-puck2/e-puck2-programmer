# Select the options that this Blackmagic Probe platform must have implemented
# Force because TRACESWO, DFU and JTAG are not available or intrinsic on this platform
# Force to 1 if support for ESP32 serial is needed
TRACESWO = 0
NO_SERIAL ?= 0
NO_DFU = 1
NO_JTAG= 1

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Istm32/include -mcpu=cortex-m4 -mthumb \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-DSTM32F4 -DEPUCK2 -I../libopencm3/include \
	-Iplatforms/stm32

ifeq ($(TRACESWO), 1)
CFLAGS += -DPLATFORM_HAS_TRACESWO
endif

ifeq ($(NO_SERIAL), 1)
CFLAGS += -DPLATFORM_HAS_NO_SERIAL
endif

ifeq ($(NO_DFU), 1)
CFLAGS += -DPLATFORM_HAS_NO_DFU_BOOTLOADER
endif

ifeq ($(NO_JTAG), 1)
CFLAGS += -DPLATFORM_HAS_NO_JTAG
endif

LDFLAGS = -lopencm3_stm32f4 -Wl,--defsym,_stack=0x20010000 \
	-Wl,-T,platforms/e-puck/2.0/e-puck2.ld -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m4 -Wl,-gc-sections \
	-mfloat-abi=hard -mfpu=fpv4-sp-d16 \
	-L../libopencm3/lib

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\
	platforms/e-puck/USB251XB/USB251XB.c \
	platforms/e-puck/USB251XB/SMBus.c \

ifeq ($(TRACESWO), 1)
SRC += 	traceswo.c
endif

ifeq ($(NO_SERIAL), 0)
SRC += 	usbuart.c
endif

all:	blackmagic.bin

host_clean:
	-$(Q)$(RM) blackmagic.bin
