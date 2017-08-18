# Select the options that this Blackmagic Probe platform must have implemented
# Force because JTAG is not available on this platform and some one are not yet managed
TRACESWO = 0
NO_SERIAL ?= 1
NO_DFU = 0
NO_JTAG = 1
CMD_PLAT = 0

ifeq ($(CMD_PLAT), 1)
	CFLAGS += -DPLATFORM_HAS_COMMANDS
endif

CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

OPT_FLAGS = -Os
CFLAGS += -mcpu=cortex-m3 -mthumb \
	-DSTM32F1 -DEPUCK1_4 -I../libopencm3/include \
	-I platforms/stm32

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

LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs \
	-lopencm3_stm32f1 -Wl,--defsym,_stack=0x20005000 \
	-Wl,-T,platforms/stm32/stlink.ld -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8002000

VPATH += platforms/stm32

SRC += 	cdcacm.c	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\

ifeq ($(NO_SERIAL), 0)
		SRC += 	usbuart.c
endif

all:	blackmagic.bin blackmagic_dfu.bin blackmagic_dfu.hex dfu_upgrade.bin dfu_upgrade.hex

blackmagic_dfu: usbdfu.o dfucore.o dfu_f1.o
	@echo "  LD      $@"
	$(Q)$(CC) $^ -o $@ $(LDFLAGS_BOOT)

dfu_upgrade: dfu_upgrade.o dfucore.o dfu_f1.o
	@echo "  LD      $@"
	$(Q)$(CC) $^ -o $@ $(LDFLAGS)

host_clean:
	-$(Q)$(RM) blackmagic.bin blackmagic_dfu blackmagic_dfu.bin blackmagic_dfu.hex dfu_upgrade dfu_upgrade.bin dfu_upgrade.hex
