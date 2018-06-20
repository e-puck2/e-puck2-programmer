#if defined(_WIN32) || defined(WIN32) || defined(__CYGWIN__) || defined(__MINGW32__) || defined(__BORLANDC__)
#define OS_WIN
#endif

PROBE_HOST ?= native
PLATFORM_DIR = platforms/$(PROBE_HOST)

ifeq ($(PROBE_HOST),e-puck/2.0)
#jumps to the makefile of the project instead of this one
all:
	cd $(PLATFORM_DIR) && make $(MAKEFLAGS)
clean:
	cd $(PLATFORM_DIR) && make $(MAKEFLAGS) clean
else


VPATH += $(PLATFORM_DIR) platforms/common target
ENABLE_DEBUG ?=
NO_JTAG ?= 0
ONLY_STM32F4 ?= 0

ifneq ($(V), 1)
MAKEFLAGS += --no-print-dir
Q := @
endif

OPT_FLAGS ?= -O2

CFLAGS += -Wall -Wextra -Werror -Wno-char-subscripts\
	$(OPT_FLAGS) -std=gnu99 -g3 -MD \
	-I. -Iinclude -Iplatforms/common -I$(PLATFORM_DIR)
LDFLAGS += $(OPT_FLAGS)

ifeq ($(ENABLE_DEBUG), 1)
CFLAGS += -DENABLE_DEBUG
endif

SRC =			\
	adiv5.c		\
	adiv5_swdp.c	\
	command.c	\
	cortexa.c	\
	cortexm.c	\
	crc32.c		\
	exception.c	\
	gdb_if.c	\
	gdb_main.c	\
	gdb_hostio.c	\
	gdb_packet.c	\
	hex_utils.c	\
	main.c		\
	morse.c		\
	platform.c	\
	swdptap.c	\
	swdptap_generic.c	\
	target.c	\

include $(PLATFORM_DIR)/Makefile.inc

# WARNING : MUST STAY AFTER include of Makefile.inc !!
#	because these options are globally defined BUT must/can be defined in this dedicated included platform's file
#		NO_JTAG : about use of dedicated pins for JTAG interface
#		ONLY_STM32F4 : about target than can be debugged
#			dedicated included platform's file
SRC +=		\
	stm32f4.c	\

ifeq ($(ONLY_STM32F4), 0)
SRC +=		\
	stm32f1.c	\
	stm32l0.c	\
	stm32l4.c	\
	lpc_common.c	\
	lpc11xx.c	\
	lpc15xx.c	\
	lpc43xx.c	\
	sam3x.c		\
	sam4l.c	\
	nrf51.c		\
	samd.c		\
	lmi.c		\
	kinetis.c	\
	efm32.c
endif

ifeq ($(NO_JTAG), 0)
SRC +=			\
	adiv5_jtagdp.c \
	jtag_scan.c	\
	jtagtap.c	\
	jtagtap_generic.c
endif

OBJ = $(SRC:.c=.o)

blackmagic: include/version.h $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS)

%.o:	%.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.bin:	%
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $^ $@

%.hex:	%
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O ihex $^ $@

.PHONY:	clean host_clean all_platforms FORCE

clean:	host_clean
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) -f *.o *.d *~ blackmagic $(HOSTFILES)
	-$(Q)$(RM) -f platforms/*/*/*.o platforms/*/*/*.d	# e-puck platform's addition
	-$(Q)$(RM) -f platforms/*/*.o platforms/*/*.d mapfile include/version.h
	-$(Q)$(RM) -r platforms/*/*/.dep platforms/*/*/build

all_platforms:
	$(Q)set -e ;\
	mkdir -p artifacts/$(shell git describe --always) ;\
	echo "<html><body><ul>" > artifacts/index.html ;\
	for i in platforms/*/Makefile.inc ; do \
		export DIRNAME=`dirname $$i` ;\
		export PROBE_HOST=`basename $$DIRNAME` ;\
		export CFLAGS=-Werror ;\
		echo "Building for hardware platform: $$PROBE_HOST" ;\
		$(MAKE) $(MAKEFLAGS) clean ;\
		$(MAKE) $(MAKEFLAGS);\
		if [ -f blackmagic.bin ]; then \
			mv blackmagic.bin artifacts/blackmagic-$$PROBE_HOST.bin ;\
			echo "<li><a href='blackmagic-$$PROBE_HOST.bin'>$$PROBE_HOST</a></li>"\
				>> artifacts/index.html ;\
		fi ;\
	done ;\
	echo "</ul></body></html>" >> artifacts/index.html ;\
	cp artifacts/*.bin artifacts/$(shell git describe --always)

command.c: include/version.h

include/version.h: FORCE
	$(Q)echo "  GIT     include/version.h"
	$(Q)echo "#define FIRMWARE_VERSION \"`git describe --always --dirty`\"" > $@.checked
	$(Q)if ! diff "$@" "$@.checked" > /dev/null; then \
	    mv $@.checked $@; \
	else \
			rm $@.checked; \
	fi; \

-include *.d

endif
