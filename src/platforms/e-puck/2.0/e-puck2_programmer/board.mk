# List of all the board related files.
BOARDSRC = $(CHIBIOS)/test_usb_413/e-puck2_programmer/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/test_usb_413/e-puck2_programmer/

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
