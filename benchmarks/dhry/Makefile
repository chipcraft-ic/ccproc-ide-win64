-include ../../common/legacy-env-var-support.mk

PROGNAME   := dhry
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

COMMON_SOURCES := syscalls.c uart.c core_util.c crc-verification.c core.S

ifndef RUNS_NUMBER
	CPPFLAGS += -DRUNS_NUMBER=1000000
	CPPFLAGS += -DPRINT_ALL
else
	CPPFLAGS += -DRUNS_NUMBER=$(RUNS_NUMBER)
endif

ifdef MI16_ENABLE
	CPPFLAGS += -D_MI16_ENABLE -mips16
	ifdef MI16_FLIP
		CPPFLAGS += -mflip-mips16
	endif
endif

#CPPFLAGS += -DREG=register
CPPFLAGS += -DHZ=1000 # see _CLOCKS_PER_SEC_ definition in NewLib
CPPFLAGS += -DNO_DEFAULT_SBRK -DNO_DEFAULT_LOCK

CFLAGS_WARN := -Wno-implicit-function-declaration
CFLAGS_OPT  := -O3 -Ofast -fno-inline-functions

LDFLAGS += -u _printf_float

# Put .data section in SPRAM
#LINKERSCRIPT := ccproc.ld
#STARTUP_SRC := startup.S

include $(COMMON_DIR)/generic.mk
