-include ../../common/legacy-env-var-support.mk

PROGNAME   := cpu-info
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

COMMON_SOURCES := syscalls.c uart.c

LDFLAGS += -u _printf_float

include $(COMMON_DIR)/generic.mk
