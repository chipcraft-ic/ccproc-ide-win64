-include ../../common/legacy-env-var-support.mk

PROGNAME   := flash-test
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

COMMON_SOURCES := syscalls.c uart.c core.S

CPPLAGS += -D_STARTUP_STACK_SIZE=12288

include $(COMMON_DIR)/generic.mk
