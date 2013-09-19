bin_PROGRAMS=energy_monitor
energy_monitor_SOURCES=energy_monitor.c

CC= arm-none-eabi-gcc
AM_CFLAGS  = -D STM32F4
AM_CFLAGS += -mcpu=cortex-m4 -mthumb
AM_CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

#AM_LDFLAGS  = -mcpu=cortex-m4 -mthumb
#AM_LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
AM_LDFLAGS = -lopencm3_stm32f4
