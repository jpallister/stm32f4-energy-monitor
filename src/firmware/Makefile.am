bin_PROGRAMS=energy_monitor
energy_monitor_SOURCES=energy_monitor.c
energy_monitor_DEPENDENCIES=libopencm3_stm32f4.ld

AM_CFLAGS  = -D STM32F4
AM_CFLAGS += -mcpu=cortex-m4 -mthumb -O2
AM_CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

#AM_LDFLAGS  = -mcpu=cortex-m4 -mthumb
#AM_LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
AM_LDFLAGS = -lopencm3_stm32f4 -T $(srcdir)/libopencm3_stm32f4.ld

