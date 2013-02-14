
CC=~/arm_cortex-m4_toolchain/bin/arm-none-eabi-gcc
SRC=energy_monitor.c
CFLAGS=-mfloat-abi=hard -mfpu=vfpv3-d16 -mcpu=cortex-m4 -mhard-float -g
# CFLAGS=

all:
	$(CC) $(CFLAGS) -Iinclude -T src/stm32_flash.ld $(SRC) src/system_stm32f4xx.c src/startup_stm32f4xx.s -o energy_monitor