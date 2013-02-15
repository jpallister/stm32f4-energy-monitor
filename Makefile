
CC=~/arm_cortex-m4_toolchain/bin/arm-none-eabi-gcc
SRC=energy_monitor.c
LIB_SRC=std_lib/src/stm32f4xx_gpio.c std_lib/src/stm32f4xx_rcc.c std_lib/src/stm32f4xx_adc.c std_lib/src/stm32f4xx_dma.c std_lib/src/misc.c std_lib/src/stm32f4xx_tim.c
CFLAGS=-mfloat-abi=hard -mfpu=vfpv3-d16 -mcpu=cortex-m4 -mhard-float -g
IDIRS= -Iinclude -Istd_lib/inc
LSCR= -T src/stm32_flash.ld
PSRC= src/system_stm32f4xx.c src/startup_stm32f4xx.s
# CFLAGS=

all:
	$(CC) $(CFLAGS) $(IDIRS) $(LSCR) $(SRC) $(LIB_SRC) $(PSRC) -o energy_monitor