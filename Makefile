CC=~/sat_toolchain/bin/arm-none-eabi-gcc

FLAGS += -T libopencm3_stm32f4.ld

FLAGS += -D STM32F4
FLAGS += -g -fno-common
FLAGS += -mcpu=cortex-m4 -mthumb
FLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

SRC += energy_monitor.c

all:
	$(CC) $(FLAGS) $(SRC) -o energy_monitor -lopencm3_stm32f4


recv: recv.c
	gcc -o recv recv.c -lusb-1.0 -g
