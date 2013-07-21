CC=arm-none-eabi-gcc

FLAGS += -T libopencm3_stm32f4.ld
FLAGS += -D STM32F4
FLAGS += -g -fno-common -save-temps
FLAGS += -mcpu=cortex-m4 -mthumb
FLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

SRC += energy_monitor.c

HOST_SRC = host/host_receiver.o host/libusbinterface.o host/dataprocessor.o host/helper.o

.PHONY: firmware hostapp all

all: energy_monitor host_receiver

energy_monitor: energy_monitor.c Makefile
	$(CC) $(FLAGS) $(SRC) -o energy_monitor -lopencm3_stm32f4

host_receiver:
	cd host; ./configure
	$(MAKE) -C host

clean:
	$(MAKE) -C host clean
	rm -f energy_monitor
