CC=~/sat_toolchain/bin/arm-none-eabi-gcc

FLAGS += -T libopencm3_stm32f4.ld
FLAGS += -D STM32F4
FLAGS += -g -fno-common
FLAGS += -mcpu=cortex-m4 -mthumb
FLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

SRC += energy_monitor.c


HLFLAGS += -lboost_thread-mt -lusb-1.0 -lreadline
HCFLAGS += -g -O2 -std=c++11

HOST_SRC = host/host_receiver.o host/libusbinterface.o host/dataprocessor.o host/helper.o

.PHONY: firmware hostapp all

all: energy_monitor
	make -j host_receiver

energy_monitor: energy_monitor.c Makefile
	$(CC) $(FLAGS) $(SRC) -o energy_monitor -lopencm3_stm32f4

host_receiver: $(HOST_SRC) Makefile
	g++ -o host_receiver $(HOST_SRC) $(HLFLAGS)

%.o : %.cpp
	g++ -c $(HCFLAGS) $< -o $@