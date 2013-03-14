CC=~/sat_toolchain/bin/arm-none-eabi-gcc

FLAGS += -T libopencm3_stm32f4.ld

FLAGS += -D STM32F4
# FLAGS += -O
FLAGS += -g -fno-common
FLAGS += -mcpu=cortex-m4 -mthumb
FLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

SRC += energy_monitor.c

.PHONY: firmware hostapp all

all: energy_monitor host_receiver

energy_monitor: energy_monitor.c Makefile
	$(CC) $(FLAGS) $(SRC) -o energy_monitor -lopencm3_stm32f4

host_receiver: host/host_receiver.cpp host/libusbinterface.cpp host/dataprocessor.cpp Makefile
	g++ -o host_receiver host/host_receiver.cpp host/libusbinterface.cpp host/dataprocessor.cpp -lboost_thread-mt -lusb-1.0 -g -O2 -std=c++11
	# clang -o host_receiver host/host_receiver.cpp host/libusbinterface.cpp host/dataprocessor.cpp -lboost_thread-mt -lusb-1.0 -g -O2
