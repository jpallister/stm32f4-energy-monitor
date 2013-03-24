CC=~/sat_toolchain/bin/arm-none-eabi-gcc

FLAGS += -T libopencm3_stm32f4.ld
FLAGS += -D STM32F4
FLAGS += -g -fno-common -save-temps
FLAGS += -mcpu=cortex-m4 -mthumb
FLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

SRC += energy_monitor.c

HCC=g++
HLFLAGS += -lboost_thread-mt -lusb-1.0 -lreadline -lboost_regex-mt
HCFLAGS += -g -std=c++11 -O3 -I /usr/include/python2.7 -fPIC
# HCFLAGS +=

HOST_SRC += host/host_receiver.o
HOST_SRC += host/libusbinterface.o
HOST_SRC += host/dataprocessor.o
HOST_SRC += host/helper.o

MHLFLAGS = $(HLFLAGS)
MHLFLAGS += -lboost_python -lpython2.7 -shared -fpic

MHOST_SRC = $(HOST_SRC)
MHOST_SRC += host/pymodule.o


.PHONY: firmware hostapp all

all: energy_monitor
	make -j pyenergy.so
	make -j host_receiver

energy_monitor: energy_monitor.c Makefile
	$(CC) $(FLAGS) $(SRC) -o energy_monitor -lopencm3_stm32f4

host_receiver: $(HOST_SRC) Makefile
	$(HCC) -o host_receiver $(HOST_SRC) $(HLFLAGS)

pyenergy.so: $(MHOST_SRC) Makefile
	$(HCC) $(MHOST_SRC) $(MHLFLAGS) -o pyenergy.so

%.o : %.cpp Makefile
	$(HCC) -c $(HCFLAGS) $< -o $@

clean:
	rm -f host_receiver host/*.o energy_monitor
