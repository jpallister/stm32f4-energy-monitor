STM32F4DISCOVERY Energy Monitor
===============================

This repository contains firmware code for the STM32DISCOVERY, and host code for a linux PC. The firmware reads the onboard ADCs, compresses and streams the data back to the host via USB. The host application uses libusb to interact with the device, setting up GPIO pins to trigger measurements and different modes the ADCs can be in.


Compilation
-----------

Requirements:

* libopencm3
* summon arm toolchain (Makefile expects it at ~/sat_toolchain)
* [stlink](https://github.com/texane/stlink) (to program the device)
* Boost (thread and regex)
* LibUSB 1.0
* libreadline


To build:

    $ make

The resulting file can then be flashed to the device using st-utils and gdb.


Usage
-----

### Firmware

The firmware is configured to read the PA1 pin as current (voltage drop across a shunt resistor inserted in the target's power supply), and PA2 as voltage (divided by two, from the target's power supply).

### Host

The host application presents a shell that can connect to the device and start and stop measurement.

    > connect
        Connected to device, serial: MSEM0000
    > start
    > stop

There are various other commands, which can be view by typing help:

    > help
        connect                     Connect to the device (if only one attached)
                 [SERIAL]           Select the device by serial number
        getserial                   Get the serial number of current device
        setserial SERIAL            Set the serial number of current device
                                    Reconnection required before new serial recognised
        leds                        Toggle the LEDs
        start                       Start energy measurement
        stop                        Stop energy measurement
        trigger
                 PIN                Trigger measurement on PIN (e.g PA0)
                 none               Remove trigger
        mode
             normal                 Normal ADC mode
             dual                   Dual ADC, one voltage, one current, multiplied on board
             oversampled            Run ADC as fast as possible, average samples
        exit                        Exit the application

Currently mode is a work in progress.

Trigger allow the energy measurement to be triggered by an external pin:

    > trigger PA0

This will set the measurement to start on the first rising edge of PA0, and stop on the next rising edge (this pin is also connected to the blue push-button on the board).

The following line may have to be added to a udev rule (energy monitor appears as device id 0xF539:0xF539):

    ATTRS{idVendor}=="0539", ATTRS{idProduct}=="0539", MODE="0666", GROUP="plugdev"

How it works
------------

### ADC Triggering

The firmware uses the TIM2 to periodically trigger the ADC. The ADC then sends its samples to a buffer using DMA. Once the DMA buffer is full, the transfer complete interrupt is generated. This interrupt encodes the buffer into a mode compact format (2 x 12bit samples into 3 bytes), and stores it in a circular buffer. It also multiplies the voltage by the current and stores that result instead of storing both.

This interrupt also looks at the amount of space left in the circular buffer, and adjusts the period of TIM2 accordingly. This allows the device to scale up the ADC to match the transfer speed of the USB. The discovery board only supports HS USB, so this becomes the main limit factor.


### USB

The main loop of the firmware polls the USB device, and when it sees unsend data in the circular buffer it sends it via USB. This loop also sends interrupt transfers when necessary (see below).


### GPIO triggering

The EXTI interrupts are used to start and stop measurements. They also include a small amount of debouncing. This sets a signal, and the main loop sends an interrupt back to the host, to let it know it should be ready to receive measurement data.


### ADC modes

Currently the ADC supports 2 modes: regular and dual. Regular mode uses ADC1 to scan both the voltage channel and current channel. The voltage channel is scanned once every 15 samples. This was chosen because the voltage doesn't change as frequently as the current. This gives a total of 42 current measurements and 3 voltage measurements, which can be encoded in 63 bytes, leaving one byte for the time period.


### Host

The host application consists of 3 threads:

* Main thread - shell and command processing.
* USB thread - send and receive usb commands, and queue data.
* Data processing - decode the data, compute statistics and save data to file

The USB thread buffers all data in a queue so it can continue waiting for USB events.


Todo list
---------

Firmware:

* Support measuring two devices at once
* Use an external ADC over SPI
* Properly implement ADC mode switching

Host:

* Select output trace file
* Output statistics on stop command

Bugs
----

* Setting a trigger while currently measuring doesn't work correctly, and breaks something with the ADC, making it return incorrect results.
* Device returns a string of 0 measurements before the first data.
* Odd crashes sometime happen when the host disconnects unexpectedly.
