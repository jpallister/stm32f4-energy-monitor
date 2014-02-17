Platform run
============

These scripts form the basis to easily run binaries on a range of platforms.
Details of the platforms and how they're set up are given below.

The script itself accepts two arguments, a platform to run on and the name of
the executable file. The configure script can be used to find the locations of
various executables needed by the various platforms. By default the configure
script will not generate code for a specific platform if the prerequisites
cannot be found.

List of platforms
-----------------

### STM32F0DISCOVERY

This board contains a cortex-m0. Stlink is used as a gdb-server and an arm
debugger is used to flash the code to the board.

Platform name: stm32f0discovery

Prerequisites for running:
 - st-util (st-link package)
 - arm-none-eabi-gdb (or other arm gdb-compatible debugger)


### ATMEGA328P

This is an ATMEGA328P chip, loaded with an arduino compatible bootloader. This
is programmed with a USB to serial converter and avrdude. This was tested on a
breadboarded ATMEGA328P chip, with a FTDI USB to serial converter.

Platform name: atmega328p

Prerequisites for running:
 - avrdude
 - avr-objcopy


### PIC32MX250F128B

This is a 32-bit pic chip, tested on a breadboard and hooked up to a pickit2.
This requires pic32prog (so that no bootloader is needed on the PIC chip) and
the pic32 tools to convert the elf to a hex file.

Platform name: pic32mx250f128b

Prerequisites for running:
 - pic32prog
 - pic32-objcopy


Measurement configuration
-------------------------

The measurement configuration file tells platformrun about the energy monitors
connectted to the platforms, and some additional details about the platforms.
For example, to measure the stm32f0discovery, the application needs to know the
serial number of the energy measurement device, as well as the measurement
point and the shunt resistor value. Other platforms have additional details,
such as atmega328p, which needs to know the ID of the USB-serial adaptor.

The measurement config is a standard JSON format file.

### Basic configuration

Keys:
 - energy-monitor. This specifies the serial number of the energy monitor that
   is used for this platform.
 - trigger-pin.  This specifies the pin on which the platform will trigger the
   energy monitor.
 - measurement-point. Which measurement point is connected to the platform.
 - resistor. The value of the shunt resistor that intercept's the platform's
   power supply.
 
### Platform specific keys

Keys for atmega328p:
 - serial-dev. This specifies the ID of the USB to serial device. This ID comes
   directly from the link found in /dev/serial/by-id/ when the adapter is
   plugged in. By selecting the ID this way, multiple similar USB-serial
   adapters can be uniquely specified.

Keys for pic32mx250f128b:
 - serial-number. This specifies the serial number of the pickit2 connected to
   the platform (not currently used).

