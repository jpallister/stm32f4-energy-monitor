The stm32f4-energy-monitor
==========================

This repository contains support software for the MAGEEC energy monitor boards. The repository is divided into two projects - the firmware that runs on the stm32f4discovery board, and a python module that is used to control the board.

To quickly get started, program firmware/energy_monitor to the stm32f4discovery and connect via the micro-USB port to the computer. Then install the pyenergy module via pip:

    sudo pip install --pre pyenergy

The energytool and platformrun command are then available to utilise the board. See the [pyenergy](https://pypi.python.org/pypi/pyenergy/) page for more detail on the module.

### Firmware Building
--------------------------
The older version of the firmware has been modified, the firmware and installation processes in the [guide](http://mageec.org/wiki/Workshop) on mageec's workshop page uses old firmware and does not use `pyenergy` tool, to use the firmware/pyenergy use the following steps:
 - Clone the repositery using the command `git clone https://github.com/jpallister/stm32f4-energy-monitor.git`
 - `cd` into firmware directory, then run following commands to build `libopencm3`: 
    - `git clone https://github.com/libopencm3/libopencm3`
    - `cd libopencm3`
    - `git checkout 806ebb18faf3285bb4dfa3c9c2caeac77dca7f34`
    - `make`
    
- To build `energy_monitor` binary, run following commands in firmware directory:
    - `./configure --host=arm-none-eabi`
    - `make`
    - then follow same steps from the [workshop page](http://mageec.org/wiki/Workshop#Software_setup) from `st-link` building/running to loading the firmware on the power measurement shield.

The power measurement shield should be ready for use with the `pyenergy` command line tool. After installing the python module using the instructions at the top, you may test it out using the following command: `[sudo] energytool list` 

- *sudo might be optional here if you have setup the `udev` rules from the workshop guide*
- *it is important to note that the firmware that works with the `pyenergy` tool does not work with older python modules e.g. read.py*
