The stm32f4-energy-monitor
==========================

This repository contains support software for the MAGEEC energy monitor boards. The repository is divided into two projects - the firmware that runs on the stm32f4discovery board, and a python module that is used to control the board.

To quickly get started, program firmware/energy_monitor to the stm32f4discovery and connect via the micro-USB port to the computer. Then install the pyenergy module via pip:

    sudo pip install --pre pyenergy

The energytool and platformrun command are then available to utilise the board. See the [pyenergy](https://pypi.python.org/pypi/pyenergy/) page for more detail on the module.
