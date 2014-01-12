#!/usr/bin/python

# Display each measurement, when triggered on PA0 (blue button)

import pyenergy
from time import sleep

# Display units nicer
def prettyPrint(v):
    units = ['', 'm', 'u', 'n', 'p']

    for unit in units:
        if v > 1.0:
            return "{:3.3f} {}".format(v, unit)
        v *= 1000.
    return "{}".format(v)


# Find and connect to the board
em = pyenergy.EnergyMonitor("EE00")
em.connect()

# Enable measurement point 1 and set PA0 as the trigger (blue button)
em.enableMeasurementPoint(1)
em.setTrigger("PA0")

while True:
    # Wait for a measurement to complete
    while not em.measurementCompleted():
        sleep(0.1) # Need a delay, or we'll flood the board with USB requests

    m = em.getMeasurement()
    print "\nReceived measurement"

    if m.time == 0:
        print "\tError: zero time"
        continue

    print "\tEnergy:      {}J".format(prettyPrint(m.energy))
    print "\tTime:        {:3.3f} s".format(m.time)
    print "\tPower:       {}W   Peak power:  {}W".format(prettyPrint(m.energy/m.time), prettyPrint(m.peak_power))
    print "\tAvg current: {}A   Avg voltage: {}V".format(prettyPrint(m.avg_current), prettyPrint(m.avg_voltage))
