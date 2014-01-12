#!/usr/bin/python

# Output continuous data in CSV format

import pyenergy
from time import sleep

em = pyenergy.EnergyMonitor("EE00")
em.connect()

em.enableMeasurementPoint(1)
em.start()

print "energy, time, power, peak_power, peak_current, peak_voltage"
while True:
    m = em.getMeasurement()

    print "{}, {}, {}, {}, {}, {}".format(m.energy, m.time, m.energy/m.time, m.peak_power, m.avg_current, m.avg_voltage)
    sleep(0.1)
