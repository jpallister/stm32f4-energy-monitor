#!/usr/bin/python

# Output continuous debug data

import pyenergy
from time import sleep

em = pyenergy.EnergyMonitor("EE00")
em.connect()

em.enableMeasurementPoint(1)
em.start()

while True:
    m = em.getInstantaneous()
    em.debugInstantaneous(m)
    sleep(0.1)
