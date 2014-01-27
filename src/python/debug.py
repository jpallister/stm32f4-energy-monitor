#!/usr/bin/python

# Output continuous debug data

import pyenergy
from time import sleep
import sys

m_points = []

for arg in sys.argv[1:]:
    try:
        m = int(arg)
        if m < 1 or m > 4:
            print "Expected measurement point in range 1-4"
            continue
        m_points.append(m)
    except TypeError:
        print "Expected integer measurement point number, got:",arg

em = pyenergy.EnergyMonitor("EE00")
em.connect()

for m in m_points:
    em.enableMeasurementPoint(m)
    em.start(m)

while True:
    for m in m_points:
        meas = em.getInstantaneous(m)
        print "Measurement point:", m
        em.debugInstantaneous(meas)
    print ""
    sleep(0.1)
