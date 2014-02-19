#!/usr/bin/python

"""Energy tools

Usage:
    energytool (-m MPOINT)... [options] read SERIAL PIN
    energytool (-m MPOINT)... [options] continuous SERIAL
    energytool (-m MPOINT)... [options] debug SERIAL
    energytool list
    energytool changeserial SERIAL NEWSERIAL

Commands:
    read            This sets up a trigger on the specified PIN and waits for
                    an energy measurement.

    continuous      Continuously read measurements from the specified energy
                    monitor.

    debug           Output some debug data about the instantaneous voltages
                    seen on the ADCs, along with current and voltage.

    list            Show the serial numbers and API version of each connected
                    energy monitor.

    changeserial    Connect to the device specified by SERIAL, and change the
                    serial to NEWSERIAL

Options:
    -m --measurement MPOINT     Specify a measurement point to use (up to 3)
                                can be specified.
    -t --time TIME              Seconds delay between measurements for
                                continuous and debug [default: 0.1]
"""
from docopt import docopt
import pyenergy
from time import sleep
import usb.util

def prettyPrint(v):
    units = ['', 'm', 'u', 'n', 'p']

    for unit in units:
        if v > 1.0:
            return "{: >8.3f} {}".format(v, unit)
        v *= 1000.
    return "{}".format(v)

def display(m):
    print "Energy:          {}J".format(prettyPrint(m.energy))
    print "Time:            {}s".format(prettyPrint(m.time))
    print "Power:           {}W".format(prettyPrint(m.avg_power))
    print "Average current: {}A".format(prettyPrint(m.avg_current))
    print "Average voltage: {}V".format(prettyPrint(m.avg_voltage))


def read(serial, pin, mpoints):
    em = pyenergy.EnergyMonitor(serial)
    em.connect()

    for mp in mpoints:
        em.enableMeasurementPoint(mp)
        em.setTrigger(pin, mp)

    while not em.measurementCompleted():
        sleep(0.1)

    for mp in mpoints:
        m = em.getMeasurement(mp)
        print "Measurement point",mp
        display(m)
        print ""

def continuous(serial, mpoints, delay=0.1):
    em = pyenergy.EnergyMonitor(serial)
    em.connect()

    print "time,",

    for mp in mpoints:
        em.enableMeasurementPoint(mp)
        em.start(mp)
        print "energy_{0}, power_{0}, avg_current_{0}, avg_voltage_{0},".format(mp),
    print ""

    while True:
        first = True
        for mp in mpoints:
            m = em.getMeasurement(mp)

            if first:
                print m.time,
            print "{}, {}, {}, {},".format(m.energy, m.energy/m.time, m.avg_current, m.avg_voltage),
        print ""
        sleep(delay)


def debug(serial, mpoints, delay=0.1):
    em = pyenergy.EnergyMonitor(serial)
    em.connect()

    for mp in mpoints:
        em.enableMeasurementPoint(mp)
        em.start(mp)

    while True:
        for mp in mpoints:
            meas = em.getInstantaneous(mp)
            print "Measurement point:", mp
            em.debugInstantaneous(meas)
        print ""
        sleep(delay)

def list_boards():
    devs = pyenergy.EnergyMonitor.getBoards()

    print "Connected energy monitors:"

    if len(devs) > 0:
        print "    Serial  API"
    else:
        print "    None :("

    for d in devs:
        d.set_configuration()
        v = pyenergy.EnergyMonitor.getVersion(d)

        if v < pyenergy.EnergyMonitor.baseVersion:
            serial = "{} (?)".format(usb.util.get_string(d, 256, 3))
        else:
            serial = pyenergy.EnergyMonitor.getSerial(d)

        print "    {: <8} {}".format(serial, v)

def change_serial(oldser, newser):
    em = pyenergy.EnergyMonitor(oldser)
    em.setSerial(newser)


def main():
    arguments = docopt(__doc__)

    mpoints = map(int, set(arguments['--measurement']))

    if arguments['read']:
        read(arguments['SERIAL'], arguments['PIN'], mpoints)
    elif arguments['debug']:
        debug(arguments['SERIAL'], mpoints, float(arguments['--time']))
    elif arguments['continuous']:
        continuous(arguments['SERIAL'], mpoints, float(arguments['--time']))
    elif arguments['list']:
        list_boards()
    elif arguments['changeserial']:
        change_serial(arguments['SERIAL'], arguments['NEWSERIAL'])

if __name__=="__main__":
    main()
