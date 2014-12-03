#!/usr/bin/python

"""Energy tools

Usage:
    energytool (-m MPOINT)... [options] read SERIAL PIN
    energytool (-m MPOINT)... [options] continuous SERIAL
    energytool (-m MPOINT)... [options] debug SERIAL
    energytool list
    energytool setup
    energytool changeserial SERIAL NEWSERIAL
    energytool interactive SERIAL

Commands:
    read            This sets up a trigger on the specified PIN and waits for
                    an energy measurement.

    continuous      Continuously read measurements from the specified energy
                    monitor.

    debug           Output some debug data about the instantaneous voltages
                    seen on the ADCs, along with current and voltage.

    list            Show the serial numbers and API version of each connected
                    energy monitor.

    setup           Prompt about the energy measurement configuration and
                    create a .measurementrc file.

    changeserial    Connect to the device specified by SERIAL, and change the
                    serial to NEWSERIAL

    interactive     Start up the interactive graph view, for energy monitor
                    SERIAL. This requires PyQt4 to be installed.

Options:
    -m --measurement MPOINT     Specify a measurement point to use (up to 3)
                                can be specified.
    -t --time TIME              Seconds delay between measurements for
                                continuous and debug [default: 0.1]
    -v LEVEL                    Verbosity level: 0, 1 or 2 [default: 0]
"""
from docopt import docopt
import pyenergy
from time import sleep
import usb.util
import textwrap, string
import json, os, os.path
import platformrun
import readline

import logging

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
        em.clearNumberOfRuns(mp)

    for mp in mpoints:
        while not em.measurementCompleted(mp):
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

    try:
        while True:
            first = True
            for mp in mpoints:
                m = em.getMeasurement(mp)

                if first:
                    print m.time,
                print "{}, {}, {}, {},".format(m.energy, m.energy/m.time, m.avg_current, m.avg_voltage),
            print ""
            sleep(delay)
    except KeyboardInterrupt:
        for mp in mpoints:
            em.stop(mp)
            em.disableMeasurementPoint(mp)


def debug(serial, mpoints, delay=0.1):
    em = pyenergy.EnergyMonitor(serial)
    em.connect()

    for mp in mpoints:
        em.enableMeasurementPoint(mp)
        em.start(mp)

    try:
        while True:
            for mp in mpoints:
                meas = em.getInstantaneous(mp)
                print "Measurement point:", mp
                em.debugInstantaneous(meas)
            print ""
            sleep(delay)
    except KeyboardInterrupt:
        for mp in mpoints:
            em.stop(mp)
            em.disableMeasurementPoint(mp)


def list_boards():
    devs = pyenergy.EnergyMonitor.getBoards()

    print "Connected energy monitors:"

    if len(devs) > 0:
        print "    Serial  API"
    else:
        print "    None :("

    for d in devs:
        try:
            d.set_configuration()
            v = pyenergy.EnergyMonitor.getVersion(d)

            if v < pyenergy.EnergyMonitor.baseVersion or v < 13:
                serial = "{}(?)".format(usb.util.get_string(d, 3).encode("utf8","ignore"))
            else:
                serial = pyenergy.EnergyMonitor.getSerial(d)
        except usb.core.USBError:
            serial = "Error"
            v = "Error"

        if v < 13:
            v = "{} (Must update firmware before use!)".format(v)
        if v < pyenergy.EnergyMonitor.newestVersion:
            v = "{} (Please update firmware)".format(v)

        print "    {: <8} {}".format(serial, v)

def change_serial(oldser, newser):
    em = pyenergy.EnergyMonitor(oldser)
    em.setSerial(newser)


def preinput(text):
    def f():
        #stdout.write(blue)
        readline.insert_text(text)
        readline.redisplay()
    return f

def get_input(prompt, initial=""):
    readline.set_pre_input_hook(preinput(initial))
    inp = raw_input(prompt)
    # stdout.write(reset)
    return inp

def choice(prompt, choices, initial=""):
    sel = None

    while True:
        sel = get_input("{} [{}] ".format(prompt, "/".join(choices)), initial=initial)
        initial = ""

        if sel == '':
            sel = None
            for c in choices:
                if c.isupper():
                    return c.lower()
        elif sel.lower() in map(string.lower, choices):
            return sel.lower()

def setup():
    list_boards()
    print ""

    text=textwrap.dedent("""
        This program will prompt you for the connections between the measurement
        boards.Necessary information includes the serial of the energy monitor
        connected to which platform, and which pin is used to trigger the energy
        data collection.

        This information is stored in the ~/.measurementrc file. The existing
        config will now be loaded (if there is one), allowing you edit it.
        """)
    print text

    cfg = os.path.expanduser("~/.measurementrc")
    if os.path.exists(cfg):
        config = json.load(open(cfg))
    else:
        print "Configuration file does not exist, creating new config"
        config = {}


    while True:
        c = choice("[A]dd board, [e]dit/[delete] existing, [f]inished?", "aedf")

        if c == "f":
            break
        if c == 'a' or c == "e":
            if c == "a":
                all_platforms = set(platformrun.detect.default_config['platforms'].keys())
                used_platforms = set(config.keys())

                pchoices = list(all_platforms - used_platforms)
            else:
                pchoices = list(config.keys())

            pstrchoices = map(lambda (i, s): "\t[{}] {}".format(i, s), enumerate(pchoices))
            cchoices = "".join(map(str, range(len(pchoices))))

            c1 = choice("Select the platform:\n" + "\n".join(pstrchoices) + "\n ? ", cchoices)
            c1 = int(c1)

            platform = pchoices[c1]

            if c == "e":
                default_ser = config[platform]['energy-monitor']
                default_mp = str(config[platform]['measurement-point'])
                default_res = str(config[platform]['resistor'])
                default_pin = str(config[platform]['trigger-pin'])
            else:
                default_ser = ""
                default_mp = ""
                default_res = ""
                default_pin = ""

            ser = ""
            while ser == "":
                ser = get_input("Serial of energy monitor connected to {}: ".format(platform), initial=default_ser)

            if platform == "beaglebone":
                print "All measurement points needed for beagle bone, autoselecting following config:"
                print "    Measurement points: 1 (DDR), 2 (MPU), 3 (CORE)"
                print "    Resistors         : 0.5,     0.05,    0.05"
                mp = [1,2,3]
                res = [0.5, 0.05, 0.05]
            else:
                mp  = choice("Measurement point connected to {}".format(platform), ["1", "2", "3" ], initial=default_mp)
                res = choice("Shunt resistor used: ", ["0.05", "0.5", "1", "5"], initial = default_res)

                mp = int(mp)
                res = float(res)

            pin = ""
            while pin == "":
                pin = get_input("Set the trigger pin: ", initial=default_pin)

            config[platform] = {"energy-monitor": str(ser), "measurement-point": mp, "resistor": res, "trigger-pin": pin}

            if platform == "atmega328p":
                sdi = get_input("ATMEGA328P requires the id of the serial-USB adapter. This can be\nfound in /dev/serial/by-id/\n ? ")
                config[platform]['serial-dev-id'] = str(sdi)

            print ""
        if c == 'd':
            pchoices = list(config.keys())

            pstrchoices = map(lambda (i, s): "\t[{}] {}".format(i, s), enumerate(pchoices))
            cchoices = "".join(map(str, range(len(pchoices))))

            c1 = choice("Select the platform:\n" + "\n".join(pstrchoices) + "\n ? ", cchoices)
            c1 = int(c1)

            platform = pchoices[c1]
            del config[platform]


    f = open(cfg, "w")
    json.dump(config, f)
    f.close()


def main():
    arguments = docopt(__doc__)

    logging.basicConfig()
    if arguments['-v'] == "1":
        logging.getLogger('').setLevel(logging.INFO)
    if arguments['-v'] == "2":
        logging.getLogger('').setLevel(logging.DEBUG)
        logging.debug("test")

    mpoints = map(int, set(arguments['--measurement']))

    if arguments['read']:
        read(arguments['SERIAL'], arguments['PIN'], mpoints)
    elif arguments['debug']:
        debug(arguments['SERIAL'], mpoints, float(arguments['--time']))
    elif arguments['continuous']:
        continuous(arguments['SERIAL'], mpoints, float(arguments['--time']))
    elif arguments['list']:
        list_boards()
    elif arguments['setup']:
        setup()
    elif arguments['changeserial']:
        change_serial(arguments['SERIAL'], arguments['NEWSERIAL'])
    elif arguments['interactive']:
        try:
            import PyQt4
        except ImportError:
            print "Error: please install PyQt4"
            quit()
        import interactive_graph
        interactive_graph.main(arguments['SERIAL'])

if __name__=="__main__":
    main()
