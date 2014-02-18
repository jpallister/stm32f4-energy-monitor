#!/usr/bin/python
"""Run an executable on a platform

Usage:
    platformrun [-v | -vv] [options] PLATFORM EXECUTABLE
    platformrun -h

Options:
    -h --help           Show this usage message
    -c --config CONF    Specify the measurement configuration to load
                            [default: ~/.measurementrc]
    -t --tools CONF     Config file for the tools needed to run on a platform
                            [default: ~/.platformrunrc]
    -v --verbose        Be verbose

    PLATFORM        Specify the platform on which to run.
                    Available platforms are:
                        stm32f0discovery
                        atmega328p
                        msp-exp430f5529


"""
from docopt import docopt

import subprocess, os, os.path
import threading
from collections import namedtuple
import tempfile
import json

import pyenergy
from time import sleep
import logging

logger = logging.getLogger(__name__)
warning = logger.warning
debug = logger.debug


tool_config = None
measurement_config = None

#######################################################################

def gdb_launch(gdbname, port, fname):
    if logger.getEffectiveLevel() == logging.DEBUG:
        silence = "-batch"
    else:
        silence = "-batch-silent"

    cmdline = '{gdbname} {silence} -ex "set confirm off" \
                  -ex "tar ext :{port}" \
                  -ex "monitor reset halt" \
                  -ex "load" \
                  -ex "delete breakpoints" \
                  -ex "break exit" \
                  -ex "break _exit" \
                  -ex "continue" \
                  -ex "quit" \
                  {fname}'.format(**locals())
    os.system(cmdline)

def background_proc(cmd, stdout=None, stderr=None):

    def run_proc(ev):
        debug("Starting background proc: \"{}\"".format(cmd))
        if logger.getEffectiveLevel() == logging.DEBUG:
            p = subprocess.Popen(cmd, shell=True)
        else:
            p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        while not ev.isSet():
            if logger.getEffectiveLevel() == logging.DEBUG:
                out = p.stdout.read(1)
                err = p.stderr.read(1)
            else:
                out = err = ''
            if (out == '' or err == '') and p.poll() != None:
                break
            ev.wait(0.1)
        debug("Killing background proc: \"{}\"".format(cmd))
        p.kill()

    ev = threading.Event()
    t = threading.Thread(target=run_proc, args=(ev,))
    t.start()

    return ev

def kill_background_proc(p):
    p.set()

def foreground_proc(cmd):
    debug("Starting foreground proc: \"{}\"".format(cmd))
    if logger.getEffectiveLevel() == logging.DEBUG:
        p = subprocess.Popen(cmd, shell=True)
    else:
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    while True:
        if logger.getEffectiveLevel() == logging.DEBUG and p.stdout:
            out = p.stdout.read(1)
        else:
            out = ''

        if logger.getEffectiveLevel() == logging.DEBUG and p.stderr:
            err = p.stderr.read(1)
        else:
            err =''
        if (out == '' or err == '') and p.poll() != None:
            break


def setupMeasurement(platform):
    # First check that all tools for platform are available
    if platform not in tool_config['enabled']:
        raise RuntimeError("Platform {0} does not have all the tools configured. Please rerun configure with --enable-{0}".format(platform))

    em = pyenergy.EnergyMonitor(measurement_config[platform]['energy-monitor'])
    mp = int(measurement_config[platform]['measurement-point'])

    em.connect()
    em.enableMeasurementPoint(mp)
    em.clearNumberOfRuns(mp)
    em.measurement_params[mp]['resistor'] = int(measurement_config[platform]['resistor'])
    em.setTrigger(measurement_config[platform]['trigger-pin'], mp)

    return em

def finishMeasurement(platform, em):
    mp = int(measurement_config[platform]['measurement-point'])

    while not em.measurementCompleted(mp):
        sleep(0.1)
    m = em.getMeasurement(mp)

    em.disconnect()
    return m

# Display units nicer
def prettyPrint(v):
    units = ['', 'm', 'u', 'n', 'p']

    for unit in units:
        if v > 1.0:
            return "{: >8.3f} {}".format(v, unit)
        v *= 1000.
    return "{}".format(v)

#######################################################################

def loadConfiguration(fname):
    global measurement_config

    fname = os.path.expanduser(fname)
    measurement_config = json.load(open(fname))

def loadToolConfiguration(fname):
    global tool_config

    fname = os.path.expanduser(fname)
    tool_config = json.load(open(fname))

#######################################################################

def stm32f0discovery(fname):
    em = setupMeasurement("stm32f0discovery")

    stproc = background_proc(tool_config['tools']['stutil'] + " -p 2001 -c 0x0bb11477 -v0")
    gdb_launch(tool_config['tools']['arm_gdb'], 2001, fname)
    kill_background_proc(stproc)

    return finishMeasurement("stm32f0discovery", em)


def stm32vldiscovery(fname):
    em = setupMeasurement("stm32vldiscovery")

    stproc = background_proc(tool_config['tools']['stutil'] + " -p 2002 -c 0x1ba01477 -v0")
    gdb_launch(tool_config['tools']['arm_gdb'], 2002, fname)
    kill_background_proc(stproc)

    return finishMeasurement("stm32vldiscovery", em)


def atmega328p(fname):
    em = setupMeasurement("atmega328p")

    # Create temporary file and convert to hex file
    tf = tempfile.NamedTemporaryFile(delete=False)
    tf.close()
    foreground_proc("{} -O ihex {} {}".format(tool_config['tools']['avr_objcopy'], fname, tf.name))

    # Flash the hex file to the AVR chip
    ser_id = measurement_config['atmega328p']['serial-dev-id']
    cmdline = "{} -F -V -c arduino -p atmega328p -e -P `readlink -m /dev/serial/by-id/{}` -b 115200 -U flash:w:{}".format(tool_config['tools']['avrdude'], ser_id, tf.name)
    foreground_proc(cmdline)

    os.unlink(tf.name)
    return finishMeasurement("atmega328p", em)


def mspexp430f5529(fname):
    em = setupMeasurement("msp-exp430f5529")

    foreground_proc("{} tilib -q \"prog {}\" &".format(tool_config['tools']['mspdebug'], fname))

    return finishMeasurement("msp-exp430f5529", em)


def mspexp430fr5739(fname):
    em = setupMeasurement("msp-exp430fr5739")

    foreground_proc("{} rf2500 -q \"prog {}\" &".format(tool_config['tools']['mspdebug'], fname))

    return finishMeasurement("msp-exp430fr5739", em)


def pic32mx250f128b(fname):
    # Create temporary file and convert to hex file
    tf = tempfile.NamedTemporaryFile(delete=False)
    tf.close()
    foreground_proc("{} -O ihex {} {}".format(tool_config['tools']['pic32_objcopy'], fname, tf.name))

    # Program the PIC and leave power on to run test
    em = setupMeasurement("pic32mx250f128b")
    foreground_proc("{} -p {}".format(tool_config['tools']['pic32prog'], tf.name))

    os.unlink(tf.name)
    return finishMeasurement("pic32mx250f128b", em)


def main():
    arguments = docopt(__doc__)

    logging.basicConfig()

    if arguments['--verbose'] == 1:
        logging.getLogger('').setLevel(logging.INFO)
    elif arguments['--verbose']== 2:
        logging.getLogger('').setLevel(logging.DEBUG)

    loadConfiguration(arguments['--config'])
    loadToolConfiguration(arguments['--tools'])

    if arguments['PLATFORM'] == "stm32f0discovery":
        m = stm32f0discovery(arguments['EXECUTABLE'])
    elif arguments['PLATFORM'] == "stm32vldiscovery":
        m = stm32vldiscovery(arguments['EXECUTABLE'])
    elif arguments['PLATFORM'] == "atmega328p":
        m = atmega328p(arguments['EXECUTABLE'])
    elif arguments['PLATFORM'] == "pic32mx250f128b":
        m = pic32mx250f128b(arguments['EXECUTABLE'])
    elif arguments['PLATFORM'] == "msp-exp430f5529":
        m = mspexp430f5529(arguments['EXECUTABLE'])
    elif arguments['PLATFORM'] == "msp-exp430fr5739":
        m = mspexp430fr5739(arguments['EXECUTABLE'])
    else:
        raise RuntimeError("Unknown platform " + arguments['PLATFORM'])

    print "Energy:          {}J".format(prettyPrint(m.energy))
    print "Time:            {}s".format(prettyPrint(m.time))
#    print "Power:           {}W".format(prettyPrint(m.avg_power))
    print "Average current: {}A".format(prettyPrint(m.avg_current))
    print "Average voltage: {}V".format(prettyPrint(m.avg_voltage))

if __name__ == "__main__":
    main()
