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
    -r --repeat N       Repeat the test N times [default: 1]
    -v --verbose        Be verbose
    --no-measure        Don't measure anything
    --csv               Print as csv

    PLATFORM        Specify the platform on which to run.
                    Available platforms are:
                        stm32f0discovery
                        stm32vldiscovery
                        atmega328p
                        xmegaa3buxplained
                        msp-exp430f5529
                        msp-exp430fr5739
                        pic32mx250f128b
                        sam4lxplained

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

import pexpect, sys, copy

logger = logging.getLogger(__name__)
warning = logger.warning
info = logger.info
debug = logger.debug


tool_config = None
measurement_config = None

#######################################################################

class CommandError(RuntimeError):
    pass

class LogWriter(object):
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level
    def write(self, msg):
        for s in msg.split('\n'):
            self.logger.log(self.level, s)
    def flush(self):
        pass
    def close(self):
        pass

def gdb_launch(gdbname, port, fname, run_timeout=30, post_commands=[]):
    info("Starting+connecting to gdb and loading file")

    gdblogger = logger.getChild(os.path.split(gdbname)[-1])
    gdb = pexpect.spawn(gdbname, logfile=LogWriter(gdblogger, logging.DEBUG))

    gdb.expect(r".*\(gdb\) ")

    gdb.sendline("set confirm off")
    gdb.expect(r".*\(gdb\) ")

    gdb.sendline("target extended :{}".format(port))
    gdb.expect(r'Remote debugging using.*\n')
    gdb.expect(r'.*\n')
    gdb.expect(r".*\(gdb\) ")

    info("load file {}".format(fname))
    gdb.sendline("file {}".format(fname))
    gdb.expect(r'Reading symbols.*\n')
    gdb.expect(r".*\(gdb\) ")

    gdb.sendline("load")
    while gdb.expect([r'Loading section.*\n',r'Start address.*\n']) == 0:
        pass
    gdb.expect(r'Transfer rate.*\n')
    gdb.expect(r'\(gdb\) ')

    gdb.sendline("delete breakpoints")
    gdb.expect(r".*\(gdb\) ")

    gdb.sendline("break exit")
    gdb.expect(r'.*Breakpoint .*\n')
    gdb.expect(r".*\(gdb\) ")

    gdb.sendline("break _exit")
    gdb.expect(r'.*Breakpoint .*\n')
    gdb.expect(r".*\(gdb\) ")

    info("Sending continue to gdb")
    gdb.sendline("continue")
    gdb.expect(r'Continuing.*\n')
    gdb.expect(r'.*Breakpoint.*exit.*\n', timeout=run_timeout)

    info("Breakpoint hit")

    for cmd in post_commands:
        gdb.sendline(cmd)
        gdb.expect(r'.*\(gdb\) ')

    info("Quitting GDB")
    gdb.sendline("quit")

bg_procs = []

def background_proc(cmd):

    def run_proc(ev):
        info("Starting background proc: \"{}\"".format(cmd))

        proclogger = logger.getChild(os.path.split(cmd.split(' ')[0])[-1])
        proc = pexpect.spawn(cmd, logfile=LogWriter(proclogger, logging.DEBUG))

        while not ev.isSet():
            proc.expect([r'.*\n', pexpect.EOF, pexpect.TIMEOUT], timeout=1)

        info("Killing background proc: \"{}\"".format(cmd))
        proc.kill(15)

    ev = threading.Event()
    t = threading.Thread(target=run_proc, args=(ev,))
    t.start()

    bg_procs.append(ev)

    return ev

def kill_background_proc(p):
    p.set()
    bg_procs.remove(p)

def killBgOnCtrlC(f):
    def wrap(platform, measurement):
        try:
            return f(platform, measurement)
        except:
            info("Keyboard interrupt, killing background procs")
            for p in copy.copy(bg_procs):
                kill_background_proc(p)
            raise
    return wrap


def foreground_proc(cmd, expected_returncode=0):
    info("Starting foreground proc: \"{}\"".format(cmd))

    proclogger = logger.getChild(cmd.split(' ')[0])

    proc = pexpect.spawn(cmd, logfile=LogWriter(proclogger, logging.DEBUG))

    proc.expect(pexpect.EOF)

    # Even though we have received an EOF, the process may not be
    # fully dead. Therefore, we wait. An exception is raised if the
    # process is already dead
    try:
        proc.wait()
    except pexpect.ExceptionPexpect:
        pass

    if expected_returncode is not None and proc.exitstatus != expected_returncode:
        raise CommandError("Command \"{}\" returned {}".format(cmd, proc.exitstatus))
    info("Foreground proc complete")


def setupMeasurement(platform, doMeasure=True):
    if not doMeasure:
        return None

    # First check that all tools for platform are available
    if platform not in tool_config['enabled']:
        raise RuntimeError("Platform {0} does not have all the tools configured. Please rerun configure with --enable-{0}".format(platform))

    em = pyenergy.EnergyMonitor(measurement_config[platform]['energy-monitor'])
    mp = int(measurement_config[platform]['measurement-point'])

    em.connect()
    em.enableMeasurementPoint(mp)
    em.clearNumberOfRuns(mp)
    em.measurement_params[mp]['resistor'] = float(measurement_config[platform]['resistor'])
    em.setTrigger(measurement_config[platform]['trigger-pin'], mp)

    return em

def finishMeasurement(platform, em, doMeasure=True):
    if not doMeasure:
        return None

    mp = int(measurement_config[platform]['measurement-point'])

    info("Waiting for measurement to complete")
    while not em.measurementCompleted(mp):
        sleep(0.1)
    info("Measurement complete")
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

def loadConfiguration(fname="~/.measurementrc"):
    global measurement_config

    fname = os.path.expanduser(fname)
    measurement_config = json.load(open(fname))

def loadToolConfiguration(fname="~/.platformrunrc"):
    global tool_config

    fname = os.path.expanduser(fname)
    tool_config = json.load(open(fname))

#######################################################################

@killBgOnCtrlC
def stm32f0discovery(fname, doMeasure=True):
    em = setupMeasurement("stm32f0discovery", doMeasure)

    stproc = background_proc(tool_config['tools']['stutil'] + " -s 2 -p 2001 -c 0x0bb11477 -v0")
    gdb_launch(tool_config['tools']['arm_gdb'], 2001, fname)
    kill_background_proc(stproc)

    return finishMeasurement("stm32f0discovery", em, doMeasure)


@killBgOnCtrlC
def stm32vldiscovery(fname, doMeasure=True):
    em = setupMeasurement("stm32vldiscovery", doMeasure)

    stproc = background_proc(tool_config['tools']['stutil'] + " -s 1 -p 2002 -c 0x1ba01477 -v0")
    gdb_launch(tool_config['tools']['arm_gdb'], 2002, fname)
    kill_background_proc(stproc)

    return finishMeasurement("stm32vldiscovery", em, doMeasure)


def atmega328p(fname, doMeasure=True):
    em = setupMeasurement("atmega328p", doMeasure)

    # Create temporary file and convert to hex file
    tf = tempfile.NamedTemporaryFile(delete=False)
    tf.close()
    foreground_proc("{} -O ihex {} {}".format(tool_config['tools']['avr_objcopy'], fname, tf.name))

    # Flash the hex file to the AVR chip
    if logger.getEffectiveLevel() == logging.DEBUG:
        silence = ""
    elif logger.getEffectiveLevel() == logging.INFO:
        silence = "-q"
    else:
        silence = "-q -q"

    ser_id = measurement_config['atmega328p']['serial-dev-id']
    cmdline = "{} -F -V -c arduino -p atmega328p -e -P `readlink -m /dev/serial/by-id/{}` -b 115200 -U flash:w:{}".format(tool_config['tools']['avrdude'], ser_id, tf.name)
    foreground_proc(cmdline)

    try:
        os.unlink(tf.name)
    except OSError:
        pass
    return finishMeasurement("atmega328p", em, doMeasure)

def xmegaa3buxplained(fname, doMeasure=True):
    em = setupMeasurement("xmegaa3buxplained", doMeasure)

    # Create temporary file and convert to hex file
    tf = tempfile.NamedTemporaryFile(delete=False)
    tf.close()
    foreground_proc("{} -O ihex {} {}".format(tool_config['tools']['avr_objcopy'], fname, tf.name))

    # Flash the hex file to the AVR chip
    if logger.getEffectiveLevel() == logging.DEBUG:
        silence = ""
    elif logger.getEffectiveLevel() == logging.INFO:
        silence = "-q"
    else:
        silence = "-q -q"

    cmdline = "{} -F -V -c jtag3 -p x256a3bu -e -U flash:w:{}".format(tool_config['tools']['avrdude'], tf.name)
    foreground_proc(cmdline)

    try:
        os.unlink(tf.name)
    except OSError:
        pass
    return finishMeasurement("xmegaa3buxplained", em, doMeasure)


def mspexp430f5529(fname, doMeasure=True):
    em = setupMeasurement("msp-exp430f5529", doMeasure)

    foreground_proc("{} tilib -q \"prog {}\" &".format(tool_config['tools']['mspdebug'], fname))

    return finishMeasurement("msp-exp430f5529", em, doMeasure)


def mspexp430fr5739(fname, doMeasure=True):
    em = setupMeasurement("msp-exp430fr5739", doMeasure)

    foreground_proc("{} rf2500 -q \"prog {}\" &".format(tool_config['tools']['mspdebug'], fname))

    return finishMeasurement("msp-exp430fr5739", em, doMeasure)


def pic32mx250f128b(fname, doMeasure=True):
    # Create temporary file and convert to hex fil, doMeasuree
    tf = tempfile.NamedTemporaryFile(delete=False)
    tf.close()
    foreground_proc("{} -O ihex {} {}".format(tool_config['tools']['pic32_objcopy'], fname, tf.name))

    # Program the PIC and leave power on to run test
    em = setupMeasurement("pic32mx250f128b")
    foreground_proc("{} -p {}".format(tool_config['tools']['pic32prog'], tf.name))

    os.unlink(tf.name)
    return finishMeasurement("pic32mx250f128b", em, doMeasure)


@killBgOnCtrlC
def sam4lxplained(fname, doMeasure=True):
    em = setupMeasurement("sam4lxplained", doMeasure)

    openocdproc = background_proc(tool_config['tools']['openocd'] + " -f board/atmel_sam4l8_xplained_pro.cfg")
    gdb_launch(tool_config['tools']['arm_gdb'], 3333, fname, post_commands=["monitor shutdown"])
    kill_background_proc(openocdproc)

    return finishMeasurement("sam4lxplained", em, doMeasure)


def run(platformname, execname, measurement=True):

    if not os.path.isfile(execname):
        raise IOError("File \"{}\" does not exist".format(execname))

    if platformname == "stm32f0discovery":
        m = stm32f0discovery(execname, measurement)
    elif platformname == "stm32vldiscovery":
        m = stm32vldiscovery(execname, measurement)
    elif platformname == "atmega328p":
        m = atmega328p(execname, measurement)
    elif platformname == "xmegaa3buxplained":
        m = xmegaa3buxplained(execname, measurement)
    elif platformname == "pic32mx250f128b":
        m = pic32mx250f128b(execname, measurement)
    elif platformname == "msp-exp430f5529":
        m = mspexp430f5529(execname, measurement)
    elif platformname == "msp-exp430fr5739":
        m = mspexp430fr5739(execname, measurement)
    elif platformname == "sam4lxplained":
        m = sam4lxplained(execname, measurement)
    else:
        raise RuntimeError("Unknown platform " + platformname)
    return m


def run_multiple(platformname, execname, repeats, measurement=True):
    measurements = []

    for i in range(repeats):
        m = run(platformname, execname, measurement)
        measurements.append(m)

    return sum(measurements) / len(measurements)

def main():
    arguments = docopt(__doc__)

    logging.basicConfig()

    if arguments['--verbose'] == 1:
        logging.getLogger('').setLevel(logging.INFO)
    elif arguments['--verbose']== 2:
        logging.getLogger('').setLevel(logging.DEBUG)

    loadConfiguration(arguments['--config'])
    loadToolConfiguration(arguments['--tools'])

    try:
        m = run(arguments['PLATFORM'],
                arguments['EXECUTABLE'],
                arguments['--no-measure'] is False)
    except (IOError, RuntimeError) as e:
        print "Error:",e
        quit(1)

    if arguments['--no-measure'] is False:
        if arguments['--csv']:
            print "{m.energy}, {m.time}, {m.avg_power}, {m.avg_current}, {m.avg_voltage}".format(m=m)
        else:
            print "Energy:          {}J".format(prettyPrint(m.energy))
            print "Time:            {}s".format(prettyPrint(m.time))
            print "Power:           {}W".format(prettyPrint(m.avg_power))
            print "Average current: {}A".format(prettyPrint(m.avg_current))
            print "Average voltage: {}V".format(prettyPrint(m.avg_voltage))

if __name__ == "__main__":
    main()
