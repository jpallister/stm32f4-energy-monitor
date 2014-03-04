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
                        stm32vldiscovery
                        atmega328p
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

import pexpect, sys

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

def gdb_launch(gdbname, port, fname, run_timeout=10, post_commands=[]):
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

def background_proc(cmd):

    def run_proc(ev):
        info("Starting background proc: \"{}\"".format(cmd))

        proclogger = logger.getChild(os.path.split(cmd.split(' ')[0])[-1])
        proc = pexpect.spawn(cmd, logfile=LogWriter(proclogger, logging.DEBUG))

        while not ev.isSet():
            proc.expect([r'.*\n', pexpect.EOF])

        info("Killing background proc: \"{}\"".format(cmd))
        proc.kill(15)

    ev = threading.Event()
    t = threading.Thread(target=run_proc, args=(ev,))
    t.start()

    return ev

def kill_background_proc(p):
    p.set()

def foreground_proc(cmd, expected_returncode=0):
    info("Starting foreground proc: \"{}\"".format(cmd))

    proclogger = logger.getChild(cmd.split(' ')[0])

    proc = pexpect.spawn(cmd, logfile=LogWriter(proclogger, logging.DEBUG))

    proc.expect(pexpect.EOF)

    if expected_returncode is not None and proc.exitstatus != expected_returncode:
        raise CommandError("Command \"{}\" returned {}".format())


def setupMeasurement(platform):
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

def loadConfiguration(fname="~/.measurementrc"):
    global measurement_config

    fname = os.path.expanduser(fname)
    measurement_config = json.load(open(fname))

def loadToolConfiguration(fname="~/.platformrunrc"):
    global tool_config

    fname = os.path.expanduser(fname)
    tool_config = json.load(open(fname))

#######################################################################

def stm32f0discovery(fname):
    em = setupMeasurement("stm32f0discovery")

    stproc = background_proc(tool_config['tools']['stutil'] + " -s 2 -p 2001 -c 0x0bb11477 -v0")
    gdb_launch(tool_config['tools']['arm_gdb'], 2001, fname)
    kill_background_proc(stproc)

    return finishMeasurement("stm32f0discovery", em)


def stm32vldiscovery(fname):
    em = setupMeasurement("stm32vldiscovery")

    stproc = background_proc(tool_config['tools']['stutil'] + " -s 1 -p 2002 -c 0x1ba01477 -v0")
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


def sam4lxplained(fname):
    em = setupMeasurement("sam4lxplained")

    openocdproc = background_proc(tool_config['tools']['openocd'] + " -f board/atmel_sam4l8_xplained_pro.cfg")
    gdb_launch(tool_config['tools']['arm_gdb'], 3333, fname, post_commands=["monitor shutdown"])
    kill_background_proc(openocdproc)

    return finishMeasurement("sam4lxplained", em)


def run(platformname, execname):

    if not os.path.exists(execname):
        raise IOError("File \"{}\" does not exist".format(execname))

    if platformname == "stm32f0discovery":
        m = stm32f0discovery(execname)
    elif platformname == "stm32vldiscovery":
        m = stm32vldiscovery(execname)
    elif platformname == "atmega328p":
        m = atmega328p(execname)
    elif platformname == "pic32mx250f128b":
        m = pic32mx250f128b(execname)
    elif platformname == "msp-exp430f5529":
        m = mspexp430f5529(execname)
    elif platformname == "msp-exp430fr5739":
        m = mspexp430fr5739(execname)
    elif platformname == "sam4lxplained":
        m = sam4lxplained(execname)
    else:
        raise RuntimeError("Unknown platform " + platformname)
    return m

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
        m = run(arguments['PLATFORM'], arguments['EXECUTABLE'])
    except (IOError, RuntimeError) as e:
        print "Error:",e
        quit(1)

    print "Energy:          {}J".format(prettyPrint(m.energy))
    print "Time:            {}s".format(prettyPrint(m.time))
    print "Power:           {}W".format(prettyPrint(m.avg_power))
    print "Average current: {}A".format(prettyPrint(m.avg_current))
    print "Average voltage: {}V".format(prettyPrint(m.avg_voltage))

if __name__ == "__main__":
    main()
