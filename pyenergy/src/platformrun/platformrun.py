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
    -s                  Split energy measurements when board using multiple
                        measurement points

    PLATFORM        Specify the platform on which to run.
                    Available platforms are:
                        stm32f0discovery
                        stm32l0discovery
                        stm32vldiscovery
                        stm32f4discovery
                        atmega328p
                        xmegaa3buxplained
                        msp-exp430f5529
                        msp-exp430fr5739
                        pic32mx250f128b
                        sam4lxplained

"""
from docopt import docopt

import os
import os.path
import threading
import tempfile
import json

import pyenergy
from time import sleep
import logging
import usb

import pexpect
import copy

logger = logging.getLogger(__name__)
warning = logger.warning
info = logger.info
debug = logger.debug


tool_config = None
measurement_config = None

#######################################################################


def killBgOnCtrlC(f):
    def wrap(instance, platform, *args, **kwargs):
        try:
            return f(instance, platform, *args, **kwargs)
        except:
            info("Exception occured, killing background procs")
            for p in copy.copy(instance.bg_procs):
                instance.kill_background_proc(p)
            raise
    return wrap


class CommandError(RuntimeError):
    pass


class LogWriter(object):
    def __init__(self, logger, level):
        self.logger = logger
        self.level = level
        self.current_msg = ""

    def write(self, msg):
        self.current_msg += msg

        if '\n' in self.current_msg:
            s = self.current_msg.split('\n')
            to_print = s[:-1]
            self.current_msg = s[-1]
            for s in to_print:
                self.logger.log(self.level, s)

    def flush(self):
        pass

    def close(self):
        pass


class PlatformRun:
    def gdb_launch(self, gdbname, port, fname, run_timeout=30,
                   post_commands=[], pre_commands=[]):
        info("Starting+connecting to gdb and loading file")

        gdblogger = logger.getChild(os.path.split(gdbname)[-1])
        gdb = pexpect.spawn(gdbname, logfile=LogWriter(gdblogger,
                                                       logging.DEBUG))

        gdb.expect(r".*\(gdb\) ")

        gdb.sendline("set confirm off")
        gdb.expect(r".*\(gdb\) ")

        gdb.sendline("target remote :{}".format(port))
        gdb.expect(r'Remote debugging using.*\n')
        gdb.expect(r'.*\n')
        gdb.expect(r".*\(gdb\) ")

        for cmd in pre_commands:
            gdb.sendline(cmd)
            gdb.expect(r'.*\(gdb\) ')

        info("load file {}".format(fname))
        gdb.sendline("file {}".format(fname))
        gdb.expect(r'Reading symbols.*\n')
        gdb.expect(r".*\(gdb\) ")

        gdb.sendline("load")
        while True:
            a = gdb.expect([r'Loading section.*\n', r'Start address.*\n',
                            r"Error finishing flash operation.*\n"])
            if a == 1:
                break
            if a == 2:
                gdb.expect(r".*\(gdb\) ")
                gdb.sendline("load")

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

    def __init__(self):
        self.bg_procs = []
        self.bg_proc_map = {}

    def background_proc(self, cmd, env=None):

        def run_proc(ev):
            info("Starting background proc: \"{}\"".format(cmd))

            proclogger = logger.getChild(os.path.split(cmd.split(' ')[0])[-1])
            proc = pexpect.spawn(cmd, logfile=LogWriter(proclogger,
                                                        logging.DEBUG),
                                 env=env)

            while not ev.isSet():
                proc.expect([r'.*\n', pexpect.EOF, pexpect.TIMEOUT], timeout=1)
                sleep(0.1)

            info("Killing background proc: \"{}\"".format(cmd))
            proc.kill(15)
            sleep(0.2)
            if proc.isalive():
                warning("Killing (sig 9) background proc: \"{}\"".format(cmd))
                proc.kill(9)

        ev = threading.Event()
        t = threading.Thread(target=run_proc, args=(ev,))
        t.start()

        self.bg_procs.append(ev)
        self.bg_proc_map[ev] = t

        return ev

    def kill_background_proc(self, p):
        p.set()
        info("Waiting for background process to exit " + str(p))
        self.bg_proc_map[p].join()
        sleep(0.1)
        self.bg_procs.remove(p)

    def foreground_proc(self, cmd, expected_returncode=0):
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

        if (expected_returncode is not None and
                proc.exitstatus != expected_returncode):
            raise CommandError(
                "Command \"{}\" returned {}".format(cmd, proc.exitstatus))
        info("Foreground proc complete")

    def setupMeasurement(platform, doMeasure=True):
        if not doMeasure:
            return None

        # First check that all tools for platform are available
        if platform not in tool_config['enabled']:
            raise RuntimeError(
                "Platform {0} does not have all the tools configured. "
                "Please rerun configure with --enable-{0}".format(platform))

        em = pyenergy.EnergyMonitor(
            measurement_config[platform]['energy-monitor'])

        mp_cfg = measurement_config[platform]['measurement-point']
        if type(mp_cfg) is int:
            mpoints = [mp_cfg]
            resistors = [measurement_config[platform]['resistor']]
        elif type(mp_cfg) is list:
            mpoints = mp_cfg
            resistors = measurement_config[platform]['resistor']
        # mp = int(measurement_config[platform]['measurement-point'])

        em.connect()
        for mp, resistor in zip(mpoints, resistors):
            em.enableMeasurementPoint(mp)
            em.clearNumberOfRuns(mp)
            em.measurement_params[mp]['resistor'] = float(resistor)
            em.setTrigger(measurement_config[platform]['trigger-pin'], mp)

        return em

    def finishMeasurement(platform, em, doMeasure=True, timeout=30):
        if not doMeasure:
            return None

        mp_cfg = measurement_config[platform]['measurement-point']
        if type(mp_cfg) is int:
            mpoints = [mp_cfg]
            # resistors = [measurement_config[platform]['resistor']]
        elif type(mp_cfg) is list:
            mpoints = mp_cfg
            # resistors = measurement_config[platform]['resistor']

        info("Waiting for measurement to complete")
        t = 0
        for mp in mpoints:
            while not em.measurementCompleted(mp):
                sleep(0.1)
                t += 1
                if t > timeout * 10:
                    raise CommandError("Measurement timeout")
        info("Measurement complete")

        if type(mp_cfg) is list:
            measurements = map(em.getMeasurement, mpoints)
            m = pyenergy.MeasurementSet(measurements)
        else:
            m = em.getMeasurement(mp)

        em.disconnect()
        return m

    # Display units nicer
    def prettyPrint(v):
        units = [' ', 'm', 'u', 'n', 'p']

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

    ##

    def findUSBLocation(devid):
        info("Discovering USB bus address of {}".format(devid))

        for bus in usb.busses():
            for dev in bus.devices:
                try:
                    print
                    if devid.lower() == "{:04x}:{:04x}".format(dev.idVendor,
                                                               dev.idProduct):
                        addr = "{:03}:{:03}".format(dev.dev.bus,
                                                    dev.dev.address)
                        info("Found {}".format(addr))
                        return addr
                except usb.core.USBError:
                    pass
        warning("Could not find device location for {}. "
                "The wrong board may be selected".format(addr))
        return ""

    #######################################################################
    @killBgOnCtrlC
    def stm32f0discovery(self, fname, doMeasure=True):
        em = self.setupMeasurement("stm32f0discovery", doMeasure)

        stproc = self.background_proc(tool_config['tools']['stutil'] +
                                      " -s 2 -p 2001 -c 0x0bb11477 -v0")
        self.gdb_launch(tool_config['tools']['arm_gdb'], 2001, fname)
        self.kill_background_proc(stproc)

        return self.finishMeasurement("stm32f0discovery", em, doMeasure)

    @killBgOnCtrlC
    def stm32vldiscovery(self, fname, doMeasure=True):
        em = self.setupMeasurement("stm32vldiscovery", doMeasure)

        stproc = self.background_proc(tool_config['tools']['stutil'] +
                                      " -s 1 -p 2002 -c 0x1ba01477 -v0")
        self.gdb_launch(tool_config['tools']['arm_gdb'], 2002, fname)
        self.ill_background_proc(stproc)

        return self.finishMeasurement("stm32vldiscovery", em, doMeasure)

    @killBgOnCtrlC
    def stm32f4discovery(self, fname, doMeasure=True):
        em = self.setupMeasurement("stm32f4discovery", doMeasure)

        stproc = self.background_proc(tool_config['tools']['stutil'] +
                                      " -s 2 -p 2003 -c 0x2ba01477 -v0")
        self.gdb_launch(tool_config['tools']['arm_gdb'], 2003, fname)
        self.kill_background_proc(stproc)

        return self.finishMeasurement("stm32f4discovery", em, doMeasure)

    @killBgOnCtrlC
    def stm32l0discovery(self, fname, doMeasure=True):
        em = self.setupMeasurement("stm32l0discovery", doMeasure)

        stproc = self.background_proc(tool_config['tools']['openocd'] +
                                      ' -f board/stm32l0discovery.cfg '
                                      '--command "gdb_port 2004"')
        self.gdb_launch(tool_config['tools']['arm_gdb'], 2004, fname,
                        post_commands=["monitor shutdown"],
                        pre_commands=["monitor reset init"])
        self.kill_background_proc(stproc)

        return self.finishMeasurement("stm32l0discovery", em, doMeasure)

    @killBgOnCtrlC
    def beaglebone(self, fname, doMeasure=True):
        em = self.setupMeasurement("beaglebone", doMeasure)

        openocdproc = self.background_proc(tool_config['tools']['openocd'] +
                                           " -f board/ti_beaglebone.cfg")
        self.gdb_launch(tool_config['tools']['arm_gdb'], 3333, fname,
                        post_commands=["monitor shutdown"],
                        pre_commands=["monitor reset halt"])
        self.kill_background_proc(openocdproc)

        return self.finishMeasurement("beaglebone", em, doMeasure)

    def atmega328p(self, fname, doMeasure=True):
        # Create temporary file and convert to hex file
        tf = tempfile.NamedTemporaryFile(delete=False)
        tf.close()
        self.foreground_proc(
            "{} -O ihex {} {}".format(tool_config['tools']['avr_objcopy'],
                                      fname, tf.name))

        # Flash the hex file to the AVR chip
        """if logger.getEffectiveLevel() == logging.DEBUG:
            silence = ""
        elif logger.getEffectiveLevel() == logging.INFO:
            silence = "-q"
        else:
            silence = "-q -q" """

        ser_id = measurement_config['atmega328p']['serial-dev-id']
        ser_path = "/dev/serial/by-id/{}".format(ser_id)

        if not os.path.exists(ser_path):
            raise RuntimeError("Cannot find the serial device: "+ser_path)

        cmdline = "readlink -m " + ser_path
        location = pexpect.run(cmdline).strip()
        info("AVR programmer @ {}".format(location))

        # info("Perform erase of the chip")
        # cmdline = "{} -F -c arduino -p atmega328p -e -P {} -b 115200".format(
        # tool_config['tools']['avrdude'], location)
        # self.foreground_proc(cmdline)

        em = self.setupMeasurement("atmega328p", doMeasure)

        cmdline = ("{} -F -V -c arduino -p atmega328p -P {} -b 115200"
                   "-U flash:w:{}").format(
                       tool_config['tools']['avrdude'], location, tf.name)
        self.foreground_proc(cmdline)

        try:
            os.unlink(tf.name)
        except OSError:
            pass
        return self.finishMeasurement("atmega328p", em, doMeasure)

    def xmegaa3buxplained(self, fname, doMeasure=True):
        em = self.setupMeasurement("xmegaa3buxplained", doMeasure)

        # Create temporary file and convert to hex file
        tf = tempfile.NamedTemporaryFile(delete=False)
        tf.close()
        self.foreground_proc("{} -O ihex {} {}".format(
            tool_config['tools']['avr_objcopy'], fname, tf.name))

        # Flash the hex file to the AVR chip
        """if logger.getEffectiveLevel() == logging.DEBUG:
            silence = ""
        elif logger.getEffectiveLevel() == logging.INFO:
            silence = "-q"
        else:
            silence = "-q -q" """

        cmdline = "{} -F -V -c jtag3 -p x256a3bu -e -U flash:w:{}".format(
            tool_config['tools']['avrdude'], tf.name)
        self.foreground_proc(cmdline)

        try:
            os.unlink(tf.name)
        except OSError:
            pass
        return self.finishMeasurement("xmegaa3buxplained", em, doMeasure)

    def mspexp430f5529(self, fname, doMeasure=True):
        em = self.setupMeasurement("msp-exp430f5529", doMeasure)

        self.foreground_proc("{} tilib -q \"prog {}\" &".format(
            tool_config['tools']['mspdebug'], fname))

        return self.finishMeasurement("msp-exp430f5529", em, doMeasure)

    def mspexp430fr5739(self, fname, doMeasure=True):
        em = self.setupMeasurement("msp-exp430fr5739", doMeasure)

        self.foreground_proc("{} rf2500 \"prog {}\" &".format(
            tool_config['tools']['mspdebug'], fname), expected_returncode=None)

        return self.finishMeasurement("msp-exp430fr5739", em, doMeasure)

    def pic32mx250f128b(self, fname, doMeasure=True):
        # Create temporary file and convert to hex fil, doMeasuree
        tf = tempfile.NamedTemporaryFile(delete=False)
        tf.close()
        self.foreground_proc("{} -O ihex {} {}".format(
            tool_config['tools']['pic32_objcopy'], fname, tf.name))

        # Program the PIC and leave power on to run test
        em = self.setupMeasurement("pic32mx250f128b")
        self.foreground_proc(
            "{} -p {}".format(tool_config['tools']['pic32prog'], tf.name))

        os.unlink(tf.name)
        return self.finishMeasurement("pic32mx250f128b", em, doMeasure)

    @killBgOnCtrlC
    def sam4lxplained(self, fname, doMeasure=True):
        em = self.setupMeasurement("sam4lxplained", doMeasure)

        openocdproc = self.background_proc(
            tool_config['tools']['openocd'] +
            " -f board/atmel_sam4l8_xplained_pro.cfg")
        self.gdb_launch(tool_config['tools']['arm_gdb'], 3333, fname,
                        post_commands=["monitor shutdown"])
        self.kill_background_proc(openocdproc)

        return self.finishMeasurement("sam4lxplained", em, doMeasure)

    def run(self, platformname, execname, measurement=True):

        if not os.path.isfile(execname):
            raise IOError("File \"{}\" does not exist".format(execname))

        if platformname == "stm32f0discovery":
            m = self.stm32f0discovery(execname, measurement)
        elif platformname == "stm32vldiscovery":
            m = self.stm32vldiscovery(execname, measurement)
        elif platformname == "stm32f4discovery":
            m = self.stm32f4discovery(execname, measurement)
        elif platformname == "stm32l0discovery":
            m = self.stm32l0discovery(execname, measurement)
        elif platformname == "beaglebone":
            m = self.beaglebone(execname, measurement)
        elif platformname == "atmega328p":
            m = self.atmega328p(execname, measurement)
        elif platformname == "xmegaa3buxplained":
            m = self.xmegaa3buxplained(execname, measurement)
        elif platformname == "pic32mx250f128b":
            m = self.pic32mx250f128b(execname, measurement)
        elif platformname == "msp-exp430f5529":
            m = self.mspexp430f5529(execname, measurement)
        elif platformname == "msp-exp430fr5739":
            m = self.mspexp430fr5739(execname, measurement)
        elif platformname == "sam4lxplained":
            m = self.sam4lxplained(execname, measurement)
        else:
            raise RuntimeError("Unknown platform " + platformname)
        return m

    def run_multiple(self, platformname, execname, repeats, measurement=True):
        measurements = []

        for i in range(repeats):
            m = self.run(platformname, execname, measurement)
            measurements.append(m)

        return sum(measurements) / len(measurements)


def main():
    arguments = docopt(__doc__)

    logging.basicConfig()

    if arguments['--verbose'] == 1:
        logging.getLogger('').setLevel(logging.INFO)
    elif arguments['--verbose'] == 2:
        logging.getLogger('').setLevel(logging.DEBUG)

    PR = PlatformRun()

    PR.loadConfiguration(arguments['--config'])
    PR.loadToolConfiguration(arguments['--tools'])

    try:
        # Execute multiple repeats, if requested
        ms = []
        for i in range(int(arguments['--repeat'])):
            m = PR.run(arguments['PLATFORM'], arguments['EXECUTABLE'],
                       arguments['--no-measure'] is False)
            ms.append(m)

        m = sum(ms) / len(ms)
    except (IOError, RuntimeError) as e:
        print "Error:", e
        quit(1)

    if arguments['--no-measure'] is False:
        if arguments['--csv']:
            print ("{m.energy}, {m.time}, {m.avg_power}, {m.avg_current}, "
                   "{m.avg_voltage}").format(m=m)
        else:
            if arguments['-s'] and hasattr(m, "measurements"):

                s = "       "
                for i in range(len(m.measurements)):
                    s += "           {}".format(i+1)
                print s

                names = ["Energy", "Time", "Power", "Average current",
                         "Average voltage"]
                keys = ["energy", "time", "avg_power", "avg_current",
                        "avg_voltage"]
                units = ["J", "s", "W", "A", "V"]

                for name, key, unit in zip(names, keys, units):
                    print "{}:{}".format(name, " " * (15-len(name))),
                    for meas in m.measurements:
                        print "{}{}".format(PR.prettyPrint(meas.__dict__[key]),
                                            unit),
                    print

            else:
                print "Energy:          {}J".format(PR.prettyPrint(m.energy))
                print "Time:            {}s".format(PR.prettyPrint(m.time))
                print "Power:           {}W".format(
                    PR.prettyPrint(m.avg_power))
                print "Average current: {}A".format(
                    PR.prettyPrint(m.avg_current))
                print "Average voltage: {}V".format(
                    PR.prettyPrint(m.avg_voltage))

if __name__ == "__main__":
    main()
