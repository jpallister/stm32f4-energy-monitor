import usb
import usb.core
import usb.util

from struct import *
from copy import copy
from collections import namedtuple
import operator
from functools import total_ordering

import logging

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())
warning = logger.warning
error = logger.error
info = logger.info
debug = logger.debug


# import multiprocessing
@total_ordering
class MeasurementSet(object):
    def __init__(self, measurements):
        self.measurements = measurements

        self.energy       = sum(self.measurements).energy
        self.time         = sum(self.measurements).time / len(self.measurements)
        self.peak_power   = sum(self.measurements).peak_power  # This is only a rough guide
        self.peak_voltage = sum(self.measurements).peak_voltage
        self.peak_current = sum(self.measurements).peak_current
        self.n_samples    = sum(self.measurements).n_samples
        self.avg_voltage  = sum(self.measurements).avg_voltage
        self.avg_current  = sum(self.measurements).avg_current
        self.avg_power    = sum(self.measurements).avg_power

    def _op_self(self, other, fn):
        params = ["energy", "time", "peak_power", "peak_voltage", "peak_current",
                "n_samples", "avg_voltage", "avg_current", "avg_power"]

        newmeasurements = []
        for m1, m2 in zip(self.measurements, other.measurements):
            m = fn(m1, m2)
            newmeasurements.append(m)
        return MeasurementSet(newmeasurements)

    def _op_const(self, const, fn):
        params = ["energy", "time", "peak_power", "peak_voltage", "peak_current",
                "n_samples", "avg_voltage", "avg_current", "avg_power"]

        newmeasurements = []
        for m1 in self.measurements:
            m = fn(m1, const)
            newmeasurements.append(m)
        return MeasurementSet(newmeasurements)

    def _op(self, other, fn):
        if isinstance(other, MeasurementSet):
            assert len(self.measurements) == len(other.measurements)
            return self._op_self(other, fn)
        else:
            return self._op_const(other, fn);

    def __add__(self, other):
        return self._op(other, operator.add)
    def __sub__(self, other):
        return self._op(other, operator.sub)
    def __mul__(self, other):
        return self._op(other, operator.mul)
    def __div__(self, other):
        return self._op(other, operator.div)

    def __radd__(self, other):
        return self._op(other, operator.add)
    def __rsub__(self, other):
        return self._op(other, operator.sub)
    def __rmul__(self, other):
        return self._op(other, operator.mul)
    def __rdiv__(self, other):
        return self._op(other, operator.div)

    def __repr__(self):
        keys = ["energy", "time", "peak_power", "peak_voltage", "peak_current", "n_samples", "avg_voltage", "avg_current", "avg_power"]
        vals = ", ".join(map(lambda x: "{}={}".format(x,self.__dict__[x]), keys))
        return "MeasurementSet({} measurements, {})".format(len(self.measurements), vals)

    def __eq__(self, rhs):
        return all(map(operator.eq, self.measurements, rhs.measurements))

    def __lt__(self, rhs):
        return self.measurements < rhs.measurements

@total_ordering
class Measurement(object):
    class_version = 1

    def __init__(self, energy=0, time=0, peak_power=0, peak_voltage=0, peak_current=0,
                n_samples=0, avg_voltage=0, avg_current=0, avg_power=0):
        self.energy       = float(energy)
        self.time         = float(time)
        self.peak_power   = float(peak_power)
        self.peak_voltage = float(peak_voltage)
        self.peak_current = float(peak_current)
        self.n_samples    = float(n_samples)
        self.avg_voltage  = float(avg_voltage)
        self.avg_current  = float(avg_current)
        self.avg_power    = float(avg_power)
        self.version      = Measurement.class_version

    def _op_self(self, other, fn):
        params = ["energy", "time", "peak_power", "peak_voltage", "peak_current",
                "n_samples", "avg_voltage", "avg_current", "avg_power"]
        m = Measurement()
        for p in params:
            m.__dict__[p] = fn(self.__dict__[p], other.__dict__[p])
        return m

    def _op_const(self, const, fn):
        params = ["energy", "time", "peak_power", "peak_voltage", "peak_current",
                "n_samples", "avg_voltage", "avg_current", "avg_power"]
        m = Measurement()
        for p in params:
            m.__dict__[p] = fn(self.__dict__[p], const)
        return m

    def _op(self, other, fn):
        if isinstance(other, Measurement):
            return self._op_self(other, fn)
        else:
            return self._op_const(other, fn);

    def __add__(self, other):
        return self._op(other, operator.add)
    def __sub__(self, other):
        return self._op(other, operator.sub)
    def __mul__(self, other):
        return self._op(other, operator.mul)
    def __div__(self, other):
        return self._op(other, operator.div)

    def __radd__(self, other):
        return self._op(other, operator.add)
    def __rsub__(self, other):
        return self._op(other, operator.sub)
    def __rmul__(self, other):
        return self._op(other, operator.mul)
    def __rdiv__(self, other):
        return self._op(other, operator.div)

    def __repr__(self):
        return "Measurement(" + ", ".join(map("{0[0]}={0[1]}".format, self.__dict__.items())) + ")"

    def __eq__(self, rhs):
        params = ["energy", "time", "peak_power", "peak_voltage", "peak_current",
                 "n_samples", "avg_voltage", "avg_current", "avg_power"]

        for p in params:
            if self.__dict__[p] != rhs.__dict__[p]:
                return False

        return True

    def __lt__(self, rhs):
        params = ["energy", "time", "peak_power", "peak_voltage", "peak_current",
                 "n_samples", "avg_voltage", "avg_current", "avg_power"]
        s1 = (self.__dict__[p] for p in params)
        s2 = (rhs.__dict__[p] for p in params)

        return s1 < s2


    # Future proof for new version
    def __getstate__(self):
        return self.__dict__
    def __setstate__(self, d):

        if 'version' not in d or d['version'] < 1:
            # likely to only happen when we have unversioned data
            # so lets make this an info message instead of a warning
            # as we've probably warned about it already
            info("Upgrading to version 1")
            d['version'] = 1
            if "avg_power" not in d:
                d['avg_power'] = d['energy'] / d['time']

        # In future, if this class gets upgraded, add code here to upgrade from
        # previous versions

        if d['version'] > self.class_version:
            error("Measurement was created with a newer version of pyenergy, please upgrade")

        self.__dict__.update(d)

class EnergyMonitor(object):
    """
        The class implements the interface over USB to the MAGEEC enenergy
        monitor boards. This is fairly simple with pyusb, as all of the
        commands are implemented over the control endpoint.

        The class should be instantiated with a serial number, which tells
        it which board to connect to.

            em = EnergyMonitor("EE00")

        Before measuring, the measurement point needs to be enabled. This
        maps an ADC to the measurement point correctly.

            em.enableMeasurementPoint(1)

        After this, either a trigger can be set, or the measurement
        manually start and stopped.

            em.setTrigger("PA0", 1)

            em.start(1)
            em.stop(1)

        The measurement can then be retrieved.

            em.getMeasurement(1)
    """

    MeasurementData = namedtuple('MeasurementData', 'energy_accum elapsed_time peak_power peak_voltage peak_current n_samples avg_current avg_voltage')
    MeasurementData_packing = "=QQLLLLQQ"

    InstantaneousData = namedtuple('InstantaneousData', 'voltage current average_voltage average_current current_time')
    InstantaneousData_packing = "=LLLLQ"

    ADC1 = 0
    ADC2 = 1
    ADC3 = 2

    # These port mappings are the pins connected to voltage and current
    # (respectively) on the stm32f4discovery board.
    port_mappings = {1: ["PA2", "PC2"], 2: ["PA3", "PA1"], 3: ["PB1", "PC5"], 4:["PB0", "PC4"]}

    # The boards did not use to have a version associated with them, the first
    # version is version 10 (thus if we have an error getting the version we
    # assume the board is older than version 10).
    newestVersion = 13
    baseVersion = 10

    def __init__(self, serial="EE00"):
        devs = self.getBoards()

        sdevs = []
        for d in devs:
            try:
                d.set_configuration()
            except usb.core.USBError, e:
                warning("Could not access one of the boards")
                warning(str(e))
            v = self.getVersion(d)
            if v < EnergyMonitor.baseVersion:
                warning("Device attached with old firmware, cannot check if this is desired device")
                continue

            if v < 13:
                if usb.util.get_string(d, 3).encode("utf8","ignore") == serial:
                    error("You need to update the firmware on this board before you can use it")
                continue
            s = self.getSerial(d)

            if s == serial:
                sdevs.append(d)

            # TODO else release config

        devs = sdevs

        if len(devs) > 1:
            warning("More than one device ({}) available with serial {}".format(len(devs), serial))
        if len(devs) == 0:
            raise RuntimeError("Cannot find energy monitor with serial " + serial)

        self.serial = serial
        self.dev = devs[0]

        # Set up default parameters for each measurement point
        defaultparams = {'resistor':1, 'gain':50, 'vref':3}
        self.measurement_params = {i : copy(defaultparams) for i in [1,2,3]}

        # Measurement point 4 is the 'self' measurement point
        self.measurement_params[4] = {'resistor':0.5, 'gain':50, 'vref':3}

        # Equal to tperiod in the firmware
        self.samplePeriod = 500

        self.adcMpoint = [None, None, None]

    @staticmethod
    def getBoards():
        """Return a list of all energy monitor boards connected to the host"""

        devs = list(usb.core.find(idVendor=0xf539, idProduct=0xf539, find_all = True))
        return devs

    # Connect to the device
    def connect(self):
        """Connect to the specified board, and check firmware version"""
        info("Connecting to energy monitor")
        self.version = self.getVersion(self)

        if self.version < EnergyMonitor.baseVersion or self.version < 13:
            error("Firmware is too old, please update")
        if self.version < EnergyMonitor.newestVersion:
            warning("More recent firmware is available, please update")

    # Get version
    @staticmethod
    def getVersion(dev):
        """
            Get the API version of the energy monitoring boards. This may fail
            if the firmware is old and does not implement this request,
            therefore we catch this error.
        """
        try:
            dev = dev.dev
        except:
            pass

        try:
            b = dev.ctrl_transfer(0xc3, 12, 0, 0, 4)
        except usb.core.USBError as e:
            try:
                b = dev.ctrl_transfer(0xc1, 12, 0, 0, 4)
            except usb.core.USBError as e:
                if e.errno == 32:
                    return 0
                raise
        version = unpack("=L", b)[0]
        return version

    # Get serial
    @staticmethod
    def getSerial(dev):
        """Get the serial from the board."""
        try:
            dev = dev.dev
        except:
            pass

        b = dev.ctrl_transfer(0xc3, 13, 0, 0, 4)
        serial = unpack("=4s", b)[0]
        return serial

    # Toggle the LEDs on the device
    def toggleLEDs(self):
        """Toggle some of the LEDs on the board."""
        self.dev.ctrl_transfer(0x43, 0, 0, 0, None)

    # Start measuring on m_point
    def start(self, m_point=1):
        """Clear the number of runs and start measurement on the specified
        measurement point."""
        self.clearNumberOfRuns()
        self.dev.ctrl_transfer(0x43, 1, int(m_point), 0, None)

    # Stop measuring on m_point
    def stop(self, m_point=1):
        """Stop the measurement on the specified measurement point."""
        self.dev.ctrl_transfer(0x43, 2, int(m_point), 0, None)

    # Return whether the measurement point is currently taking
    # measurements or not
    def isRunning(self, m_point=1):
        """Check whether a particular measurement point is running."""
        debug("Check if running")
        b = self.dev.ctrl_transfer(0xc3, 8, int(m_point), 0, 4)
        debug("Received bytes: {0}".format(b))

        running = unpack("=L", b)
        debug("Running: {0}".format(running))
        return bool(running[0])

    # This counts the number of end measurement signals caught
    # by the energy monitor.
    def getNumberOfRuns(self, m_point=1):
        """
            Return the number of 'runs' seen by the energy monitor. This is
            the number of times a full measurement has been taken since the
            last clearNumberOfRuns call.

            This is mostly used as part of checking whether a triggered
            measurement has completed, in measurementComplete.
        """

        debug("Get number of runs")
        b = self.dev.ctrl_transfer(0xc3, 9, int(m_point), 0, 4)

        runs = unpack("=L", b)
        debug("Runs: {0}".format(runs[0]))
        return int(runs[0])

    # Reset the number of runs counts to 0
    def clearNumberOfRuns(self, m_point=1):
        """Clear the counter used to store the number of runs."""
        self.dev.ctrl_transfer(0x43, 10, int(m_point), 0, None)

    # Have we completed a measurement?
    def measurementCompleted(self, m_point = 1):
        """
            Check whether a full measurement has been taken. Also clear the
            number of runs if we are waiting for a trigger.
        """

        runs = self.getNumberOfRuns(m_point)
        if runs > 1:
            warning("More than one measurement has completed (expected one)")
        if not self.isRunning(m_point) and runs > 0:
            self.clearNumberOfRuns(m_point)
            return True
        return False

    # Set the serial number
    def setSerial(self, ser):
        """Set the board's serial number. Must be exactly 4 characters long."""

        if len(ser) != 4:
            warning("Serial should be 4 characters.")
            ser = (ser + "0000")[:4]
        self.dev.ctrl_transfer(0x43, 3, ord(ser[0]) | (ord(ser[1])<<8), ord(ser[2]) | (ord(ser[3])<<8), None)

    # Set a particular port as a pin trigger for a measurement point
    #   e.g PA0
    def setTrigger(self, port, m_point=1):
        """
            Set up a trigger on the specified port and measurement point. The
            board will automatically start measuring when the pin rises, and
            stop when the pin falls.

            The port should be of the form P[A-H][0-9]. Not all pins on the
            board will work, due to peripheral multiplexing and additional
            components attached to some pins.

            TODO: a list of acceptable and tested pins (PA0 works).
        """

        # TODO check port is of the form PA0
        info("Set trigger on {}".format(port))
        self.dev.ctrl_transfer(0x43, 4, ord(port[1]) | (m_point<<8), int(port[2]), None)

    # Enable a particular measurement point. There are
    # only 3 ADCs in the device, so only 3 measurement points
    # can be used simultaneously
    def enableMeasurementPoint(self, m_point, adc=None):
        """
            Enable a measurement point on the board.

            There are only 3 ADCs and 4 measurement points. Additionally
            certain ADCs can only measure certain measurement points,
            hence this function to map an ADC to a measurement point.

            This function records which measurement points are in use,
            the attempts to map an ADC to it, if any are free.
        """

        if m_point in self.adcMpoint:
            warning("Tried to enable already enabled measurement point "+str(m_point))
            return
        if self.adcMpoint.count(None) == 0:
            raise RuntimeError("Cannot enable measurement point {}. Maximum of enabled measurement points reached".format(m_point))
        if adc is not None and self.adcMpoint[adc] is not None:
            raise RuntimeError("Cannot enable map measurement point {0} to ADC{1}. ADC{1} already has measurement point {2}".format(m_point, adc, self.adcMpoint[adc]))

        if adc is None:
            # If we want mpoint 1 or 2 and ADC3 is free, prioritise it
            #  becuase ADC3 doesnt work with mpoint 3 or self
            if m_point in [1, 2] and self.adcMpoint[2] is None:
                adc = 2
            else:
                adc = self.adcMpoint.index(None)
        if m_point in [3,4] and adc == 2:
            raise RuntimeError("Measurement point {} cannot be used with ADC3 (the only free ADC)".format(m_point))
        info("Measurement point {} mapped to ADC {}".format(m_point, adc))
        self.adcMpoint[adc] = m_point
        self.dev.ctrl_transfer(0x43, 7, int(m_point), adc, None)

    # Disable a particular measurement point, so that the
    # ADC could potentially be used with a different one
    def disableMeasurementPoint(self, m_point):
        """
            To disable the measurement point we simply remove it from the set
            of mapped measurement points.

            We also send a stop command, to ensure no further measurements are
            taken.
        """

        info("Disable measurement point {}".format(m_point))

        if m_point not in self.adcMpoint:
            warning("Tried to disable already disabled measurement point "+str(m_point))
            return
        adc = self.adcMpoint.index(m_point)
        self.adcMpoint[adc] = None

        # TODO remove the trigger
        self.stop()

    def convertData(self, md, resistor, gain, vref, samplePeriod):
        """
            Convert the data from the raw form returned, and into standard
            units.
        """
        try:
            en = float(vref)**2 / gain / resistor / 4096**2 * 2 * samplePeriod * 2 / 168000000. * md.energy_accum * 2
            el = md.elapsed_time * 2. / 168000000 * 2
            pp = float(vref)**2 / gain / resistor / 4096**2 * md.peak_power * 2
            pv = float(vref) / 4096. * md.peak_voltage * 2
            pi = float(vref) / gain / resistor / 4096. * md.peak_current
            av = float(vref) / 4096. * md.avg_voltage / md.n_samples * 2
            ai = float(vref) / gain / resistor / 4096. * md.avg_current / md.n_samples
            ap = en/el
        except ZeroDivisionError:
            en, el, pp, pv, pi, av, ai, ap = 0, 0, 0, 0, 0, 0, 0, 0

        m = Measurement(en, el, pp, pv, pi, md.n_samples, av, ai, ap)

        return m

    def getMeasurement(self, m_point=1):
        """
            Get the most recent measurement taken for a particular measurement
            point.
        """

        info("getMeasurement")
        b = self.dev.ctrl_transfer(0xc3, 6, int(m_point), 0, calcsize(EnergyMonitor.MeasurementData_packing))
        u = EnergyMonitor.MeasurementData._make(unpack(EnergyMonitor.MeasurementData_packing, b))

        info(u)

        return self.convertData(u, samplePeriod=self.samplePeriod, **self.measurement_params[m_point])

    # get an instantaneous measurement of voltage and current (debugging)
    def getInstantaneous(self, m_point=1):
        """
            This is mostly a debug function that returns the instantaneous
            current and voltage, as well as an average of the current and
            voltage over the last 32 samples.
        """
        b = self.dev.ctrl_transfer(0xc3, 11, int(m_point), 0, calcsize(EnergyMonitor.InstantaneousData_packing))
        args = list(unpack(EnergyMonitor.InstantaneousData_packing, b))
        args.append(m_point)
        return args

    # Convert and display instantaneous measurement
    def debugInstantaneous(self, v):
        """
            Display the data returned in getInstantaneous.
        """
        mp = v[5]
        resistor = self.measurement_params[mp]['resistor']
        gain = self.measurement_params[mp]['gain']
        vref = self.measurement_params[mp]['vref']

        print "Timestamp:", v[4] * 2. / 168000000 * 2
        print "Current:  Raw={:4d}  Voltage@{}={:1.3f}V  Res Vdrop={:1.5f}V  Current={:1.5f}A".format(mp,
            EnergyMonitor.port_mappings[mp][1],
            v[3]/4096.*vref,
            float(vref) / gain / 4096. * v[3],
            float(vref) / gain / resistor / 4096. * v[3])
        print "Voltage:  Raw={:4d}  Voltage@{}={:1.3f}V                      Voltage={:1.5f}V".format(v[2],
            EnergyMonitor.port_mappings[mp][0],
            v[2]/4096.*vref,
            float(vref) / 4096. * v[2] * 2)
        print ""

    def disconnect(self):
        """For now this doesn't do anything"""
        pass



if __name__ == "__main__":
    from time import sleep

    em = EnergyMonitor("EE00")
    em.connect()

    em.toggleLEDs()

    em.enableMeasurementPoint(1)
    em.enableMeasurementPoint(2)

    em.setTrigger("PA0", 1)
    em.setTrigger("PA0", 2)

    print "*** Press the blue button to make a measurement"

    while True:
        while not em.measurementCompleted(): sleep(0.1)
        print em.getMeasurement(1)
        print em.getMeasurement(2)
