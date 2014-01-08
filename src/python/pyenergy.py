import usb
import usb.core
import usb.util

from struct import *
from copy import copy
from collections import namedtuple

from logging import warning, error

class Measurement(object):
    def __init__(self):
        pass

class EnergyMonitor(object):
    MeasurementData = namedtuple('MeasurementData', 'energy_accum elapsed_time peak_power peak_voltage peak_current n_samples avg_current avg_voltage')
    MeasurementData_packing = "=QQLLLLQQ"

    ADC1 = 0
    ADC2 = 0
    ADC3 = 0

    def __init__(self, serial="EE00"):
        # Find the usb device that corresponds to the serial number
        devs = usb.core.find(idVendor=0xf539, idProduct=0xf539,
            find_all = True, custom_match=lambda d: d.serial_number == serial)

        if len(devs) > 1:
            warning("More than one device available with serial " + serial)
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

    # Connect to the device
    def connect(self):
        self.dev.set_configuration()

    # Toggle the LEDs on the device
    def toggleLEDs(self):
        self.dev.ctrl_transfer(0x41, 0, 0, 0, None)

    # Start measuring on m_point
    def start(self, m_point=1):
        self.dev.ctrl_transfer(0x41, 1, int(m_point), 0, None)

    # Stop measuring on m_point
    def stop(self, m_point=1):
        self.dev.ctrl_transfer(0x41, 2, int(m_point), 0, None)

    def isRunning(self, m_point=1):
        b = self.dev.ctrl_transfer(0xc1, 8, int(m_point), 0, 4)

        running = unpack("=L", b)
        return bool(running)

    # Trigger
    def setTrigger(self, port, m_point=1):

        # TODO check port is of the form PA0
        self.dev.ctrl_transfer(0x41, 4, ord(port[1]), int(port[2]), None)

    # Enable a particular measurement point. There are
    # only 3 ADCs in the device, so only 3 measurement points
    # can be used simultaneously
    def enableMeasurementPoint(self, m_point, adc=None):
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
            if m_point in [1, 2] and self.adcMpoint[2] is not None:
                adc = 2
            else:
                adc = self.adcMpoint.index(None)
        if m_point in [3,4] and adc == 2:
            raise RuntimeError("Measurement point {} cannot be used with ADC3 (the only free ADC)".format(m_point))
        self.adcMpoint[adc] = m_point
        self.dev.ctrl_transfer(0x41, 7, int(m_point), adc, None)

    # Disable a particular measurement point, so that the
    # ADC could potentially be used with a different one
    def disableMeasurementPoint(self, m_point):
        if m_point not in self.adcMpoint:
            warning("Tried to disable already disabled measurement point "+str(m_point))
        adc = self.adcMpoint.index(m_point)
        self.adcMpoint[adc] = None
        # TODO: perhaps a control transfer to actually disable the mpoint

    def convertData(self, md, resistor, gain, vref, samplePeriod):
        en = float(vref)**2 / gain / resistor / 4096**2 * 2 * samplePeriod * 2 / 168000000. * md.energy_accum * 2
        el = md.elapsed_time * 2. / 168000000 * 2
        pp = float(vref)**2 / gain / resistor / 4096**2 * md.peak_power * 2
        pv = float(vref) / 4096. * md.peak_voltage * 2
        pi = float(vref) / gain / resistor / 4096. * md.peak_current
        av = float(vref) / 4096. * md.avg_voltage / md.n_samples * 2
        ai = float(vref) / gain / resistor / 4096. * md.avg_current / md.n_samples

        print "Peak power   (W):", pp
        print "Peak voltage (v):", pv
        print "Peak current (A):", pi
        print "Energy       (J):", en
        print "Time         (s):", el
        print "Avg Power    (W):", en/el
        print "Avg voltage  (V):", av
        print "Avg current  (A):", ai
        print md

    def getMeasurement(self, m_point=1):
        b = self.dev.ctrl_transfer(0xc1, 6, int(m_point), 0, 48)
        u = EnergyMonitor.MeasurementData._make(unpack(EnergyMonitor.MeasurementData_packing, b))

        self.convertData(u, samplePeriod=self.samplePeriod, **self.measurement_params[m_point])


    def disconnect(self):
        pass



if __name__ == "__main__":
    from time import sleep

    em = EnergyMonitor("EE00")
    em.connect()

    em.toggleLEDs()

    em.measurement_params[1]['resistor'] = 1
    # em.enableMeasurementPoint(1)
    em.start()
    sleep(1)
    em.stop()
    sleep(0.1)
    em.getMeasurement()

    sleep(1)
    em.setTrigger("PA0")

    while True:
        print em.isRunning()
        em.getMeasurement()
        sleep(0.1)
