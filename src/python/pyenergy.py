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
        self.measurement_params = {i : copy(defaultparams) for i in range(1,4)}

        self.samplePeriod = 500

    def connect(self):
        self.dev.set_configuration()

    def toggleLEDs(self):
        self.dev.ctrl_transfer(0x41, 0, 0, 0, None)

    def start(self):
        self.dev.ctrl_transfer(0x41, 1, 0, 0, None)

    def stop(self):
        self.dev.ctrl_transfer(0x41, 2, 0, 0, None)

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
        b = self.dev.ctrl_transfer(0xc1, 6, 0, 0, 48)
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
    em.start()
    sleep(1)
    em.stop()
    sleep(0.1)
    em.getMeasurement()
