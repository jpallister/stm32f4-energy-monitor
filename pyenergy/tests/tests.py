from pyenergy import EnergyMonitor
from time import sleep

src_v = 3.0
resistors = [330, 1100, None]

class checkOutput(object):
    def __init__(self, m_point, time, bounds=0.075):
        self.m_point = m_point
        self.time = time
        self.bounds = bounds

    def __call__(self, f):
        def wrap(em, *args):
            f(em, *args)

            m = em.getMeasurement(self.m_point)

            print("\tChecking measurement point", self.m_point, "...", end=' ')

            result = True

            res = resistors[self.m_point-1]
            if res is None:
                print("Cannot check - no resistor")
                return True # Can't test it because it could be floating. Assume it worked
            else:
                # Calculate expected values
                en = src_v ** 2 / res * self.time  # V^2*R*T
                el = self.time
                pp = src_v ** 2 / res
                pv = src_v
                pi = src_v / res
                av = src_v
                ai = src_v / res

            result = result and self.checkBound(el, m.time,       "Error in time,   Expected:{expected}, Got:{test}")

            # Energy is very dependent on time. Scale expected energy to time.
            # The tests are a poor check for timing, due to USB  and OS timing
            # over heads.
            en = en / self.time * m.time

            result = result and self.checkBound(en, m.energy,     "Error in energy, Expected:{expected}, Got:{test}")
            # result = result and self.checkBound(pp, m.peak_power, "Error in peak power,   Expected:{expected}, Got:{test}", bounds=self.bounds*2)
            # result = result and self.checkBound(pv, m.peak_voltage, "Error in peak voltage,   Expected:{expected}, Got:{test}", bounds=self.bounds*2)
            # result = result and self.checkBound(pi, m.peak_current, "Error in peak current,   Expected:{expected}, Got:{test}", bounds=self.bounds*2)
            result = result and self.checkBound(av, m.avg_voltage, "Error in average voltage,   Expected:{expected}, Got:{test}")
            result = result and self.checkBound(ai, m.avg_current, "Error in average current,   Expected:{expected}, Got:{test}")

            if not result:
                print("Failed")
            else:
                print("Passed")
            return result
        return wrap

    def checkBound(self, expected, test, msg, bounds=None, noise=0):
        if bounds == None:
            bounds = self.bounds
        if test < expected * (1 - bounds) - noise or test > expected * (1 + bounds) + noise:
            print(msg.format(test=test, expected=expected))
            return False
        return True

class withMeasurementPoints(object):
    def __init__(self, *args):
        self.pnts = args

    def __call__(self, f):
        def wrap(em, *args):
            for p in self.pnts:
                em.enableMeasurementPoint(p)
            ret = f(em, *args)
            for p in self.pnts:
                em.disableMeasurementPoint(p)
            return ret
        return wrap

class testName(object):
    def __init__(self, name):
        self.name = name

    def __call__(self, f):
        def wrap(em, *args):
            print("Running test:",self.name)
            return f(em, *args)
        return wrap

@checkOutput(2, 1.0)
@checkOutput(1, 1.0)
@withMeasurementPoints(1, 2)
@testName("Multiple measurements")
def test_1(em):
    em.start(2)
    em.start(1)
    sleep(1)
    em.stop(1)
    em.stop(2)

@checkOutput(2, 1.0)
@checkOutput(1, 1.0)
@withMeasurementPoints(1, 2)
@testName("Interleaved start/stop")
def test_2(em):
    em.start(2)
    em.start(1)
    sleep(1)
    em.stop(2)
    em.stop(1)

@checkOutput(2, 1.0)
@checkOutput(1, 1.0)
@withMeasurementPoints(1, 2)
@testName("Interleaved offsetted start/stop")
def test_3(em):
    em.start(1)
    sleep(0.5)
    em.start(2)
    sleep(0.5)
    em.stop(1)
    sleep(0.5)
    em.stop(2)

@checkOutput(2, 1.0)
@checkOutput(1, 2.0)
@withMeasurementPoints(1, 2)
@testName("Interleaved offsetted start/stop")
def test_4(em):
    em.start(1)
    sleep(0.5)
    em.start(2)
    sleep(1.0)
    em.stop(2)
    sleep(0.5)
    em.stop(1)


@checkOutput(3, 1.0)
@checkOutput(2, 1.0)
@checkOutput(1, 1.0)
@withMeasurementPoints(1, 2, 3)
@testName("Three measurement points")
def test_5(em):
    em.start(1)
    em.start(2)
    em.start(3)
    sleep(1)
    em.stop(1)
    em.stop(2)
    em.stop(3)

@checkOutput(3, 1.0)
@checkOutput(2, 1.0)
@checkOutput(1, 1.0)
@withMeasurementPoints(1, 2, 3)
@testName("Three measurement points interleaved")
def test_6(em):
    em.start(3)
    em.start(2)
    em.start(1)
    sleep(1)
    em.stop(1)
    em.stop(2)
    em.stop(3)

@checkOutput(1, 0.2)
@withMeasurementPoints(1)
@testName("Short test")
def test_7(em):
    em.start(1)
    sleep(0.2)
    em.stop(1)

@checkOutput(1, 10.)
@withMeasurementPoints(1)
@testName("Long test")
def test_8(em):
    em.start(1)
    sleep(10.)
    em.stop(1)

@checkOutput(2, 10.)
@checkOutput(1, 0.2)
@withMeasurementPoints(1,2)
@testName("Many short/long tests interleaved")
def test_9(em):
    em.start(2)
    for i in range(50):
        em.start(1)
        sleep(0.2)
        em.stop(1)
    em.stop(2)

    em.start(1)
    sleep(0.2)
    em.stop(1)

if __name__ == "__main__":

    em = EnergyMonitor("EE00")
    em.connect()

    em.toggleLEDs()

    test_1(em)
    test_2(em)
    test_3(em)
    test_4(em)
    test_5(em)
    test_6(em)
    test_7(em)
    test_8(em)
    test_9(em)

