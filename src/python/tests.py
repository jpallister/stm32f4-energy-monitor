from pyenergy import EnergyMonitor
from time import sleep

resistors = [330, 330, 330]

# Check values are as expected
def checkOutput(em, m_point, time, bounds=0.05):
    vals = em.getMeasurement(m_point)

    # TODO check

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

@withMeasurementPoints(1, 2)
def test_1(em):
    em.start(2)
    em.start(1)
    sleep(1)
    em.stop(1)
    em.stop(2)

@withMeasurementPoints(1, 2)
def test_2(em):
    em.start(2)
    em.start(1)
    sleep(1)
    em.stop(2)
    em.stop(1)

@withMeasurementPoints(1, 2)
def test_3(em):
    em.start(1)
    sleep(0.5)
    em.start(2)
    sleep(0.5)
    em.stop(1)
    sleep(0.5)
    em.stop(2)

@withMeasurementPoints(1, 2, 3)
def test_4(em):
    em.start(1)
    em.start(2)
    em.start(3)
    sleep(1)
    em.stop(1)
    em.stop(2)
    em.stop(3)


if __name__ == "__main__":

    em = EnergyMonitor("EE00")
    em.connect()

    em.toggleLEDs()

    test_1(em)
    em.getMeasurement(1)
    em.getMeasurement(2)
    test_2(em)
    em.getMeasurement(1)
    em.getMeasurement(2)
    test_3(em)
    em.getMeasurement(1)
    em.getMeasurement(2)

    test_4(em)

