import matplotlib
from matplotlib.pyplot import *
import pyenergy
from time import sleep

em = pyenergy.EnergyMonitor("EE00")
em.connect()

em.enableMeasurementPoint(1)
em.start(1)

xdata = []
ydata = []
ydata2 = []
ydata3 = []

fig = figure()
ion()
show()

gain = 50
resistor = 1
vref = 3

while True:
    m = em.getInstantaneous(1)
    # m2 = em.getInstantaneous(2)
    # m3 = em.getInstantaneous(3)

    print m, m3

    print m[1] * float(vref)**2 / gain / resistor / 4096**2 * 2
    ydata.append(m[1] * float(vref)**2 / gain / resistor / 4096**2 * 2*1000)
    # ydata2.append(m2[1] * float(vref)**2 / gain / resistor / 4096**2 * 2)
    # ydata3.append(m3[1] * float(vref)**2 / gain / resistor / 4096**2 * 2*1000)
    xdata.append(m[2] / 168000000. * 2)
    # em.debugInstantaneous(m)

    fig.clf()
    ax = fig.add_subplot(111)
    ax.plot(xdata, ydata, '-')
    # ax.plot(xdata, ydata2, '-')
    # ax.plot(xdata, ydata3, '-')

    # p.set_ydata(ydata)
    # p.set_xdata(xdata)
    # ax.set_ylim([min(min(ydata),min(ydata3))*0.9, max(max(ydata),max(ydata3))*1.1])
    ax.set_ylim([min(ydata)*0.9, max(ydata)*1.1])
    ax.set_xlim([min(xdata),max(xdata)])
    ax.set_xlabel("Time")
    ax.set_ylabel("Current (mA)")

    fig.show()
    draw()

    sleep(0.01)

    if len(ydata) > 100:
        ydata = ydata[-100:]
        # ydata2 = ydata2[-100:]
        # ydata3 = ydata3[-100:]
        # ydata2 = ydata3[-100:]
        xdata = xdata[-100:]
