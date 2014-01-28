import sys, os, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import matplotlib
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.axes_grid.parasite_axes import SubplotHost

import collections

class Graph(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self, parent)
        self.setWindowTitle('Energy monitoring graph')

        self.tinterval = 0.1
        self.ctime = 0
        self.wsize = 100

        self.create_main()
        self.setUpdate()
        self.setup_graphs()

        self.on_draw()

    def single_graph(self):
        self.fig.clear()
        self.fig.subplots_adjust(right=0.8,left=0.1)

        self.axes = SubplotHost(self.fig, 111)
        self.fig.add_axes(self.axes)
        self.vaxes = self.axes.twinx()

        self.paxes = self.axes.twinx()
        self.paxes.axis["right"].set_visible(False)
        offset = (70, 0)
        new_axisline = self.paxes.get_grid_helper().new_fixed_axis

        self.paxes.axis["side"] = new_axisline(loc="right", axes=self.paxes, offset=offset)
        self.paxes.axis["side"].label.set_visible(True)
        self.paxes.axis["side"].set_label("Power (W)")

        self.axes.set_xlabel("Time (s)")
        self.axes.set_ylabel("Current (A)")
        self.vaxes.set_ylabel("Voltage (V)")
        self.paxes.set_ylabel("Power (W)")

    def horiz_graph(self):
        self.fig.clear()
        self.fig.subplots_adjust(right=0.97,left=0.1, wspace=0.33)

        self.axes = self.fig.add_subplot(131)
        self.vaxes = self.fig.add_subplot(132)
        self.paxes = self.fig.add_subplot(133)

        self.axes.set_xlabel("Time (s)")
        self.vaxes.set_xlabel("Time (s)")
        self.paxes.set_xlabel("Time (s)")

        self.axes.set_ylabel("Current (A)")
        self.vaxes.set_ylabel("Voltage (V)")
        self.paxes.set_ylabel("Power (W)")

    def vert_graph(self):
        self.fig.clear()
        self.fig.subplots_adjust(right=0.97,left=0.1, hspace=0.25)

        self.axes = self.fig.add_subplot(311)
        self.vaxes = self.fig.add_subplot(312)
        self.paxes = self.fig.add_subplot(313)

        self.axes.xaxis.set_ticklabels([])
        self.vaxes.xaxis.set_ticklabels([])
        # self.axes.set_xlabel("Time (s)")
        # self.vaxes.set_xlabel("Time (s)")
        self.paxes.set_xlabel("Time (s)")

        self.axes.set_ylabel("Current (A)")
        self.vaxes.set_ylabel("Voltage (V)")
        self.paxes.set_ylabel("Power (W)")


    def create_main(self):
        self.main_frame = QWidget()

        # Create the mpl Figure and FigCanvas objects.
        # 5x4 inches, 100 dots-per-inch
        #
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.single_graph()

        hbox = QHBoxLayout()
        hbox.setAlignment(Qt.AlignCenter)

        # Create general settings
        grid = QGridLayout()
        grid.setSpacing(5)
        grid.setAlignment(Qt.AlignCenter)

        # Add time slider
        grid.addWidget(QLabel("Interval (s)"), 1, 0)
        self.timeslider = QSlider(1)
        self.timeslider.setMinimum(1)
        self.timeslider.setMaximum(200)
        self.timeslider.setValue(20)
        self.timeslider.setTickInterval(1)
        self.timeslider.setSingleStep(1)
        self.connect(self.timeslider, SIGNAL('valueChanged(int)'), self.updatesliders)
        grid.addWidget(self.timeslider, 2, 0)

        # Add window slider
        grid.addWidget(QLabel("Window size"), 3, 0)
        self.windowslider = QSlider(1)
        self.windowslider.setMinimum(2)
        self.windowslider.setMaximum(100)
        self.windowslider.setValue(40)
        self.windowslider.setTickInterval(1)
        self.windowslider.setSingleStep(1)
        self.connect(self.windowslider, SIGNAL('valueChanged(int)'), self.updatesliders)
        grid.addWidget(self.windowslider, 4, 0)

        # Add graph selector
        grid.addWidget(QLabel("Graph stack"), 5, 0)
        self.graphselect = QComboBox()
        self.graphselect.addItems(["Combined", "Horizontal", "Vertical"])
        self.graphselect.setCurrentIndex(0)
        self.connect(self.graphselect, SIGNAL('currentIndexChanged(int)'), self.changegraph)
        grid.addWidget(self.graphselect, 6,0)

        box = QGroupBox("General")
        box.setLayout(grid)
        box.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        hbox.addWidget(box)

        self.controls = collections.defaultdict(list)


        for mp in ["1", "2", "3", "Self"]:
            grid = QGridLayout()
            grid.setSpacing(5)
            grid.setAlignment(Qt.AlignCenter)

            combo = QComboBox()
            combo.addItems(["0.05", "0.5", "1", "5"])
            combo.setCurrentIndex(2)
            self.controls[mp].append(combo)
            grid.addWidget(QLabel("Resistor"), 1, 0)
            grid.addWidget(combo, 1, 1, alignment=Qt.AlignCenter)

            cb = QCheckBox()
            self.controls[mp].append(cb)
            grid.addWidget(QLabel("Plot Current"), 2, 0)
            grid.addWidget(cb, 2, 1, alignment=Qt.AlignCenter)

            cb = QCheckBox()
            self.controls[mp].append(cb)
            grid.addWidget(QLabel("Plot Voltage"), 3, 0)
            grid.addWidget(cb, 3, 1, alignment=Qt.AlignCenter)

            cb = QCheckBox()
            self.controls[mp].append(cb)
            grid.addWidget(QLabel("Plot Power"), 4, 0)
            grid.addWidget(cb, 4, 1, alignment=Qt.AlignCenter)

            box = QGroupBox(mp)
            box.setLayout(grid)
            box.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)

            hbox.addWidget(box)

        vbox = QVBoxLayout()
        vbox.addWidget(self.canvas)
        # vbox.addWidget(self.mpl_toolbar)
        vbox.addLayout(hbox)

        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

    def updatesliders(self):
        self.tinterval = self.timeslider.value() / 100.
        self.wsize = self.windowslider.value()
        self.setUpdate()
        print self.tinterval

    def changegraph(self):
        ind = self.graphselect.currentIndex()

        self.plots = []

        if ind == 0:
            self.single_graph()
        elif ind == 1:
            self.horiz_graph()
        else:
            self.vert_graph()

    def on_draw(self):
        mintimes = []
        maxtimes = []

        maxps = [0]
        minps = [10]
        maxis = [0]
        minis = [10]
        maxvs = [0]
        minvs = [10]

        for p in self.plots:
            p.remove()
        self.plots = []

        for mp, vals in self.data.items():
            if self.state[mp][1]:
                p1,  = self.axes.plot(vals["xdata"], vals["idata"], 'g')
                self.plots.append(p1)
                maxis.append(max(vals["idata"]))
                minis.append(min(vals["idata"]))
            if self.state[mp][2]:
                p1,  = self.vaxes.plot(vals["xdata"], vals["vdata"], 'b')
                self.plots.append(p1)
                maxvs.append(max(vals["vdata"]))
                minvs.append(min(vals["vdata"]))
            if self.state[mp][3]:
                p1,  = self.paxes.plot(vals["xdata"], vals["pdata"], 'r')
                self.plots.append(p1)
                maxps.append(max(vals["pdata"]))
                minps.append(min(vals["pdata"]))

            if self.state[mp][1] or self.state[mp][2] or self.state[mp][3]:
                maxtimes.append(max(vals["xdata"]))
                mintimes.append(min(vals["xdata"]))

        mmtimes = self.tinterval * self.wsize
        if len(maxtimes) != 0 and max(maxtimes) > mmtimes:
            mmtimes = max(maxtimes)

        self.axes.set_xlim([ mmtimes - self.tinterval*self.wsize,  mmtimes])
        self.vaxes.set_xlim([ mmtimes - self.tinterval*self.wsize,  mmtimes])
        self.paxes.set_xlim([ mmtimes - self.tinterval*self.wsize,  mmtimes])

        poff = (max(maxps) - min(minps)) *0.1
        self.paxes.set_ylim([min(minps)-poff, max(maxps)+poff])

        ioff = (max(maxis) - min(minis)) *0.1
        self.axes.set_ylim([min(minis) - ioff, max(maxis) + ioff])

        voff = (max(maxvs) - min(minvs)) *0.1
        self.vaxes.set_ylim([min(minvs)-voff, max(maxvs)+voff])

        self.canvas.draw()

    def setup_graphs(self):
        self.data = {
            "1": {"xdata": [],
                  "idata": [],
                  "pdata": [],
                  "vdata": []},
            "2": {"xdata": [],
                  "idata": [],
                  "pdata": [],
                  "vdata": []},
            "3": {"xdata": [],
                  "idata": [],
                  "pdata": [],
                  "vdata": []},
            "Self": {"xdata": [],
                  "idata": [],
                  "pdata": [],
                  "vdata": []},
            }
        self.state = {
            "1": [2, False, False, False],
            "2": [2, False, False, False],
            "3": [2, False, False, False],
            "Self": [2, False, False, False],
            }
        self.plots = []

    def setUpdate(self):
        self.timer = QTimer()
        self.timer.setSingleShot(False)
        self.timer.timeout.connect(self.update)
        self.timer.start(self.tinterval * 1000)

    def getState(self):
        state = collections.defaultdict(list)
        for mp, vals in self.controls.items():
            for i, control in enumerate(vals):
                if i == 0:
                    state[mp].append(control.currentIndex())
                else:
                    state[mp].append(control.checkState())
        return state


    def update(self):
        state = self.getState()

        self.ctime += self.tinterval

        for i, mp in enumerate(sorted(state.keys())):
            #self.em.measurement_params[i+1]['resistor'] = [0.05, 0.5, 1.0, 5.0][state[mp][0]]

            # Check we need to enable or disable a measurement point
            if bool(sum(state[mp][1:])) != bool(sum(self.state[mp][1:])):
                if bool(sum(state[mp][1:])):
                    #self.em.enableMeasurementPoint(i+1)
                    #self.em.start(i+1)
                    print "Enable", mp
                else:
                    #self.em.stop(i+1)
                    #self.em.disableMeasurementPoint(i+1)
                    print "Disable", mp

            if bool(sum(state[mp][1:])):
                # m = self.em.getMeasurement(i+1)
                # v = m.average_voltage
                # c = m.average_current
                # p = v * c
                # t = m.elapsed_time

                v = 3 + random.randint(0,100)/300.0
                c = 0.01 + random.randint(0,100)/1000.0
                p = v * c
                t = self.ctime

                self.data[mp]['xdata'].append(t)
                self.data[mp]['idata'].append(c)
                self.data[mp]['vdata'].append(v)
                self.data[mp]['pdata'].append(p)

                self.data[mp]['xdata'] = self.data[mp]['xdata'][-self.wsize:]
                self.data[mp]['idata'] = self.data[mp]['idata'][-self.wsize:]
                self.data[mp]['vdata'] = self.data[mp]['vdata'][-self.wsize:]
                self.data[mp]['pdata'] = self.data[mp]['pdata'][-self.wsize:]

        self.state = state
        self.on_draw()

def main():
    app = QApplication(sys.argv)
    app.setStyle("plastique")
    form = Graph()
    form.show()
    app.exec_()


if __name__ == "__main__":
    main()
