import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from python_qt_binding.QtWidgets import QWidget, QVBoxLayout

class ParamTuningPlotter(QWidget):
    def __init__(self, config: dict, paramClient, layout: QVBoxLayout):
        super(ParamTuningPlotter, self).__init__()
        self.setObjectName('ParamTuningPlotter')

        self.config = config
        self.paramClient = paramClient
        self.canvas = FigureCanvasQTAgg(Figure())
        layout.addWidget(self.canvas)

        self.ax = self.canvas.figure.subplots()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Value')
        self.ax.grid(True)
        x = np.linspace(0, 10, 100)
        y = np.sin(x)
        self.ax.plot(x, y)
        self.canvas.draw()
