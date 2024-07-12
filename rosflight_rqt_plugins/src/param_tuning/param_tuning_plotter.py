import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QApplication

class ParamTuningPlotter(QWidget):
    def __init__(self, config: dict, paramClient, layout: QVBoxLayout):
        super(ParamTuningPlotter, self).__init__()
        self.setObjectName('ParamTuningPlotter')

        self.config = config
        self.paramClient = paramClient
        self.canvas = FigureCanvasQTAgg(Figure())
        layout.addWidget(self.canvas)

        # Get current font size (not sure if this is needed on non-HiDPI screens?)
        default_font: QFont = QApplication.font()
        if default_font.pointSize() > 0:
            self.fontSize = default_font.pointSize() * 2
        elif default_font.pixelSize() > 0:
            self.fontSize = default_font.pixelSize() * 2

        self.plot()

    def plot(self):
        self.ax = self.canvas.figure.subplots()
        self.ax.set_xlabel('Time (s)', fontsize=self.fontSize)
        self.ax.set_ylabel('Value', fontsize=self.fontSize)
        self.ax.tick_params(axis='both', labelsize=self.fontSize)
        self.ax.grid(True)
        x = np.linspace(0, 10, 100)
        y1 = np.sin(x)
        y2 = np.cos(x)
        self.ax.plot(x, y1, label='sin(x)')
        self.ax.plot(x, y2, label='cos(x)')
        self.ax.legend(loc='upper right', fontsize=self.fontSize)
        self.canvas.draw()
