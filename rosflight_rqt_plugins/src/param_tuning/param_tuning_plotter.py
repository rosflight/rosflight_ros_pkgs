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

        self._config = config
        self._paramClient = paramClient
        self._canvas = FigureCanvasQTAgg(Figure())
        layout.addWidget(self._canvas)

        # Get current font size (not sure if this is needed on non-HiDPI screens?)
        default_font: QFont = QApplication.font()
        if default_font.pointSize() > 0:
            self._fontSize = default_font.pointSize() * 2
        elif default_font.pixelSize() > 0:
            self._fontSize = default_font.pixelSize() * 2

        self._plot()

    def _plot(self):
        self._ax = self._canvas.figure.subplots()
        self._ax.set_xlabel('Time (s)', fontsize=self._fontSize)
        self._ax.set_ylabel('Value', fontsize=self._fontSize)
        self._ax.tick_params(axis='both', labelsize=self._fontSize)
        self._ax.grid(True)
        x = np.linspace(0, 10, 100)
        y1 = np.sin(x)
        y2 = np.cos(x)
        self._ax.plot(x, y1, label='sin(x)')
        self._ax.plot(x, y2, label='cos(x)')
        self._ax.legend(loc='upper right', fontsize=self._fontSize)
        self._canvas.draw()
