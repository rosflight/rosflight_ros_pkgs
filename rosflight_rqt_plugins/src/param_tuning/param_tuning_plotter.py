import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QApplication

class ParamTuningPlotter(QWidget):
    def __init__(self, config: dict, paramClient, layout: QVBoxLayout, plotRate: float=5):
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

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._plot)
        self._timer.start(int(1000 / plotRate))

        # Plot parameters
        self._currentGroup = list(self._config.keys())[0]

    def _plot(self):
        self._canvas.figure.clear()
        self._ax = self._canvas.figure.subplots()
        self._ax.set_xlabel('Time (s)', fontsize=self._fontSize)
        self._ax.set_ylabel('Value', fontsize=self._fontSize)
        self._ax.tick_params(axis='both', labelsize=self._fontSize)
        self._ax.grid(True)

        plot_topics = self._config[self._currentGroup]['plot_topics']
        for topic in plot_topics:
            topic_str = self._config[self._currentGroup]['plot_topics'][topic]
            x, y = self._paramClient.get_data(topic_str)
            self._ax.plot(x, y, label=topic)

        self._ax.legend(loc='upper right', fontsize=self._fontSize)
        self._canvas.draw()

    def switchPlotGroup(self, group: str) -> None:
        self._currentGroup = group
        self._plot()

    def shutdown(self):
        self._timer.stop()
        self._canvas.close()
