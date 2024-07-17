import threading
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

        # Plot parameters
        self._currentGroup = list(self._config.keys())[0]
        self._plotInitialized = False

        # QTimer for updating the plot
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._updatePlot)
        self._timer.start(int(1000 / plotRate))

        # Thread for plotting, keeping GUI responsive
        self._thread = threading.Thread(target=self._plot)
        self._lock = threading.Lock()

    def _updatePlot(self):
        if not self._thread.is_alive():
            self._thread = threading.Thread(target=self._plot)
            self._thread.start()

    def _plot(self):
        with self._lock:
            if not self._plotInitialized:
                self._canvas.figure.clear()
                self._ax = self._canvas.figure.subplots()
                self._ax.set_xlabel('Time (s)', fontsize=self._fontSize)
                self._ax.set_ylabel(self._config[self._currentGroup]['plot_axes']['y_label'], fontsize=self._fontSize)
                self._ax.tick_params(axis='both', labelsize=self._fontSize)
                self._ax.grid(True)
                self._lineObjects = {}
                self._canvas.figure.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)

            for topic in self._config[self._currentGroup]['plot_topics']:
                topic_str = self._config[self._currentGroup]['plot_topics'][topic]
                x, y = self._paramClient.get_data(topic_str)

                if topic not in self._lineObjects:
                    line, = self._ax.plot(x, y, label=topic)
                    self._lineObjects[topic] = line
                else:
                    self._lineObjects[topic].set_data(x, y)

            if not self._plotInitialized:
                self._ax.legend(loc='upper right', fontsize=self._fontSize)

            self._ax.relim()
            self._ax.autoscale_view()
            self._canvas.draw()
            self._plotInitialized = True

    def switchPlotGroup(self, group: str) -> None:
        with self._lock:
            self._currentGroup = group
            self._plotInitialized = False
            self._updatePlot()

    def shutdown(self):
        self._timer.stop()
        self._canvas.close()
