import threading
import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QApplication

class ParamTuningPlotter(QWidget):
    def __init__(self, config: dict, param_client, layout: QVBoxLayout, plot_rate: float=5):
        super(ParamTuningPlotter, self).__init__()
        self.setObjectName('ParamTuningPlotter')

        self._config = config
        self._param_client = param_client
        self._canvas = FigureCanvasQTAgg(Figure())
        layout.addWidget(self._canvas)

        # Plot parameters
        self._current_group = list(self._config.keys())[0]
        self._plot_initialized = False

        # QTimer for updating the plot
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_plot)
        self._timer.start(int(1000 / plot_rate))

        # Thread for plotting, keeping GUI responsive
        self._thread = threading.Thread(target=self._plot)
        self._lock = threading.Lock()

    def _update_plot(self):
        if not self._thread.is_alive():
            self._thread = threading.Thread(target=self._plot)
            self._thread.start()

    def _plot(self):
        with self._lock:
            if 'plot_topics' not in self._config[self._current_group]:
                self._canvas.figure.clear()
                self._canvas.draw()
                return

            if not self._plot_initialized:
                self._canvas.figure.clear()
                self._ax = self._canvas.figure.subplots()
                self._ax.set_xlabel('Time (s)')
                self._ax.set_ylabel(self._config[self._current_group]['plot_axis_label'])
                self._ax.tick_params(axis='both')
                self._ax.grid(True)
                self._lineObjects = {}
                self._canvas.figure.subplots_adjust(left=0.05, right=0.98, top=0.98, bottom=0.05)

                if 'plot_axis_range' in self._config[self._current_group]:
                    min_value, max_value = self._config[self._current_group]['plot_axis_range']
                    self._ax.set_ylim(min_value, max_value)

            for plot_name in self._config[self._current_group]['plot_topics']:
                topic_str = self._config[self._current_group]['plot_topics'][plot_name]['topic']
                x, y = self._param_client.get_data(topic_str)
                y = np.array(y) * self._config[self._current_group]['plot_topics'][plot_name].get('scale', 1.0)

                if plot_name not in self._lineObjects:
                    line, = self._ax.plot(x, y, label=plot_name)
                    self._lineObjects[plot_name] = line
                else:
                    self._lineObjects[plot_name].set_data(x, y)

            if not self._plot_initialized:
                self._ax.legend(loc='upper right')

            self._ax.relim()
            self._ax.autoscale_view()
            self._canvas.draw()
            self._plot_initialized = True

    def switch_plot_group(self, group: str) -> None:
        with self._lock:
            self._current_group = group
            self._plot_initialized = False
            self._update_plot()

    def pause_plotting(self, pause: bool) -> None:
        if pause:
            self._timer.stop()
        else:
            self._timer.start()

    def shutdown(self):
        self._timer.stop()
        self._canvas.close()
