import os
from pathlib import Path
import sys

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class ParamTuningWidget(QWidget):
    def __init__(self):
        super(ParamTuningWidget, self).__init__()
        self.setObjectName('ParamTuningWidget')

        _, path = get_resource('packages', 'rosflight_rqt_plugins')
        ui_file = os.path.join(path, 'share', 'rosflight_rqt_plugins', 'resources', 'param_tuning.ui')
        loadUi(ui_file, self)

