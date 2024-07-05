import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtWidgets import QWidget, QPushButton


class ParamTuningWidget(QWidget):
    def __init__(self):
        # Initialize widget
        super(ParamTuningWidget, self).__init__()
        self.setObjectName('ParamTuningWidget')

        # Load the UI file
        _, path = get_resource('packages', 'rosflight_rqt_plugins')
        ui_file = os.path.join(path, 'share', 'rosflight_rqt_plugins', 'resources', 'param_tuning.ui')
        loadUi(ui_file, self)

        # Load parameter configuration
        # Temporary hard-coded configuration
        self.configuration = {
            'Roll Angle': {
                'r_kp': {'value': 0.0, 'desc': 'Roll Angle P Gain'},
                'r_kd': {'value': 0.0, 'desc': 'Roll Angle D Gain'},
            },
            'Pitch Angle': {
                'p_kp': {'value': 0.0, 'desc': 'Pitch Angle P Gain'},
                'p_kd': {'value': 0.0, 'desc': 'Pitch Angle D Gain'},
            },
            'Airspeed': {
                'a_t_kp': {'value': 0.0, 'desc': 'Airspeed Throttle P Gain'},
                'a_t_ki': {'value': 0.0, 'desc': 'Airspeed Throttle I Gain'},
            },
            'Course': {
                'c_kp': {'value': 0.0, 'desc': 'Course P Gain'},
                'c_ki': {'value': 0.0, 'desc': 'Course I Gain'},
            },
            'Altitude': {
                'a_kp': {'value': 0.0, 'desc': 'Altitude P Gain'},
                'a_ki': {'value': 0.0, 'desc': 'Altitude I Gain'},
            },
        }

        # Set up the widget
        # Group selection - QComboBox
        self.groupSelection.addItems(self.configuration.keys())
        self.groupSelection.currentTextChanged.connect(self.groupSelectionCallback)
        # Publish to ROS button - QPushButton
        self.publishButton.clicked.connect(self.publishButtonCallback)
        # Load from file button - QPushButton
        self.loadButton.clicked.connect(self.loadButtonCallback)
        # Save to file button - QPushButton
        self.saveButton.clicked.connect(self.saveButtonCallback)
        # Parameter table - QTableView
        self.setupTableModels()
        self.insertButtonsInTable()

    def setupTableModels(self):
        # Create a model for every group
        self.models = {}
        self.currentGroupKey = list(self.configuration.keys())[0]
        for group in self.configuration:
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(['Parameter', 'Value', 'Description',
                                             'Reset to Previous', 'Reset to Original'])
            for param in self.configuration[group]:
                value = self.configuration[group][param]['value']
                desc = self.configuration[group][param]['desc']
                model.appendRow([QStandardItem(param), QStandardItem(str(value)), QStandardItem(desc)])
            self.models[group] = model

        # Load the first model into the table
        self.paramTableView.setModel(self.models[self.currentGroupKey])

    def insertButtonsInTable(self):
        # Get a list of gains for the current group
        currentGroup = self.configuration[self.currentGroupKey]
        currentGains = list(currentGroup.keys())

        for i, gain in enumerate(currentGains):
            # Create reset to previous buttons
            button = QPushButton(gain)
            button.clicked.connect(lambda: print(self.currentGroupKey, ' "Reset to Previous" button clicked'))
            index = self.paramTableView.model().index(i, 3)
            self.paramTableView.setIndexWidget(index, button)

            # Create reset to original buttons
            button = QPushButton(gain)
            button.clicked.connect(lambda: print(self.currentGroupKey, ' "Reset to Original" button clicked'))
            index = self.paramTableView.model().index(i, 4)
            self.paramTableView.setIndexWidget(index, button)

    def groupSelectionCallback(self, text):
        self.currentGroupKey = text
        self.paramTableView.setModel(self.models[text])
        self.insertButtonsInTable()

    def publishButtonCallback(self):
        print('Publish button clicked')

    def loadButtonCallback(self):
        print('Load button clicked')

    def saveButtonCallback(self):
        print('Save button clicked')

