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
                'node_name': '/autopilot',
                'params': {
                    'r_kp': 'Roll Angle P Gain',
                    'r_kd': 'Roll Angle D Gain',
                }
            },
            'Pitch Angle': {
                'node_name': '/autopilot',
                'params': {
                    'p_kp': 'Pitch Angle P Gain',
                    'p_kd': 'Pitch Angle D Gain',
                }
            },
            'Airspeed': {
                'node_name': '/autopilot',
                'params': {
                    'a_kp': 'Airspeed P Gain',
                    'a_ki': 'Airspeed I Gain',
                }
            },
            'Course': {
                'node_name': '/autopilot',
                'params': {
                    'c_kp': 'Course P Gain',
                    'c_ki': 'Course I Gain',
                }
            },
            'Altitude': {
                'node_name': '/autopilot',
                'params': {
                    'a_kp': 'Altitude P Gain',
                    'a_ki': 'Altitude I Gain',
                }
            },
            'Line Following': {
                'node_name': '/path_follower',
                'params': {
                    'l_kp': 'Line Following P Gain',
                    'l_ki': 'Line Following I Gain',
                }
            },
            'Orbit Following': {
                'node_name': '/path_follower',
                'params': {
                    'o_kp': 'Orbit Following P Gain',
                    'o_ki': 'Orbit Following I Gain',
                }
            },
            'Fake Controller': {
                'node_name': '/fake_controller',
                'params': {
                    'f_kp': 'Fake Controller P Gain',
                    'f_ki': 'Fake Controller I Gain',
                    'f_kd': 'Fake Controller D Gain',
                }
            },
        }

        # Define table formatting
        self.tableHeaders = ['Parameter', 'Value', 'Description', 'Reset to Previous', 'Reset to Original']
        self.tableWidths = [175, 125, 500, 250, 250]

        # Set up the widget
        # Group selection - QComboBox
        self.groupSelection.addItems(self.configuration.keys())
        self.groupSelection.currentTextChanged.connect(self.groupSelectionCallback)
        # Refresh button - QPushButton
        self.refreshButton.clicked.connect(self.refreshButtonCallback)
        # Save to file button - QPushButton
        self.saveButton.clicked.connect(self.saveButtonCallback)
        # Parameter table - QTableView
        self.setupTableModels()
        self.insertButtonsInTable()
        self.applyTableFormatting()

    def setupTableModels(self):
        # Create a model for every group
        self.models = {}
        self.currentGroupKey = list(self.configuration.keys())[0]
        for group in self.configuration:
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(self.tableHeaders)
            for param in self.configuration[group]['params']:
                desc = self.configuration[group]['params'][param]
                model.appendRow([QStandardItem(param), QStandardItem('0.0'), QStandardItem(desc)])
            self.models[group] = model

        # Load the first model into the table
        self.paramTableView.setModel(self.models[self.currentGroupKey])

    def insertButtonsInTable(self):
        # Get a list of gains for the current group
        currentGroup = self.configuration[self.currentGroupKey]
        currentParams = list(currentGroup['params'].keys())

        for i, param in enumerate(currentParams):
            # Create reset to previous buttons
            button = QPushButton(param)
            button.clicked.connect(lambda: print(self.currentGroupKey, '"Reset to Previous" button clicked'))
            index = self.paramTableView.model().index(i, 3)
            self.paramTableView.setIndexWidget(index, button)

            # Create reset to original buttons
            button = QPushButton(param)
            button.clicked.connect(lambda: print(self.currentGroupKey, '"Reset to Original" button clicked'))
            index = self.paramTableView.model().index(i, 4)
            self.paramTableView.setIndexWidget(index, button)

    def applyTableFormatting(self):
        # Set the column widths
        for i, width in enumerate(self.tableWidths):
            self.paramTableView.setColumnWidth(i, width)

        # Hide the number row
        self.paramTableView.verticalHeader().hide()

    def groupSelectionCallback(self, text):
        self.currentGroupKey = text
        self.paramTableView.setModel(self.models[text])
        self.insertButtonsInTable()

    def refreshButtonCallback(self):
        print('Refresh button clicked')

    def saveButtonCallback(self):
        print('Save button clicked')

