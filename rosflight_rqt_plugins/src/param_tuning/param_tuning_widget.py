import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtWidgets import QWidget, QPushButton


class ParamTuningWidget(QWidget):
    def __init__(self, config: dict, param_client):
        # Initialize widget
        super(ParamTuningWidget, self).__init__()
        self.setObjectName('ParamTuningWidget')

        # Load the UI file
        _, path = get_resource('packages', 'rosflight_rqt_plugins')
        ui_file = os.path.join(path, 'share', 'rosflight_rqt_plugins', 'resources', 'param_tuning.ui')
        loadUi(ui_file, self)

        # Define table formatting
        self.config = config
        self.tableHeaders = ['Parameter', 'Value', 'Description', 'Reset to Previous', 'Reset to Initial']
        self.tableWidths = [175, 125, 500, 250, 250]

        # Set up the parameter client
        self.param_client = param_client
        self.originalValues = {}
        for group in config:
            node_name = config[group]['node']
            for param in config[group]['params']:
                self.originalValues[(group, param)] = self.param_client.get_param(node_name, param)

        # Set up the widget
        # Group selection - QComboBox
        self.groupSelection.addItems(self.config.keys())
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
        self.currentGroupKey = list(self.config.keys())[0]
        for group in self.config:
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(self.tableHeaders)
            for param in self.config[group]['params']:
                desc = self.config[group]['params'][param]
                model.appendRow([QStandardItem(param), QStandardItem('0.0'), QStandardItem(desc)])
            self.models[group] = model

        # Load the first model into the table
        self.paramTableView.setModel(self.models[self.currentGroupKey])

    def insertButtonsInTable(self):
        # Get a list of gains for the current group
        currentGroup = self.config[self.currentGroupKey]
        currentParams = list(currentGroup['params'].keys())

        for i, param in enumerate(currentParams):
            # Create reset to previous buttons
            button = QPushButton(param)
            button.clicked.connect(lambda: print(self.currentGroupKey, '"Reset to Previous" button clicked'))
            index = self.paramTableView.model().index(i, 3)
            self.paramTableView.setIndexWidget(index, button)

            # Create reset to original buttons
            button = QPushButton(str(self.originalValues[(self.currentGroupKey, param)]))
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

