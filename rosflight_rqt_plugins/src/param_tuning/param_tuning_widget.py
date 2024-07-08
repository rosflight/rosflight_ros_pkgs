import os

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSlot, QModelIndex
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtWidgets import QWidget, QPushButton


class ParamTuningWidget(QWidget):
    def __init__(self, config: dict, paramClient):
        # Initialize widget
        super(ParamTuningWidget, self).__init__()
        self.setObjectName('ParamTuningWidget')

        # Load the UI file
        _, path = get_resource('packages', 'rosflight_rqt_plugins')
        uiFile = os.path.join(path, 'share', 'rosflight_rqt_plugins', 'resources', 'param_tuning.ui')
        loadUi(uiFile, self)

        # Define table formatting
        self.config = config
        self.tableHeaders = ['Parameter', 'Value', 'Description', 'Reset to Previous', 'Reset to Initial']
        self.tableWidths = [175, 125, 500, 250, 250]

        # Get the original values of the parameters
        self.paramClient = paramClient
        self.previousValues = {}
        for group in config:
            node_name = config[group]['node']
            for param in config[group]['params']:
                value = self.paramClient.get_param(node_name, param)
                self.previousValues[(group, param)] = [value]

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
        self.refreshTableValues()

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

        # Set the column widths
        for i, width in enumerate(self.tableWidths):
            self.paramTableView.setColumnWidth(i, width)

        # Hide the number row
        self.paramTableView.verticalHeader().hide()

        # Connect the model change signal
        self.paramTableView.model().dataChanged.connect(self.onModelChange)

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
            button = QPushButton(str(self.previousValues[(self.currentGroupKey, param)][0]))
            button.clicked.connect(lambda: print(self.currentGroupKey, '"Reset to Original" button clicked'))
            index = self.paramTableView.model().index(i, 4)
            self.paramTableView.setIndexWidget(index, button)

    def changeTableValue(self, group, param, value):
        for i in range(self.models[group].rowCount()):
            if self.models[group].item(i, 0).text() == param:
                self.models[group].item(i, 1).setText(str(value))
                break
        self.paramClient.set_param(self.config[group]['node'], param, value)

    def refreshTableValues(self):
        # Temporarily disconnect the model change signal
        self.paramTableView.model().dataChanged.disconnect(self.onModelChange)

        # Get current values of the parameters
        node_name = self.config[self.currentGroupKey]['node']
        for i in range(self.models[self.currentGroupKey].rowCount()):
            param = self.models[self.currentGroupKey].item(i, 0).text()
            value = self.paramClient.get_param(node_name, param)
            self.models[self.currentGroupKey].item(i, 1).setText(str(value))

        # Reconnect the model change signal
        self.paramTableView.model().dataChanged.connect(self.onModelChange)

    def groupSelectionCallback(self, text):
        self.currentGroupKey = text

        # Swap models and connect the model change signal to the new model
        self.paramTableView.model().dataChanged.disconnect(self.onModelChange)
        self.paramTableView.setModel(self.models[text])
        self.paramTableView.model().dataChanged.connect(self.onModelChange)

        # Update table
        self.insertButtonsInTable()
        self.refreshTableValues()

    def refreshButtonCallback(self):
        self.refreshTableValues()

    def saveButtonCallback(self):
        print('Save button clicked')

    @pyqtSlot(QModelIndex, QModelIndex)
    def onModelChange(self, topLeft, bottomRight):
        try:
            value = float(topLeft.data())
        except ValueError:
            self.paramClient.print_warning('Invalid value type, please enter a number.')
            self.refreshTableValues()
            return
        param = self.models[self.currentGroupKey].item(topLeft.row(), 0).text()
        node_name = self.config[self.currentGroupKey]['node']
        self.paramClient.set_param(node_name, param, value)
