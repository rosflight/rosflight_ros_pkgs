import os
import yaml

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSlot, QModelIndex
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtWidgets import QWidget, QPushButton, QFileDialog


class ParamTuningWidget(QWidget):
    def __init__(self, config: dict, paramClient, paramFilepath: str):
        # Initialize widget
        super(ParamTuningWidget, self).__init__()
        self.setObjectName('ParamTuningWidget')
        self._paramFilePath = paramFilepath
        self._addChangedValuesToHist = True
        self._plotSwapCallback = lambda x: None

        # Load the UI file
        _, path = get_resource('packages', 'rosflight_rqt_plugins')
        uiFile = os.path.join(path, 'share', 'rosflight_rqt_plugins', 'resources', 'param_tuning.ui')
        loadUi(uiFile, self)

        # Define table formatting
        self._config = config
        self._tableHeaders = ['Parameter', 'Value', 'Description', 'Reset to Previous', 'Reset to Initial']
        self._tableWidths = [175, 125, 500, 250, 250]

        # Get the original values of the parameters
        self._paramClient = paramClient
        self._valueStack = {}
        for group in config:
            for param in config[group]['params']:
                value = self._paramClient.get_param(group, param)
                self._valueStack[(group, param)] = [value]

        # Set up the widget
        # Group selection - QComboBox
        self.groupSelection.addItems(self._config.keys())
        self.groupSelection.currentTextChanged.connect(self._groupSelectionCallback)
        # Refresh button - QPushButton
        self.refreshButton.clicked.connect(self._refreshButtonCallback)
        # Save to file button - QPushButton
        self.saveButton.clicked.connect(self._saveButtonCallback)
        # Parameter table - QTableView
        self._setupTableModels()
        self._createTableButtons()
        self._refreshTableValues()

    def _setupTableModels(self):
        # Create a model for every group
        self._models = {}
        self._currentGroupKey = list(self._config.keys())[0]
        for group in self._config:
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(self._tableHeaders)
            for param in self._config[group]['params']:
                desc = self._config[group]['params'][param]['description']
                param_item = QStandardItem(param)
                param_item.setEditable(False)
                value_item = QStandardItem('0.0')
                desc_item = QStandardItem(desc)
                desc_item.setEditable(False)
                model.appendRow([param_item, value_item, desc_item])
            self._models[group] = model

        # Load the first model into the table
        self.paramTableView.setModel(self._models[self._currentGroupKey])

        # Set the column widths
        for i, width in enumerate(self._tableWidths):
            self.paramTableView.setColumnWidth(i, width)

        # Hide the number row
        self.paramTableView.verticalHeader().hide()

        # Connect the model change signal
        self.paramTableView.model().dataChanged.connect(self._onModelChange)

    def _createTableButtons(self):
        # Get a list of gains for the current group
        currentGroup = self._config[self._currentGroupKey]
        currentParams = list(currentGroup['params'].keys())

        for i, param in enumerate(currentParams):
            # Create reset to previous buttons
            previousButtonValue = self._valueStack[(self._currentGroupKey, param)][-2] \
                if len(self._valueStack[(self._currentGroupKey, param)]) > 1 \
                else self._valueStack[(self._currentGroupKey, param)][0]

            button = QPushButton(str(previousButtonValue))
            button.clicked.connect(
                lambda _, g=self._currentGroupKey, index=i, p=param: self._resetPreviousButtonCallback(g, index, p)
            )
            index = self.paramTableView.model().index(i, 3)
            self.paramTableView.setIndexWidget(index, button)

            # Create reset to original buttons
            button = QPushButton(str(self._valueStack[(self._currentGroupKey, param)][0]))
            button.clicked.connect(
                lambda _, g=self._currentGroupKey, index=i, p=param: self._resetInitialButtonCallback(g, index, p)
            )
            index = self.paramTableView.model().index(i, 4)
            self.paramTableView.setIndexWidget(index, button)

    def _resetPreviousButtonCallback(self, group, row, param):
        # Pop the last value from list, unless it is the last value
        if len(self._valueStack[(group, param)]) > 1:
            self._valueStack[(group, param)].pop()
        value = self._valueStack[(group, param)][-1]

        # Update the table
        self._addChangedValuesToHist = False
        self._models[group].item(row, 1).setText(str(value))
        self._addChangedValuesToHist = True

    def _resetInitialButtonCallback(self, group, row, param):
        self._models[group].item(row, 1).setText(str(self._valueStack[(group, param)][0]))
        self._valueStack[(group, param)] = [self._valueStack[(group, param)][0]]
        self._createTableButtons()

    def _refreshTableValues(self):
        self._paramClient.print_info('Getting all parameters from ROS network...')

        # Temporarily disconnect the model change signal
        self.paramTableView.model().dataChanged.disconnect(self._onModelChange)

        # Get current values of the parameters
        for group in self._config:
            for i in range(self._models[group].rowCount()):
                param = self._models[group].item(i, 0).text()
                value = self._paramClient.get_param(group, param)

                # If the value is different from the previous value, add it to the stack and update the buttons
                if value != self._valueStack[(group, param)][-1]:
                    self._valueStack[(group, param)].append(value)
                    self._createTableButtons()

                # Update the table
                self._models[group].item(i, 1).setText(str(value))

        # Reconnect the model change signal
        self.paramTableView.model().dataChanged.connect(self._onModelChange)

    def _groupSelectionCallback(self, text):
        self._currentGroupKey = text

        # Swap models and connect the model change signal to the new model
        self.paramTableView.model().dataChanged.disconnect(self._onModelChange)
        self.paramTableView.setModel(self._models[text])
        self.paramTableView.model().dataChanged.connect(self._onModelChange)

        # Update table
        self._createTableButtons()

        # Tell the plotter to swap the plot
        self._plotSwapCallback(text)

    def _refreshButtonCallback(self):
        self._refreshTableValues()

    def _saveButtonCallback(self):
        # Request a filepath is a param filepath hasn't already been given
        if self._paramFilePath is None:
            options = QFileDialog.Options()
            filepath, _ = QFileDialog.getSaveFileName(None, 'Save Parameters to ROS .yaml', '', 'YAML Files (*.yaml)',
                                                      options=options)
            if not filepath:
                self._paramClient.print_warning('No file selected, parameters not saved.')
                return
        else:
            filepath = self._paramFilePath

        # Load the existing parameter file if it exists
        if os.path.exists(filepath):
            with open(filepath, 'r') as file:
                params = yaml.safe_load(file)
        else:
            params = {}

        # Create a dictionary formatted for ROS parameters, based on the current ROS parameters
        for group in self._config:
            param_dict = {}
            for i in range(self._models[group].rowCount()):
                param_name = self._models[group].item(i, 0).text()
                param_dict[param_name] = self._paramClient.get_param(group, param_name, False)

            # Add new items to dictionary, appending it if already exists
            stripped_node_name = self._config[group]['node'].lstrip('/')
            if stripped_node_name in params:
                params[stripped_node_name]['ros__parameters'].update(param_dict)
            else:
                params[stripped_node_name] = {'ros__parameters': param_dict}

        # Save the dictionary to the file
        with open(filepath, 'w') as file:
            yaml.dump(params, file)

    @pyqtSlot(QModelIndex, QModelIndex)
    def _onModelChange(self, topLeft, bottomRight):
        # Check if the value is a number
        try:
            value = float(topLeft.data())
        except ValueError:
            self._paramClient.print_warning('Invalid value type, please enter a number.')

            # Restore the previous value
            self.paramTableView.model().dataChanged.disconnect(self._onModelChange)
            self.paramTableView.model().itemFromIndex(topLeft).setText(str(
                self._valueStack[(self._currentGroupKey,
                                  self._models[self._currentGroupKey].item(topLeft.row(), 0).text())][-1]))
            self.paramTableView.model().dataChanged.connect(self._onModelChange)
            return

        # Set the new value
        param = self._models[self._currentGroupKey].item(topLeft.row(), 0).text()
        self._paramClient.set_param(self._currentGroupKey, param, value)

        # Add the new value to the previous values list
        if self._addChangedValuesToHist:
            self._valueStack[(self._currentGroupKey, param)].append(value)

        # Update the buttons with the new previous value
        # Creating all new buttons is inefficient, but it is the easiest and most consistent way to update the values
        self._createTableButtons()

    def registerPlotSwapCallback(self, callback: callable) -> None:
        self._plotSwapCallback = callback
