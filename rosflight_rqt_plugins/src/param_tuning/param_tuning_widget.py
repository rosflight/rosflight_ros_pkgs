import os
import yaml

from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSlot, QModelIndex
from python_qt_binding.QtGui import QStandardItemModel, QStandardItem
from python_qt_binding.QtWidgets import QWidget, QPushButton, QFileDialog


class ParamTuningWidget(QWidget):
    def __init__(self, config: dict, paramClient, param_filepath: str):
        # Initialize widget
        super(ParamTuningWidget, self).__init__()
        self.setObjectName('ParamTuningWidget')
        self._param_file_path = param_filepath
        self._add_changed_values_to_hist = True
        self._plot_swap_callback = lambda x: None

        # Load the UI file
        _, path = get_resource('packages', 'rosflight_rqt_plugins')
        ui_file = os.path.join(path, 'share', 'rosflight_rqt_plugins', 'resources', 'param_tuning.ui')
        loadUi(ui_file, self)

        # Define table formatting
        self._config = config
        self._table_headers = ['Parameter', 'Value', 'Description', 'Reset to Previous', 'Reset to Initial']
        self._table_widths = [175, 125, 500, 250, 250]

        # Get the original values of the parameters
        self._param_client = paramClient
        self._value_stack = {}
        for group in config:
            for param in config[group]['params']:
                value = self._param_client.get_param(group, param)
                self._value_stack[(group, param)] = [value]

        # Set up the widget
        # Group selection - QComboBox
        self.group_selection.addItems(self._config.keys())
        self.group_selection.currentTextChanged.connect(self._group_selection_callback)
        # Refresh button - QPushButton
        self.refresh_button.clicked.connect(self._refresh_button_callback)
        # Save to file button - QPushButton
        self.save_button.clicked.connect(self._save_button_callback)
        # Parameter table - QTableView
        self._setupTableModels()
        self._create_table_buttons()
        self._refresh_table_values()

    def _setupTableModels(self):
        # Create a model for every group
        self._models = {}
        self._current_group_key = list(self._config.keys())[0]
        for group in self._config:
            model = QStandardItemModel()
            model.setHorizontalHeaderLabels(self._table_headers)
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
        self.param_table_view.setModel(self._models[self._current_group_key])

        # Set the column widths
        for i, width in enumerate(self._table_widths):
            self.param_table_view.setColumnWidth(i, width)

        # Hide the number row
        self.param_table_view.verticalHeader().hide()

        # Connect the model change signal
        self.param_table_view.model().dataChanged.connect(self._on_model_change)

    def _create_table_buttons(self):
        # Get a list of gains for the current group
        current_group = self._config[self._current_group_key]
        current_params = list(current_group['params'].keys())

        for i, param in enumerate(current_params):
            # Create reset to previous buttons
            previous_button_value = self._value_stack[(self._current_group_key, param)][-2] \
                if len(self._value_stack[(self._current_group_key, param)]) > 1 \
                else self._value_stack[(self._current_group_key, param)][0]

            button = QPushButton(str(previous_button_value))
            button.clicked.connect(
                lambda _, g=self._current_group_key, index=i, p=param: self._reset_previous_button_callback(g, index, p)
            )
            index = self.param_table_view.model().index(i, 3)
            self.param_table_view.setIndexWidget(index, button)

            # Create reset to original buttons
            button = QPushButton(str(self._value_stack[(self._current_group_key, param)][0]))
            button.clicked.connect(
                lambda _, g=self._current_group_key, index=i, p=param: self._reset_initial_button_callback(g, index, p)
            )
            index = self.param_table_view.model().index(i, 4)
            self.param_table_view.setIndexWidget(index, button)

    def _reset_previous_button_callback(self, group, row, param):
        # Pop the last value from list, unless it is the last value
        if len(self._value_stack[(group, param)]) > 1:
            self._value_stack[(group, param)].pop()
        value = self._value_stack[(group, param)][-1]

        # Update the table
        self._add_changed_values_to_hist = False
        self._models[group].item(row, 1).setText(str(value))
        self._add_changed_values_to_hist = True

    def _reset_initial_button_callback(self, group, row, param):
        self._models[group].item(row, 1).setText(str(self._value_stack[(group, param)][0]))
        self._value_stack[(group, param)] = [self._value_stack[(group, param)][0]]
        self._create_table_buttons()

    def _refresh_table_values(self):
        self._param_client.print_info('Getting all parameters from ROS network...')

        # Temporarily disconnect the model change signal
        self.param_table_view.model().dataChanged.disconnect(self._on_model_change)

        # Get current values of the parameters
        for group in self._config:
            for i in range(self._models[group].rowCount()):
                param = self._models[group].item(i, 0).text()
                value = self._param_client.get_param(group, param)

                # If the value is different from the previous value, add it to the stack and update the buttons
                if value != self._value_stack[(group, param)][-1]:
                    self._value_stack[(group, param)].append(value)
                    self._create_table_buttons()

                # Update the table
                self._models[group].item(i, 1).setText(str(value))

        # Reconnect the model change signal
        self.param_table_view.model().dataChanged.connect(self._on_model_change)

    def _group_selection_callback(self, text):
        self._current_group_key = text

        # Swap models and connect the model change signal to the new model
        self.param_table_view.model().dataChanged.disconnect(self._on_model_change)
        self.param_table_view.setModel(self._models[text])
        self.param_table_view.model().dataChanged.connect(self._on_model_change)

        # Update table
        self._create_table_buttons()

        # Tell the plotter to swap the plot
        self._plot_swap_callback(text)

    def _refresh_button_callback(self):
        self._refresh_table_values()

    def _save_button_callback(self):
        # Request a filepath is a param filepath hasn't already been given
        if self._param_file_path is None:
            options = QFileDialog.Options()
            filepath, _ = QFileDialog.getSaveFileName(None, 'Save Parameters to ROS .yaml', '', 'YAML Files (*.yaml)',
                                                      options=options)
            if not filepath:
                self._param_client.print_warning('No file selected, parameters not saved.')
                return
        else:
            filepath = self._param_file_path

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
                param_dict[param_name] = self._param_client.get_param(group, param_name, False)

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
    def _on_model_change(self, top_left, bottom_right):
        # Check if the value is a number
        try:
            value = float(top_left.data())
        except ValueError:
            self._param_client.print_warning('Invalid value type, please enter a number.')

            # Restore the previous value
            self.param_table_view.model().dataChanged.disconnect(self._on_model_change)
            self.param_table_view.model().itemFromIndex(top_left).setText(str(
                self._value_stack[(self._current_group_key,
                                   self._models[self._current_group_key].item(top_left.row(), 0).text())][-1]))
            self.param_table_view.model().dataChanged.connect(self._on_model_change)
            return

        # Set the new value
        param = self._models[self._current_group_key].item(top_left.row(), 0).text()
        self._param_client.set_param(self._current_group_key, param, value)

        # Add the new value to the previous values list
        if self._add_changed_values_to_hist:
            self._value_stack[(self._current_group_key, param)].append(value)

        # Update the buttons with the new previous value
        # Creating all new buttons is inefficient, but it is the easiest and most consistent way to update the values
        self._create_table_buttons()

    def register_plot_swap_callback(self, callback: callable) -> None:
        self._plot_swap_callback = callback
