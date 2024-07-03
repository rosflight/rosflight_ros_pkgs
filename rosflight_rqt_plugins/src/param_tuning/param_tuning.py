import os
import rclpy
from rclpy import qos
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, ParameterEvent

from rosflight_msgs.msg import Status

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow, QWidget, QSizePolicy, QHBoxLayout, QVBoxLayout, QRadioButton, QPushButton, QLabel, QLayout, QSlider, QDoubleSpinBox
from python_qt_binding import QtCore, QtGui

from ament_index_python import get_resource

class ParameterClient(Node):
    def __init__(self):
        super().__init__('tuning_gui_parameter_client')

        self.get_client = self.create_client(GetParameters, '/autopilot/get_parameters')
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GET_PARAMETERS service not available, waiting again... is ROSplane autopilot running?')
        self.get_req = GetParameters.Request()

        self.set_client = self.create_client(SetParameters, '/autopilot/set_parameters')
        while not self.get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SET_PARAMETERS service not available, waiting again... is ROSplane autopilot running?')
        self.set_req = SetParameters.Request()

    def get_autopilot_params(self, param_names:list[str]):
        self.get_req.names = param_names
        self.get_future = self.get_client.call_async(self.get_req)
        rclpy.spin_until_future_complete(self, self.get_future)
        return self.get_future.result()
    
    def set_autopilot_params(self, param_names:list[str], param_values:list[float]):
        assert len(param_values) == len(param_names)
        # Create list of Parameter objects
        parameters = []
        for i in range(len(param_names)):
            newParamValue = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=param_values[i])
            parameters.append(Parameter(name=param_names[i], value=newParamValue))
        self.set_req.parameters = parameters
        self.set_future = self.set_client.call_async(self.set_req)
        rclpy.spin_until_future_complete(self, self.set_future)
        return self.set_future.result()


class ROSflightGUI(Plugin):
    colorChanged = QtCore.pyqtSignal(QtGui.QColor)

    def __init__(self, context):
        super(ROSflightGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('ROSflightGUI')
        self._context = context

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns) 

        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        _, path = get_resource('packages', 'rosplane_tuning')
        ui_file = os.path.join(path, 'share', 'rosplane_tuning', 'resource', 'ParamTuning.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ROSflightTuningUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Create ROS2 node to do node things
        self._node = ParameterClient()
        self.rc_override = False
        self._rc_override_sub = self._context.node.create_subscription(Status, '/status', self.status_sub_callback, qos.qos_profile_sensor_data)
        self._param_sub = self._context.node.create_subscription(ParameterEvent, '/parameter_events', self.parameter_event_callback, 10)

        # Set up the UI
        self.setup_UI()

    
    def setup_UI(self):
        self.initialize_gui()
        self.connectSignalSlots()
        self.set_sizes()
    
    def initialize_gui(self):
        self.tuning_mode = ''
        self.curr_kp = 0.0
        self.curr_kd = 0.0
        self.curr_ki = 0.0

        self.temp_kp = 0.0
        self.temp_kd = 0.0
        self.temp_ki = 0.0
        
        self.undo_kp = self.curr_kp
        self.undo_kd = self.curr_kd
        self.undo_ki = self.curr_ki
        # This allows us to have different ranges for fine tuning kp, ki, and kd
        self.kp_edit_dist = 2.0
        self.ki_edit_dist = 0.5
        self.kd_edit_dist = 2.0
        # Boolean values for controlling debugging statements
        self.time = False
        self.disp = True

        # Original parameters saved at init, called with clear button
        self.call_originals()

        # Strings for the readout list
        self.ckp = self.orig_c_kp
        self.cki = self.orig_c_ki
        self.ckd = self.orig_c_kd
        self.rkp = self.orig_r_kp
        self.rki = self.orig_r_ki
        self.rkd = self.orig_r_kd
        self.pkp = self.orig_p_kp
        self.pki = self.orig_p_ki
        self.pkd = self.orig_p_kd
        self.akp = self.orig_a_kp
        self.aki = self.orig_a_ki
        self.akd = self.orig_a_kd
        self.a_t_kp = self.orig_a_t_kp
        self.a_t_ki = self.orig_a_t_ki
        self.a_t_kd = self.orig_a_t_kd
    
    def call_originals(self):
        (self.orig_c_kp, self.orig_c_ki, self.orig_c_kd) = self.get_params(['c_kp', 'c_ki', 'c_kd'])
        (self.orig_p_kp, self.orig_p_ki, self.orig_p_kd) = self.get_params(['p_kp', 'p_ki', 'p_kd'])
        (self.orig_r_kp, self.orig_r_ki, self.orig_r_kd) = self.get_params(['r_kp', 'r_ki', 'r_kd'])
        (self.orig_a_t_kp, self.orig_a_t_ki, self.orig_a_t_kd) = self.get_params(['a_t_kp', 'a_t_ki', 'a_t_kd'])
        (self.orig_a_kp, self.orig_a_ki, self.orig_a_kd) = self.get_params(['a_kp', 'a_ki', 'a_kd'])
        

    def set_sizes(self):
        self._widget.kpSlider.setMinimum(-100)
        self._widget.kpSlider.setMaximum(100)
        self._widget.kiSlider.setMinimum(-100)
        self._widget.kiSlider.setMaximum(100)
        self._widget.kdSlider.setMinimum(-100)
        self._widget.kdSlider.setMaximum(100)
        self._widget.kpSpinBox.setSingleStep(0.001)
        self._widget.kiSpinBox.setSingleStep(0.001)
        self._widget.kdSpinBox.setSingleStep(0.001)
        self._widget.kpSpinBox.setDecimals(3)
        self._widget.kiSpinBox.setDecimals(3)
        self._widget.kdSpinBox.setDecimals(3)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
    def connectSignalSlots(self):
        # This is where we define signal slots (callbacks) for when the buttons get clicked
        self._widget.CourseButton.toggled.connect(self.courseButtonCallback)
        self._widget.pitchButton.toggled.connect(self.pitchButtonCallback)
        self._widget.rollButton.toggled.connect(self.rollButtonCallback)
        self._widget.airspeedButton.toggled.connect(self.airspeedButtonCallback)
        self._widget.altitudeButton.toggled.connect(self.altitudeButtonCallback)
        self._widget.runButton.clicked.connect(self.runButtonCallback)
        self._widget.clearButton.clicked.connect(self.clearButtonCallback)
        self._widget.saveButton.clicked.connect(self.saveButtonCallback)
        self._widget.undoButton.clicked.connect(self.undoButtonCallback)
        self._widget.kpSlider.valueChanged.connect(self.kp_slider_callback)
        self._widget.kiSlider.valueChanged.connect(self.ki_slider_callback)
        self._widget.kdSlider.valueChanged.connect(self.kd_slider_callback)
        self._widget.kpSpinBox.valueChanged.connect(self.kpSpinBox_callback)
        self._widget.kiSpinBox.valueChanged.connect(self.kiSpinBox_callback)
        self._widget.kdSpinBox.valueChanged.connect(self.kdSpinBox_callback)
        # Color changer for the RC Override box
        self.colorChanged.connect(self.on_color_change)
    
    def courseButtonCallback(self):
        if self._widget.CourseButton.isChecked():
            if self.disp: print("COURSE gains selected")
            # Set the tuning mode
            self.tuning_mode = 'c'
            # Get the other parameters from ROS
            (self.curr_kp, self.curr_ki, self.curr_kd) = self.get_params([self.tuning_mode + '_kp', self.tuning_mode + '_ki', self.tuning_mode + '_kd'])
            # Set the sliders to the appropriate values
            self.set_sliders()
            self.set_SpinBoxes()
            self.set_previousVals()

    def rollButtonCallback(self):
        if self._widget.rollButton.isChecked():
            if self.disp: print("ROLL gains selected")
            # Set the tuning mode
            self.tuning_mode = 'r'
            # Get the other parameters from ROS
            (self.curr_kp, self.curr_ki, self.curr_kd) = self.get_params([self.tuning_mode + '_kp', self.tuning_mode + '_ki', self.tuning_mode + '_kd'])
            self.set_sliders()
            self.set_SpinBoxes()
            self.set_previousVals()

    def pitchButtonCallback(self):
        if self._widget.pitchButton.isChecked():
            if self.disp: print("PITCH gains selected")
            # Set the tuning mode
            self.tuning_mode = 'p'
            # Get the other parameters from ROS
            (self.curr_kp, self.curr_ki, self.curr_kd) = self.get_params([self.tuning_mode + '_kp', self.tuning_mode + '_ki', self.tuning_mode + '_kd'])
            self.set_sliders()
            self.set_SpinBoxes()
            self.set_previousVals()

    def airspeedButtonCallback(self):
        if self._widget.airspeedButton.isChecked():
            if self.disp: print("AIRSPEED gains selected")
            # Set the tuning mode
            self.tuning_mode = 'a_t'
            # Get the other parameters from ROS
            (self.curr_kp, self.curr_ki, self.curr_kd) = self.get_params([self.tuning_mode + '_kp', self.tuning_mode + '_ki', self.tuning_mode + '_kd'])
            self.set_sliders()
            self.set_SpinBoxes()
            self.set_previousVals()

    def altitudeButtonCallback(self):
        if self._widget.altitudeButton.isChecked():
            if self.disp: print("ALTITUDE gains selected")
            # Set the tuning mode
            self.tuning_mode = 'a'
            # Get the other parameters from ROS
            (self.curr_kp, self.curr_ki, self.curr_kd) = self.get_params([self.tuning_mode + '_kp', self.tuning_mode + '_ki', self.tuning_mode + '_kd'])
            self.set_sliders()
            self.set_SpinBoxes()
            self.set_previousVals()
        
    def get_params(self, param_names:list[str]):
        response = self._node.get_autopilot_params(param_names)
        return [par_val.double_value for par_val in response.values]

    def set_sliders(self):
        # Sliders have an integer range. Set this from +- 100
        self._widget.kpSlider.setValue(0)
        self._widget.kiSlider.setValue(0)
        self._widget.kdSlider.setValue(0)

    def set_SpinBoxes(self):
        # Sliders have an integer range. Set this from +- 100
        self._widget.kpSpinBox.setMinimum(self.curr_kp - self.kp_edit_dist)
        self._widget.kpSpinBox.setMaximum(self.curr_kp + self.kp_edit_dist)
        self._widget.kpSpinBox.setValue(self.curr_kp)

        self._widget.kiSpinBox.setMinimum(self.curr_ki - self.ki_edit_dist)
        self._widget.kiSpinBox.setMaximum(self.curr_ki + self.ki_edit_dist)
        self._widget.kiSpinBox.setValue(self.curr_ki)

        self._widget.kdSpinBox.setMinimum(self.curr_kd - self.kd_edit_dist)
        self._widget.kdSpinBox.setMaximum(self.curr_kd + self.kd_edit_dist)
        self._widget.kdSpinBox.setValue(self.curr_kd)

    def kp_slider_callback(self):
        slider_val = self._widget.kpSlider.value()
        self.temp_kp = self.curr_kp + self.kp_edit_dist * slider_val / 100
        # if self.disp: print(self.temp_kp)
        self._widget.kpSpinBox.setValue(self.temp_kp)
    
    def ki_slider_callback(self):
        slider_val = self._widget.kiSlider.value()
        self.temp_ki = self.curr_ki + self.ki_edit_dist * slider_val / 100
        # if self.disp: print(self.temp_ki)
        self._widget.kiSpinBox.setValue(self.temp_ki)
   
    def kd_slider_callback(self):
        slider_val = self._widget.kdSlider.value()
        self.temp_kd = self.curr_kd + self.kd_edit_dist * slider_val / 100
        # if self.disp: print(self.temp_kd)
        self._widget.kdSpinBox.setValue(self.temp_kd)

    def kpSpinBox_callback(self):
        kpSpinBox_value = self._widget.kpSpinBox.value()
        self.temp_kp = kpSpinBox_value
        # slider_val = (self.temp_kp - self.curr_kp)*100/self.kp_edit_dist
        # self._widget.kpSlider.setValue(int(slider_val))

    def kiSpinBox_callback(self):
        kiSpinBox_value = self._widget.kiSpinBox.value()
        self.temp_ki = kiSpinBox_value
        # slider_val = (self.temp_ki - self.curr_ki)*100/self.ki_edit_dist
        # self._widget.kiSlider.setValue(int(slider_val))

    def kdSpinBox_callback(self):
        kdSpinBox_value = self._widget.kdSpinBox.value()
        self.temp_kd = kdSpinBox_value
        # slider_val = (self.temp_kd - self.curr_kd)*100/self.kd_edit_dist
        # self._widget.kdSlider.setValue(int(slider_val))
    
    def runButtonCallback(self, *args, mode=None):
        # call this if run button is pushed
        # Set the undo values to be the current values
        self.undo_kp = self.curr_kp
        self.undo_ki = self.curr_ki
        self.undo_kd = self.curr_kd
        # Set current variables to be temp variables
        self.curr_kp = self.temp_kp
        self.curr_ki = self.temp_ki
        self.curr_kd = self.temp_kd
        #execute ros param set functions
        if mode is not None: 
            tun_mode = mode
        else:
            tun_mode = self.tuning_mode

        if self.disp: print('RUNNING parameters')

        param_names = [tun_mode+'_kp', tun_mode+'_ki', tun_mode+'_kd']
        print(param_names)
        param_vals = [self.curr_kp, self.curr_ki, self.curr_kd]
        result = self._node.set_autopilot_params(param_names, param_vals)

        for msg in result.results:
            if not msg.successful:
                print(f'WARNING: {msg} was not successful')

        if self.disp:
            print('Kp set to:', self.curr_kp)
            print('Ki set to:', self.curr_ki)
            print('Kd set to:', self.curr_kd)
        
        # Reinitialize the gui
        self.set_sliders()
        self.set_SpinBoxes()
        self.set_previousVals()
    
    # Resets the current mode's inputs to original or last save values
    def clearButtonCallback(self):
        # for mode in ['c', 'p', 'r', 'a', 'a_t']:
        #     if self.disp: print('\nCLEARING <' + mode + '> parameters')
        #     params = ['p','i','d']
        #     for param in params:
        #         orig_var_name = f"orig_{mode}_k{param}"
        #             #get parameter values for orig_var_name
        #         original_value = getattr(self,orig_var_name)
        #             #generate curr param variable names
        #         curr_var_name = f"temp_k{param}"
        #             #Assign original values to curr parameters
        #         setattr(self, curr_var_name, float(original_value))
        #         print(f'{curr_var_name} set to {original_value}')
        #     #run button callback to apply changes
        #     self.tuning_mode = mode
        #     self.runButtonCallback()
        # reset current values to prevent the undos from being wild
        self.curr_kd = 0.0
        self.curr_ki = 0.0
        self.curr_kp = 0.0

        temporary_kp = 0.0
        temporary_ki = 0.0
        temporary_kd = 0.0

        self.temp_kp = self.orig_c_kp
        self.temp_ki = self.orig_c_ki
        self.temp_kd = self.orig_c_kd
        if self.tuning_mode == 'c':
            temporary_kp = self.temp_kp
            temporary_ki = self.temp_ki
            temporary_kd = self.temp_kd
        self.runButtonCallback(mode='c')

        self.temp_kp = self.orig_r_kp
        self.temp_ki = self.orig_r_ki
        self.temp_kd = self.orig_r_kd
        if self.tuning_mode == 'r':
            temporary_kp = self.temp_kp
            temporary_ki = self.temp_ki
            temporary_kd = self.temp_kd
        self.runButtonCallback(mode='r')
        
        self.temp_kp = self.orig_p_kp
        self.temp_ki = self.orig_p_ki
        self.temp_kd = self.orig_p_kd
        if self.tuning_mode == 'p':
            temporary_kp = self.temp_kp
            temporary_ki = self.temp_ki
            temporary_kd = self.temp_kd
        self.runButtonCallback(mode='p')

        self.temp_kp = self.orig_a_kp
        self.temp_ki = self.orig_a_ki
        self.temp_kd = self.orig_a_kd
        if self.tuning_mode == 'a':
            temporary_kp = self.temp_kp
            temporary_ki = self.temp_ki
            temporary_kd = self.temp_kd
        self.runButtonCallback(mode='a')

        self.temp_kp = self.orig_a_t_kp
        self.temp_ki = self.orig_a_t_ki
        self.temp_kd = self.orig_a_t_kd
        if self.tuning_mode == 'a_t':
            temporary_kp = self.temp_kp
            temporary_ki = self.temp_ki
            temporary_kd = self.temp_kd
        self.runButtonCallback(mode='a_t')

        # Reset the current values on the sliders to match the current tuning mode
        self.temp_kp = temporary_kp
        self.temp_ki = temporary_ki
        self.temp_kd = temporary_kd
        self.runButtonCallback()

    def saveButtonCallback(self):
        if self.disp: print('\nSAVING all parameters')
        self.call_originals()
        self.clearButtonCallback()
    
    def undoButtonCallback(self):
        self.temp_kp = self.undo_kp
        self.temp_ki = self.undo_ki
        self.temp_kd = self.undo_kd
        self.runButtonCallback()
    
    def set_previousVals(self):
        self._widget.kp_prev_val.setText(f'Previous Value: {self.undo_kp}')
        self._widget.ki_prev_val.setText(f'Previous Value: {self.undo_ki}')
        self._widget.kd_prev_val.setText(f'Previous Value: {self.undo_kd}')
        if self.tuning_mode == 'p':
            self._widget.kp_saved_val.setText(f'Saved Value: {self.orig_p_kp}')
            self._widget.ki_saved_val.setText(f'Saved Value: {self.orig_p_ki}')
            self._widget.kd_saved_val.setText(f'Saved Value: {self.orig_p_kd}')
        elif self.tuning_mode == 'c':
            self._widget.kp_saved_val.setText(f'Saved Value: {self.orig_c_kp}')
            self._widget.ki_saved_val.setText(f'Saved Value: {self.orig_c_ki}')
            self._widget.kd_saved_val.setText(f'Saved Value: {self.orig_c_kd}')
        elif self.tuning_mode == 'r':
            self._widget.kp_saved_val.setText(f'Saved Value: {self.orig_r_kp}')
            self._widget.ki_saved_val.setText(f'Saved Value: {self.orig_r_ki}')
            self._widget.kd_saved_val.setText(f'Saved Value: {self.orig_r_kd}')
        elif self.tuning_mode == 'a':
            self._widget.kp_saved_val.setText(f'Saved Value: {self.orig_a_kp}')
            self._widget.ki_saved_val.setText(f'Saved Value: {self.orig_a_ki}')
            self._widget.kd_saved_val.setText(f'Saved Value: {self.orig_a_kd}')
        elif self.tuning_mode == 'a_t':
            self._widget.kp_saved_val.setText(f'Saved Value: {self.orig_a_t_kp}')
            self._widget.ki_saved_val.setText(f'Saved Value: {self.orig_a_t_ki}')
            self._widget.kd_saved_val.setText(f'Saved Value: {self.orig_a_t_kd}')

    def status_sub_callback(self, msg):
        if msg.rc_override:
            if not self.rc_override:
                self.rc_override = True
                print('RC OVERRIDE ?!?!?!?')
            self.colorChanged.emit(QtGui.QColor(219, 44, 44))
        #     self._widget.rcOverrideLabel.setStyleSheet("QLabel {background-color: red; color: white}")
        else:
            if self.rc_override:
                self.rc_override = False
                print('not rc override')
            self.colorChanged.emit(QtGui.QColor(87, 225, 97))
        #     self._widget.rcOverrideLabel.setStyleSheet("QLabel {background-color: green; color: white}")
    
    @QtCore.pyqtSlot(QtGui.QColor)
    def on_color_change(self, color):
        self._widget.rcOverrideLabel.setStyleSheet("QLabel {background-color: " + color.name() + "; color: white}")

    def parameter_event_callback(self,msg):
        # Look only at the autopilot node
        if msg.node == '/autopilot':
            for param in msg.changed_parameters:
                if param.name == 'c_kp':
                    self.ckp = param.value.double_value
                elif param.name == 'c_ki':
                    self.cki = param.value.double_value
                elif param.name == 'c_kd':
                    self.ckd = param.value.double_value
                elif param.name == 'r_kp':
                    self.rkp = param.value.double_value
                elif param.name == 'r_ki':
                    self.rki = param.value.double_value
                elif param.name == 'r_kd':
                    self.rkd = param.value.double_value
                elif param.name == 'p_kp':
                    self.pkp = param.value.double_value
                elif param.name == 'p_ki':
                    self.pki = param.value.double_value
                elif param.name == 'p_kd':
                    self.pkd = param.value.double_value
                elif param.name == 'a_kp':
                    self.akp = param.value.double_value
                elif param.name == 'a_ki':
                    self.aki = param.value.double_value
                elif param.name == 'a_kd':
                    self.akd = param.value.double_value
                elif param.name == 'a_t_kp':
                    self.a_t_kp = param.value.double_value
                elif param.name == 'a_t_ki':
                    self.a_t_ki = param.value.double_value
                elif param.name == 'a_t_kd':
                    self.a_t_kd = param.value.double_value
            
            # Update the text boxes. If updating all of them is too slow, set them one at a time
            self._widget.courseReadout.setText(f'c_kp: {self.ckp}\nc_ki: {self.cki}\nc_kd: {self.ckd}')
            self._widget.rollReadout.setText(f'r_kp: {self.rkp}\nr_ki: {self.rki}\nr_kd: {self.rkd}')
            self._widget.pitchReadout.setText(f'p_kp: {self.pkp}\np_ki: {self.pki}\np_kd: {self.pkd}')
            self._widget.altitudeReadout.setText(f'a_kp: {self.akp}\na_ki: {self.aki}\na_kd: {self.akd}')
            self._widget.airspeedReadout.setText(f'a_t_kp: {self.a_t_kp}\na_t_ki: {self.a_t_ki}\na_t_kd: {self.a_t_kd}')