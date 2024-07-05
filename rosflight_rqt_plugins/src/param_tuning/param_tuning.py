import yaml

from rqt_gui_py.plugin import Plugin

from .param_tuning_widget import ParamTuningWidget
from .param_tuning_client import ParameterClient

class ParamTuning(Plugin):

    def __init__(self, context):
        super(ParamTuning, self).__init__(context)
        self.setObjectName('ParamTuning')

        self._context = context
        self._node = context.node

        # Load the configuration file
        filepath = '/rosflight_ws/src/rosflight_ros_pkgs/rosflight_rqt_plugins/resources/config.yaml'
        with open(filepath, 'r') as file:
            self._config = yaml.safe_load(file)

        # Initialize the ROS client
        self._client = ParameterClient(self._config)

        # Initialize the widget
        self._widget = ParamTuningWidget(self._config)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
