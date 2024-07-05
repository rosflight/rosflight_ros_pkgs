from rqt_gui_py.plugin import Plugin

from .param_tuning_widget import ParamTuningWidget

class ParamTuning(Plugin):

    def __init__(self, context):
        super(ParamTuning, self).__init__(context)
        self.setObjectName('ParamTuning')

        self._context = context
        self._node = context.node
        self._args = self._parse_args(context.argv())
        self._widget = ParamTuningWidget()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        return None
