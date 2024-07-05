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

        context.add_widget(self._widget)

    def _parse_args(self, argv):
        return None

