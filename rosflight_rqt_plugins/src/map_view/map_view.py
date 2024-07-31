import rclpy
from rqt_gui_py.plugin import Plugin


class MapView(Plugin):
    def __init__(self, context):
        super(MapView, self).__init__(context)
        self.setObjectName('MapView')

        self._context = context
        self._node = context.node

        print("Hello, World!")

    @staticmethod
    def add_arguments(parser):
        pass

    def shutdown_plugin(self):
        self._node.destroy_node()
        rclpy.shutdown()
