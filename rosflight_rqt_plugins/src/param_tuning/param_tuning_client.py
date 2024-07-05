from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters


class ParameterClient(Node):
    def __init__(self, config):
        super().__init__('param_tuning_client')

        self.set_clients = {}
        self.get_clients = {}
        for group in config:
            node_name = config[group]['node']

            if node_name not in self.set_clients:
                self.set_clients[node_name] = self.create_client(SetParameters, f'{node_name}/set_parameters')
                while not self.set_clients[node_name].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'{node_name}/set_parameters service not available, waiting...')
                self.get_clients[node_name] = self.create_client(GetParameters, f'{node_name}/get_parameters')
                while not self.get_clients[node_name].wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'{node_name}/get_parameters service not available, waiting...')
