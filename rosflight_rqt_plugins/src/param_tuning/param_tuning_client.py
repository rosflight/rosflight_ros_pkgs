import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
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

    def get_param(self, node_name, param_name):
        request = GetParameters.Request()
        request.names = [param_name]
        future = self.get_clients[node_name].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if len(future.result().values) == 0:
                self.get_logger().error(f'Parameter {param_name} not found')
                return
            if future.result().values[0].type == ParameterType.PARAMETER_DOUBLE:
                return future.result().values[0].double_value
            else:
                self.get_logger().error(f'Unsupported parameter type for {param_name}, only double is supported')
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))
