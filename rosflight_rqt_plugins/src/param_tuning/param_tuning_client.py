import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType, Parameter
from rcl_interfaces.srv import SetParameters, GetParameters


class ParameterClient(Node):
    def __init__(self, config: dict):
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

    def get_param(self, node_name: str, param_name: str) -> float:
        request = GetParameters.Request()
        request.names = [param_name]
        future = self.get_clients[node_name].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if len(future.result().values) == 0:
                self.get_logger().error(f'Parameter {param_name} not found')
                return 0.0
            if future.result().values[0].type == ParameterType.PARAMETER_DOUBLE:
                return future.result().values[0].double_value
            else:
                self.get_logger().error(f'Unsupported parameter type for {param_name}, only double is supported')
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))

    def set_param(self, node_name: str, param_name: str, value: float) -> None:
        request = SetParameters.Request()
        parameter = Parameter(param_name, Parameter.Type.DOUBLE, value)
        request.parameters.append(parameter.to_parameter_msg())
        future = self.set_clients[node_name].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().results[0].successful:
                self.get_logger().info(f'Successfully set {param_name} to {value}')
            else:
                self.get_logger().error(f'Failed to set {param_name} to {value}: {future.result().results[0].reason}')
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))

    def print_info(self, message: str):
        self.get_logger().info(message)

    def print_warning(self, message: str):
        self.get_logger().warning(message)

    def print_error(self, message: str):
        self.get_logger().error(message)

    def print_fatal(self, message: str):
        self.get_logger().fatal(message)