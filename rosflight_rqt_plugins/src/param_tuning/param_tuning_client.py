import re
import time
from collections import deque
from rclpy.node import Node
from rclpy.parameter import ParameterType, Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rosidl_runtime_py.utilities import get_message


class ParameterClient():
    def __init__(self, config: dict, node: Node, hist_duration: float):
        self._config = config
        self._node = node
        self._hist_duration = hist_duration
        # Minimum time between data points accepted, to reduce amount of data to plot
        self._min_delta_time = hist_duration / 250

        # Initialize parameter clients
        self._set_clients = {}
        self._get_clients = {}
        for group in config:
            node_name = config[group]['node']
            if node_name not in self._set_clients:
                self._set_clients[node_name] = self._node.create_client(SetParameters, f'{node_name}/set_parameters')
                while not self._set_clients[node_name].wait_for_service(timeout_sec=1.0):
                    self._node.get_logger().info(f'{node_name}/set_parameters service not available, waiting...')
                self._get_clients[node_name] = self._node.create_client(GetParameters, f'{node_name}/get_parameters')
                while not self._get_clients[node_name].wait_for_service(timeout_sec=1.0):
                    self._node.get_logger().info(f'{node_name}/get_parameters service not available, waiting...')

        # Initialize topics subscribers for plotting
        self._plot_subscribers = {}
        self._data_history = {}
        for group in config:
            if 'plot_topics' in config[group]:
                for topic in config[group]['plot_topics']:
                    topic_name, field_name, field_index = self._split_topic_str(config[group]['plot_topics'][topic])

                    if topic_name not in self._plot_subscribers:
                        message_type = self._get_message_type(topic_name)
                        if message_type is None:
                            self._node.get_logger().error(f'Failed to get message type for {topic_name},'
                                                         f' does the topic exist?')
                        else:
                            self._plot_subscribers[topic_name] = self._node.create_subscription(
                                message_type,
                                topic_name,
                                lambda msg, t=topic_name: self._message_callback(msg, t),
                                10
                            )
                            self._data_history[topic_name] = {(field_name, field_index): deque()}
                    else:
                        self._data_history[topic_name][(field_name, field_index)] = deque()

    def _split_topic_str(self, topic_str: str) -> tuple:
        topic_name = topic_str.split('/')[1]
        field = topic_str.split('/')[2]
        match = re.match(r"(\w+)\[(\d+)\]", field)
        if match:
            field_name = match.group(1)
            field_index = int(match.group(2))
        else:
            field_name = field
            field_index = None

        return topic_name, field_name, field_index

    def _get_message_type(self, topic_name: str):
        topic_array = self._node.get_topic_names_and_types()
        for topic in topic_array:
            if topic[0] == '/' + topic_name:
                return get_message(topic[1][0])
        return None

    def _message_callback(self, msg, topic_name: str) -> None:
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        curr_time = self._node.get_clock().now()
        curr_time = curr_time.seconds_nanoseconds()[0] + curr_time.seconds_nanoseconds()[1] * 1e-9

        # Skip storing message if time since last message is less than minimum delta time
        first_array = next(iter(self._data_history[topic_name].values()))
        if len(first_array) > 0 and msg_time - first_array[-1][1] < self._min_delta_time:
            return

        for field in self._data_history[topic_name]:
            field_name = field[0]
            field_index = field[1]

            # Add new values to array
            msg_value = getattr(msg, field_name) if field_index is None else getattr(msg, field_name)[field_index]
            self._data_history[topic_name][(field_name, field_index)].append((msg_value, msg_time))

            # Remove old values
            while curr_time - self._data_history[topic_name][(field_name, field_index)][0][1] > self._hist_duration:
                self._data_history[topic_name][(field_name, field_index)].popleft()

    def get_data(self, topic_str: str) -> tuple[list, list]:
        topic_name, field_name, field_index = self._split_topic_str(topic_str)
        x_data = []
        y_data = []
        for data in self._data_history[topic_name][(field_name, field_index)]:
            x_data.append(data[1])
            y_data.append(data[0])

        return x_data, y_data

    def get_param(self, group: str, param: str, scaled: bool = True) -> float:
        if not scaled or 'scale' not in self._config[group]['params'][param]:
            scale = 1.0
        else:
            scale = self._config[group]['params'][param]['scale']
        node_name = self._config[group]['node']

        request = GetParameters.Request()
        request.names = [param]

        future = self._get_clients[node_name].call_async(request)

        call_time = time.time()
        callback_complete = False
        while call_time + 5 > time.time():
            if future.done():
                callback_complete = True
                break
        if not callback_complete or future.result() is None:
            self._node.get_logger().error(f'Failed to get {node_name}/{param} after 5 seconds')
            return 0.0

        if future.result() is not None:
            if len(future.result().values) == 0:
                self._node.get_logger().error(f'Parameter {param} not found')
                return 0.0
            if future.result().values[0].type == ParameterType.PARAMETER_DOUBLE:
                return future.result().values[0].double_value * scale
            else:
                self._node.get_logger().error(f'Unsupported parameter type for {param}, only double is supported')
        else:
            self._node.get_logger().error('Service call failed %r' % (future.exception(),))

    def set_param(self, group: str, param: str, value: float, scaled: bool = True) -> None:
        if scaled and 'scale' in self._config[group]['params'][param]:
            value /= self._config[group]['params'][param]['scale']

        node_name = self._config[group]['node']
        request = SetParameters.Request()
        parameter = Parameter(param, Parameter.Type.DOUBLE, value)
        request.parameters.append(parameter.to_parameter_msg())
        future = self._set_clients[node_name].call_async(request)

        call_time = time.time()
        callback_complete = False
        while call_time + 5 > time.time():
            if future.done():
                callback_complete = True
                break
        if not callback_complete or future.result() is None:
            self._node.get_logger().error(f'Failed to set {node_name}/{param} after 5 seconds')

        if future.result() is not None:
            if future.result().results[0].successful:
                self._node.get_logger().info(f'Set {node_name}/{param} to {value}')
            else:
                self._node.get_logger().error(f'Failed to set {param} to {value}: {future.result().results[0].reason}')
        else:
            self._node.get_logger().error('Service call failed %r' % (future.exception(),))

    def print_info(self, message: str) -> None:
        self._node.get_logger().info(message)

    def print_warning(self, message: str) -> None:
        self._node.get_logger().warning(message)

    def print_error(self, message: str) -> None:
        self._node.get_logger().error(message)

    def print_fatal(self, message: str) -> None:
        self._node.get_logger().fatal(message)

    def shutdown(self) -> None:
        for topic in self._plot_subscribers:
            self._plot_subscribers[topic].destroy()
        for client in self._set_clients:
            self._set_clients[client].destroy()
            self._get_clients[client].destroy()