#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import numpy as np
from scipy.spatial.transform import Rotation
from rosflight_msgs.msg import SimState, RangeFinderSensor, RGBCamera
import time
from std_srvs.srv import Trigger
from holoocean_interface import HolooceanInterface


class HoloOceanNode(Node):
    """
    ROS2 Node wrapping a Holoocean simulation environment. Subscribes to /sim/truth_state
    to update the agent state, and publishes sensor data to appropriate topics.
    """
    def __init__(self):
        """
        Attributes:
            agent (str): The main agent in the scenario
            env (str): The holoocean environment
            interface (HolooceanInterface): The Holoocean interface instance
        """
        super().__init__('holoocean_node')

        # Initialize agent and environment parameters.
        self.declare_parameter('agent', 'fixedwing')
        self.agent = self.get_parameter('agent').get_parameter_value().string_value
        self.declare_parameter('env', 'default')
        self.env = self.get_parameter('env').get_parameter_value().string_value

        # --- Collision / state sharing between ROS callback thread and sim thread ---
        self._state_lock = threading.Lock()
        self._latest_velocity = np.zeros(3, dtype=float)

        # Debounce/cooldown so we don't spam resets on consecutive ticks
        self._last_reset_time = 0.0
        self._reset_cooldown_s = 1.0  # tweak if needed

        # Construct scenario path.
        scenario_file = f'{self.env}_{self.agent}.json'
        scenario_path = os.path.join(
            get_package_share_directory('rosflight_sim'),
            'config',
            scenario_file
        )

        # Error handling for missing scenario file.
        if not Path(scenario_path).is_file():
            self.get_logger().error(f'Scenario file not found: {scenario_path}')
            raise FileNotFoundError(f'Scenario file not found: {scenario_path}')
        self.get_logger().info(f'Using scenario file: {scenario_path}')

        # Viewport and render quality parameters.
        self.declare_parameter('show_viewport', True)
        show_viewport = self.get_parameter('show_viewport').get_parameter_value().bool_value
        self.declare_parameter('render_quality', -1)
        render_quality = self.get_parameter('render_quality').get_parameter_value().integer_value
        if render_quality == -1:
            render_quality = None

        # Initialize Holoocean interface.
        self.interface = HolooceanInterface(scenario_path, tps=30, show_viewport=show_viewport, render_quality=render_quality)
        self.get_logger().info('Holoocean interface initialized.')

        # Create services.
        self.holoocean_reset = self.create_service(Trigger, 'holoocean_reset', self.reset)

        # Initialize subscriber to truth state and publishers for sensors.
        self.truth_state_sub = self.create_subscription(SimState, '/sim/truth_state', self.truth_state_callback, 10)
        self.RGBCamera_pub = self.create_publisher(RGBCamera, '/rgb_camera_sensor', 10)
        self.Horizontal_Range_pub = self.create_publisher(RangeFinderSensor, '/range_finder_sensor', 10)
        self.Ground_Range_pub = self.create_publisher(RangeFinderSensor, '/ground_range_sensor', 10)
        self.ros_publish = self.interface.ros_publish

        # Start simulation thread.
        self._sim_running = True
        self.tick_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.tick_thread.start()

        self.get_logger().info('HoloOcean simulation thread started.')

    def sim_loop(self):
        """
        Thread loop to continuously advance the Holoocean simulation, publishing sensor data.
        """
        try:
            while self._sim_running:
                # Advance simulation and get sensor data.
                sensors_dict = self.interface.tick()

                if self.ros_publish:
                    # Publish each sensor's data.
                    for sensor_name, sensor_data in sensors_dict.items():
                        self.publish_sensor(sensor_name, sensor_data)

                    # Detect collision using range finder data + latest velocity
                    ground_range = sensors_dict["GroundRange"] if "GroundRange" in sensors_dict else None
                    horizontal_range = sensors_dict["HorizontalRange"] if "HorizontalRange" in sensors_dict else None

                    # Pull latest velocity once per tick (thread-safe)
                    with self._state_lock:
                        velocity = self._latest_velocity.copy()

                    if ground_range is not None and horizontal_range is not None:
                        if self.detect_collision(velocity, ground_range, horizontal_range):
                            self.get_logger().info('Collision detected and environment reset.')
                    else:
                        print(100*"\n")


        except Exception as e:
            self.get_logger().error(f"Sim loop error: {e}")

    def destroy_node(self):
        """
        Override node destruction for clean shutdown of simulation thread.
        """
        self._sim_running = False
        if getattr(self, "tick_thread", None) and self.tick_thread.is_alive():
            self.tick_thread.join(timeout=2.0)
        super().destroy_node()

    def reset(self, request, response):
        """
        Service callback to reset the holoocean environment.
        """
        self.interface.reset_environment()
        response.success = True
        response.message = 'Resetting the HoloOcean Environment'
        return response
    
    def detect_collision(self, velocity, ground_range, horizontal_range, distance_threshold=0.5, speed_threshold=20.0):
        """
        Detect collision based on proximity + speed.

        - Uses min of ground/horizontal ranges without concatenation.
        - Ignores empty, None, NaN/inf, or non-positive ranges.
        - Debounces resets with a cooldown timer.
        """

        # Cooldown gate to prevent repeated resets on consecutive ticks
        now = time.monotonic()
        if (now - self._last_reset_time) < self._reset_cooldown_s:
            return False

        # Compute speed (guard against bad velocity)
        try:
            speed = float(np.linalg.norm(velocity))
        except Exception:
            return False

        if speed < speed_threshold:
            return False

        min_ground = np.min(ground_range)
        min_horizontal = np.min(horizontal_range)
        min_distance = min(min_ground, min_horizontal)

        if min_distance < distance_threshold:
            self.get_logger().warn(
            f'Collision detected (speed={speed:.3f} m/s). Resetting environment.'
            )
            self._last_reset_time = now
            rclpy.shutdown()
            self.destroy_node()
            return True

        return False


    def _quat_to_euler(self, q_xyzw):
        """
        Helper function converting quaternion [x,y,z,w] to Euler angles [roll, pitch, yaw] in degrees
        according to HoloOcean's coordinates (FLU: x fwd, y left, z up).

        Parameters:
            q_xyzw (list or ndarray): Quaternion in [x, y, z, w] format
        Returns:
            euler_angles (ndarray): Euler angles [roll, pitch, yaw] in degrees
        """
        r = Rotation.from_quat(q_xyzw)
        return r.as_euler('xyz', degrees=True)

    def truth_state_callback(self, msg: SimState):
        """
        Callback to update the agent's state in the Holoocean simulation.

        Parameters:
            msg (SimState): The truth state message containing pose and velocity information
        """
        # Extract pose and velocities using the helper function
        location, rotation, velocity, angular_velocity = self.extract_state(msg)

        # Share latest velocity with sim thread (thread-safe)
        with self._state_lock:
            self._latest_velocity = velocity

        try:
            # Update agent state in the Holoocean simulation.
            self.interface.set_agent_state(location, rotation, velocity, angular_velocity)
        except KeyError as e:
            self.get_logger().error(str(e))
        except Exception as e:
            self.get_logger().error(f"Error setting agent state: {e}")

    def extract_state(self, msg: SimState):
        """
        Helper function converting SimState message to Holoocean-compatible state arrays.

        Parameters:
            msg (SimState): The truth state message containing pose and velocity information
        Returns:
            location (ndarray): Position array [x, y, z]
            rotation (ndarray): Orientation array [roll, pitch, yaw] in degrees
            velocity (ndarray): Linear velocity array [vx, vy, vz]
            angular_velocity (ndarray): Angular velocity array [wx, wy, wz] in degrees"""
        
        # Extract position and orientation
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Extract linear and angular velocities
        vx = msg.twist.linear.x
        vy = -msg.twist.linear.y
        vz = -msg.twist.linear.z
        wx = msg.twist.angular.x
        wy = -msg.twist.angular.y
        wz = -msg.twist.angular.z

        # Convert to Holoocean coordinate system.
        location = np.array([x, -y, -z])
        rotation = self._quat_to_euler([qx, -qy, -qz, qw])
        velocity = np.array([vx, -vy, -vz])
        angular_velocity = np.degrees(np.array([wx, -wy, -wz]))

        return location, rotation, velocity, angular_velocity
    
    def publish_sensor(self, sensor_name, sensor_data=None):
        """
        Publish sensor data to the appropriate ROS2 topic.
        Parameters:
            sensor_name (str): Name of the sensor to publish data from
            sensor_data: Optional pre-fetched sensor data. If None, retrieves from interface."""
        # Retrieve sensor data if not provided.
        if sensor_data is None:
            sensor_data = self.interface.sensor_callback(sensor_name)

        # Initialize an empty message and create it based on sensor type.
        msg = None
        if sensor_name == 'RGBCamera':  
            msg = RGBCamera()
            msg.timestamp = self.get_clock().now().nanoseconds
            msg.width = 512
            msg.height = 512
            msg.channels = 4
            msg.image = sensor_data.ravel().tolist()
            self.RGBCamera_pub.publish(msg)

        if sensor_name == 'GroundRange':
            msg = RangeFinderSensor()
            msg.distances = sensor_data.ravel().tolist()
            msg.angles = []
            self.Ground_Range_pub.publish(msg)

        elif sensor_name == 'HorizontalRange':
            msg = RangeFinderSensor()
            msg.distances = sensor_data.ravel().tolist()
            msg.angles = []
            self.Horizontal_Range_pub.publish(msg)

def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = HoloOceanNode()
        
        rclpy.spin(node)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

