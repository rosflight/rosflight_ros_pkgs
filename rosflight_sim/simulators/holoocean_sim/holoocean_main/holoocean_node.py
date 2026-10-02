#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
import numpy as np
from scipy.spatial.transform import Rotation
from rosflight_msgs.msg import SimState, RangeFinderSensor, RGBCamera
from rosflight_msgs.msg import GNSS
from rosflight_msgs.srv import StepFirmware
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
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
        self.declare_parameter('lockstep', False)
        self.lockstep = self.get_parameter('lockstep').value
        self.declare_parameter('imu_update_frequency', 200.0)
        self.imu_update_frequency = self.get_parameter('imu_update_frequency').value

        self.camera_config = None
        if self.agent == 'fixedwing':
            self.camera_config = self.configure_camera()
        if self.lockstep and self.camera_config is None:
            raise ValueError('lockstep is only supported for the fixedwing camera launch')
        if self.lockstep and (
                self.imu_update_frequency <= 0
                or not float(self.imu_update_frequency).is_integer()
                or self.camera_config['step_hz'] % int(self.imu_update_frequency)):
            raise ValueError('camera.step_hz must be a multiple of imu_update_frequency')
        if self.lockstep and self.camera_config['step_hz'] % 10:
            raise ValueError('camera.step_hz must be a multiple of the 10 Hz GNSS rate')

        # --- Collision / state sharing between ROS callback thread and sim thread ---
        self._state_lock = threading.Lock()
        self._latest_velocity = np.zeros(3, dtype=float)
        self._interface_lock = threading.RLock()
        self._step_condition = threading.Condition()
        self._latest_truth = None
        self._latest_imu_stamp = -1
        self._latest_clock_sync_stamp = -1
        self._latest_gnss_stamp = -1
        self._latest_sensor_state_stamp = -1
        self._latest_forces_state_stamp = -1

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
        tps = self.camera_config['rate_hz'] if self.camera_config else 30
        self.interface = HolooceanInterface(
            scenario_path, tps=tps, show_viewport=show_viewport,
            render_quality=render_quality, camera_config=self.camera_config)
        self.get_logger().info('Holoocean interface initialized.')

        # Create services.
        self.holoocean_reset = self.create_service(Trigger, 'holoocean_reset', self.reset)

        # Initialize subscriber to truth state and publishers for sensors.
        self.truth_state_sub = self.create_subscription(SimState, '/sim/truth_state', self.truth_state_callback, 10)
        if self.agent != 'fixedwing':
            self.RGBCamera_pub = self.create_publisher(RGBCamera, '/rgb_camera_sensor', 10)
        self.Horizontal_Range_pub = self.create_publisher(RangeFinderSensor, '/range_finder_sensor', 10)
        self.Ground_Range_pub = self.create_publisher(RangeFinderSensor, '/ground_range_sensor', 10)
        self.ros_publish = self.interface.ros_publish

        if self.camera_config:
            self.image_pub = self.create_publisher(Image, '/fixedwing/camera/image_raw', 10)
            self.camera_info_pub = self.create_publisher(CameraInfo, '/fixedwing/camera/camera_info', 10)
            self.static_tf = StaticTransformBroadcaster(self)
            self.publish_camera_transform()

        if self.lockstep:
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            sync_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.clock_pub = self.create_publisher(Clock, '/clock', qos)
            self.imu_sub = self.create_subscription(
                Imu, '/sim/sensors/imu/data', self.imu_callback, 10)
            self.clock_sync_sub = self.create_subscription(
                Header, '/sim/clock_sync', self.clock_sync_callback, sync_qos)
            self.gnss_sub = self.create_subscription(
                GNSS, '/sim/sensors/gnss', self.gnss_callback, 10)
            self.sensor_state_sub = self.create_subscription(
                Header, '/sim/sensors/state_sync', self.sensor_state_callback, sync_qos)
            self.forces_state_sub = self.create_subscription(
                Header, '/sim/forces/state_sync', self.forces_state_callback, sync_qos)
            self.sil_client = self.create_client(StepFirmware, '/sil_board/step')

        # Start simulation thread.
        self._sim_running = True
        loop = self.lockstep_loop if self.lockstep else self.sim_loop
        self.tick_thread = threading.Thread(target=loop, daemon=True)
        self.tick_thread.start()

        self.get_logger().info('HoloOcean simulation thread started.')

    def configure_camera(self):
        defaults = {
            'width': 1024,
            'height': 1024,
            'fov_deg': 100.0,
            'exposure_method': 'AEM_Histogram',
            'exposure_compensation': 2.0,
            'rate_hz': 30,
            'step_hz': 600,
            'encoding': 'mono8',
            'location': [0.5, 0.0, -0.15],
            'rotation': [0.0, 75.0, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter('camera.' + name, value)
        config = {name: self.get_parameter('camera.' + name).value for name in defaults}
        for name in ('width', 'height', 'rate_hz', 'step_hz'):
            if type(config[name]) is not int or config[name] <= 0:
                raise ValueError(f'camera.{name} must be a positive integer')
        if not 0.0 < config['fov_deg'] < 180.0:
            raise ValueError('camera.fov_deg must be between 0 and 180 degrees')
        if config['exposure_method'] not in ('AEM_Histogram', 'AEM_Basic', 'AEM_Manual'):
            raise ValueError('camera.exposure_method must be AEM_Histogram, AEM_Basic, or AEM_Manual')
        if not -15.0 <= config['exposure_compensation'] <= 15.0:
            raise ValueError('camera.exposure_compensation must be between -15 and 15 stops')
        if config['encoding'] not in ('mono8', 'rgb8'):
            raise ValueError('camera.encoding must be mono8 or rgb8')
        if len(config['location']) != 3 or len(config['rotation']) != 3:
            raise ValueError('camera.location and camera.rotation must have three values')
        if config['step_hz'] % config['rate_hz']:
            raise ValueError('camera.step_hz must be a multiple of camera.rate_hz')
        return config

    def publish_camera_transform(self):
        config = self.camera_config
        # HoloOcean and its camera mount use FLU. ROSflight IMU data use FRD.
        flu_to_frd = np.diag([1.0, -1.0, -1.0])
        optical_to_camera = np.array([[0.0, 0.0, 1.0],
                                      [-1.0, 0.0, 0.0],
                                      [0.0, -1.0, 0.0]])
        camera_to_imu = flu_to_frd @ Rotation.from_euler(
            'xyz', config['rotation'], degrees=True).as_matrix() @ optical_to_camera
        quaternion = Rotation.from_matrix(camera_to_imu).as_quat()
        position = flu_to_frd @ np.asarray(config['location'], dtype=float)
        transform = TransformStamped()
        transform.header.frame_id = 'imu_frd'
        transform.child_frame_id = 'down_camera_optical'
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.static_tf.sendTransform(transform)

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def imu_callback(self, msg):
        with self._step_condition:
            self._latest_imu_stamp = self.stamp_ns(msg.header.stamp)
            self._step_condition.notify_all()

    def clock_sync_callback(self, msg):
        with self._step_condition:
            self._latest_clock_sync_stamp = self.stamp_ns(msg.stamp)
            self._step_condition.notify_all()

    def gnss_callback(self, msg):
        with self._step_condition:
            self._latest_gnss_stamp = self.stamp_ns(msg.header.stamp)
            self._step_condition.notify_all()

    def sensor_state_callback(self, msg):
        with self._step_condition:
            self._latest_sensor_state_stamp = self.stamp_ns(msg.stamp)
            self._step_condition.notify_all()

    def forces_state_callback(self, msg):
        with self._step_condition:
            self._latest_forces_state_stamp = self.stamp_ns(msg.stamp)
            self._step_condition.notify_all()

    def wait_for_sample(self, name, stamp_ns, clock):
        deadline = time.monotonic() + 10.0
        sample = {
            'IMU': lambda: self._latest_imu_stamp,
            'clock sync': lambda: self._latest_clock_sync_stamp,
            'GNSS': lambda: self._latest_gnss_stamp,
            'sensor state': lambda: self._latest_sensor_state_stamp,
            'forces state': lambda: self._latest_forces_state_stamp,
            'truth': lambda: self.stamp_ns(self._latest_truth.header.stamp)
            if self._latest_truth is not None else -1,
        }[name]
        while self._sim_running:
            with self._step_condition:
                if sample() >= stamp_ns:
                    if sample() != stamp_ns:
                        raise RuntimeError(f'{name} skipped simulation time {stamp_ns} ns')
                    return self._latest_truth if name == 'truth' else None
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f'{name} did not arrive for simulation time {stamp_ns} ns '
                        f'(last received {sample()} ns)')
                arrived = self._step_condition.wait_for(
                    lambda: not self._sim_running or sample() >= stamp_ns,
                    min(0.1, remaining))
            # Repeating the same time recovers a missed clock without advancing timers again.
            if not arrived:
                self.clock_pub.publish(clock)
        return None

    def lockstep_loop(self):
        try:
            deadline = time.monotonic() + 60.0
            while self._sim_running and not (
                self.sil_client.wait_for_service(timeout_sec=1.0)
                and self.count_publishers('/sim/sensors/imu/data')
                and self.count_publishers('/sim/clock_sync')
                and self.count_publishers('/sim/sensors/gnss')
                and self.count_publishers('/sim/truth_state')
                and self.count_publishers('/sim/sensors/state_sync')
                and self.count_publishers('/sim/forces/state_sync')
                and self.count_subscribers('/sim/pwm_output')
                and self.count_subscribers('/sim/forces_and_moments')
                and self.clock_pub.get_subscription_count()
            ):
                if time.monotonic() > deadline:
                    raise TimeoutError('ROSflight nodes did not connect to the lockstep clock')
            if not self._sim_running:
                return

            self.clock_pub.publish(Clock())
            step_hz = self.camera_config['step_hz']
            steps_per_frame = step_hz // self.camera_config['rate_hz']
            steps_per_imu = step_hz // int(self.imu_update_frequency)
            steps_per_gnss = step_hz // 10
            imu_stamp_ns = 0
            gnss_stamp_ns = 0
            step = 0
            while self._sim_running:
                wall_step_start = time.monotonic()
                step += 1
                stamp_ns = (step * 1_000_000_000 + step_hz // 2) // step_hz
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
                self.clock_pub.publish(clock)
                self.wait_for_sample('clock sync', stamp_ns, clock)
                if step % steps_per_imu == 0:
                    self.wait_for_sample('IMU', stamp_ns, clock)
                    imu_stamp_ns = stamp_ns
                if step % steps_per_gnss == 0:
                    self.wait_for_sample('GNSS', stamp_ns, clock)
                    gnss_stamp_ns = stamp_ns

                request = StepFirmware.Request()
                request.stamp = clock.clock
                request.imu_stamp.sec, request.imu_stamp.nanosec = divmod(imu_stamp_ns, 1_000_000_000)
                request.gnss_stamp.sec, request.gnss_stamp.nanosec = divmod(gnss_stamp_ns, 1_000_000_000)
                deadline = time.monotonic() + 10.0
                while self._sim_running:
                    completed = threading.Event()
                    result = self.sil_client.call_async(request)
                    result.add_done_callback(lambda future: completed.set())
                    while self._sim_running and not completed.wait(0.1):
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f'Firmware service did not respond at {stamp_ns} ns')
                        self.clock_pub.publish(clock)
                    if not self._sim_running:
                        return
                    response = result.result()
                    if response.success:
                        break
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f'Firmware step at {stamp_ns} ns: {response.message}')
                    self.clock_pub.publish(clock)
                    time.sleep(0.001)
                truth = self.wait_for_sample('truth', stamp_ns, clock)
                if truth is None:
                    break
                # Every consumer must ingest the completed step before the next clock tick.
                self.wait_for_sample('sensor state', stamp_ns, clock)
                self.wait_for_sample('forces state', stamp_ns, clock)

                if step % steps_per_frame == 0:
                    location, rotation, velocity, angular_velocity = self.extract_state(truth)
                    with self._interface_lock:
                        # HoloOcean renders the ROSflight pose; avoid advancing it again at flight speed.
                        self.interface.set_agent_state(location, rotation, np.zeros(3), np.zeros(3))
                        sensors_dict = self.interface.tick()
                    if self.ros_publish:
                        for sensor_name, sensor_data in sensors_dict.items():
                            self.publish_sensor(sensor_name, sensor_data, clock.clock)
                    ground = sensors_dict.get('GroundRange')
                    horizontal = sensors_dict.get('HorizontalRange')
                    if ground is not None and horizontal is not None:
                        if self.detect_collision(velocity, ground, horizontal):
                            self.get_logger().info('Collision detected and environment reset.')
                remaining = 1.0 / step_hz - (time.monotonic() - wall_step_start)
                if remaining > 0:
                    time.sleep(remaining)
        except Exception as e:
            self.get_logger().error(f'Lockstep simulation stopped: {e}')
            self._sim_running = False
            rclpy.shutdown()

    def sim_loop(self):
        """
        Thread loop to continuously advance the Holoocean simulation, publishing sensor data.
        """
        try:
            while self._sim_running:
                # Advance simulation and get sensor data.
                with self._interface_lock:
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


        except Exception as e:
            self.get_logger().error(f"Sim loop error: {e}")

    def destroy_node(self):
        """
        Override node destruction for clean shutdown of simulation thread.
        """
        self._sim_running = False
        with self._step_condition:
            self._step_condition.notify_all()
        if getattr(self, "tick_thread", None) and self.tick_thread.is_alive():
            self.tick_thread.join(timeout=2.0)
        super().destroy_node()

    def reset(self, request, response):
        """
        Service callback to reset the holoocean environment.
        """
        with self._interface_lock:
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
            with self._interface_lock:
                self.interface.reset_environment()
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
        if self.lockstep:
            with self._step_condition:
                self._latest_truth = msg
                self._step_condition.notify_all()
            return

        # Extract pose and velocities using the helper function
        location, rotation, velocity, angular_velocity = self.extract_state(msg)

        # Share latest velocity with sim thread (thread-safe)
        with self._state_lock:
            self._latest_velocity = velocity

        try:
            # Update agent state in the Holoocean simulation.
            with self._interface_lock:
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
    
    def publish_camera(self, sensor_data, stamp):
        config = self.camera_config
        pixels = np.asarray(sensor_data)
        if pixels.shape != (config['height'], config['width'], 4):
            raise ValueError(f'Unexpected DownCamera image shape: {pixels.shape}')

        rgb = pixels[:, :, :3]
        if config['encoding'] == 'mono8':
            image_data = np.rint(
                0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]
            ).astype(np.uint8)
            channels = 1
        else:
            image_data = np.ascontiguousarray(rgb)
            channels = 3

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'down_camera_optical'
        image.height = config['height']
        image.width = config['width']
        image.encoding = config['encoding']
        image.is_bigendian = False
        image.step = config['width'] * channels
        image.data = image_data.tobytes()

        focal = config['width'] / (2.0 * np.tan(np.deg2rad(config['fov_deg']) / 2.0))
        cx = (config['width'] - 1) / 2.0
        cy = (config['height'] - 1) / 2.0
        info = CameraInfo()
        info.header = image.header
        info.height = image.height
        info.width = image.width
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.camera_info_pub.publish(info)
        self.image_pub.publish(image)

    def publish_sensor(self, sensor_name, sensor_data=None, stamp=None):
        """
        Publish sensor data to the appropriate ROS2 topic.
        Parameters:
            sensor_name (str): Name of the sensor to publish data from
            sensor_data: Optional pre-fetched sensor data. If None, retrieves from interface."""
        # Retrieve sensor data if not provided.
        if sensor_data is None:
            sensor_data = self.interface.sensor_callback(sensor_name)
        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        # Initialize an empty message and create it based on sensor type.
        msg = None
        if sensor_name == 'RGBCamera':  
            msg = RGBCamera()
            msg.timestamp = self.stamp_ns(stamp)
            msg.width = 512
            msg.height = 512
            msg.channels = 4
            msg.image = sensor_data.ravel().tolist()
            self.RGBCamera_pub.publish(msg)

        elif sensor_name == 'DownCamera':
            self.publish_camera(sensor_data, stamp)

        elif sensor_name == 'GroundRange':
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
