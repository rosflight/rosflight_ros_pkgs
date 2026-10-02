# Fixedwing downward camera recording

The fixedwing HoloOcean launch uses `params/fixedwing_down_camera.yaml` for camera size,
horizontal FOV, rate, output encoding, and mount pose. `mono8` is the default; `rgb8`
is also supported. The mount coordinates use HoloOcean's forward-left-up body frame.
The exposure settings use histogram metering and +2 stops, two stops below
HoloOcean's documented +4 default, to retain detail in bright desert terrain.
Adjust `camera.exposure_compensation` in the YAML file if the image remains too
bright or becomes too dark; lower values make it darker.
The default camera is 0.5 m forward and 0.15 m below the center of mass, tilted
15 degrees forward from straight down. The HoloOcean viewport remains a
third-person view; the separate `image_view` window shows the
recorded camera. Set `show_camera_viewer:=false` to omit that window.

```sh
ros2 launch rosflight_sim fixedwing_holoocean.launch.py env:=desert
ros2 bag record -a --use-sim-time
```

Start the bag before takeoff. `--use-sim-time` gives the bag simulation-time
timestamps, so playback timing follows the simulated flight even if rendering
slows wall-clock time. Without it, the bag records wall-clock arrival times and
normal-speed playback reproduces the slow recording pace. For later playback,
use the synchronized simulated
sensor topics `/sim/sensors/imu/data` and `/sim/sensors/gnss` (the GNSS message
includes `vel_n`, `vel_e`, and `vel_d`). ROSflight also publishes `/imu/data` and
`/gnss` through MAVLink; their time conversion is separate from the simulated
sensor timestamps. The camera data is on `/fixedwing/camera/image_raw` and
`/fixedwing/camera/camera_info`. Record `/tf_static` and `/clock` as well.

`CameraInfo` contains the ideal pinhole intrinsics used for the configured
horizontal FOV, with zero distortion. `/tf_static` contains the transform from
`imu_frd` to `down_camera_optical`; the default translation is `[0.5, 0, 0.15]`
m in `imu_frd`, and its rotation includes the configured forward tilt. These can
be converted to OpenVINS's
`T_imu_cam`, intrinsics, resolution, and distortion fields when configuring
playback. The simulated IMU is at the aircraft center of mass.

The fixedwing launch uses simulation time. Firmware advances at 600 Hz, while the
simulated IMU publishes at 200 Hz. HoloOcean ticks, captures, and publishes the
camera at 30 Hz in simulation time.
Rendering may slow wall-clock flight without skipping camera timestamps. Choose
positive rates with `step_hz` divisible by `rate_hz`. HoloOcean is configured to
capture on every tick, so each published camera message is a new capture. ROS
timestamps are rounded to the nearest nanosecond, so successive
30 Hz camera intervals alternate between 33,333,333 and 33,333,334 ns. If you
change `step_hz` or the `imu_update_frequency` launch argument, keep `step_hz`
divisible by both the camera and IMU rates. The fixedwing launch also sets
`clock_sync_frequency` to the flight step rate so the firmware waits for each
clock update; set the `flight_step_hz` launch argument to the same value if you
change `step_hz`.
