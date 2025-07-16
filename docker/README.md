# Docker build instructions

This Docker container can be used instead of a native ROS2 installation.

## Setup

Instead of cloning all the relevant repositories into the Docker container, it will mount the `rosflight_ws` repository to the docker container at runtime.
Thus, it is important to make sure you have set up your `rosflight_ws` directory as follows:
```bash
rosflight_ws
├── build
├── install
├── log
└── src
    ├── roscopter
    ├── rosflight_ros_pkgs
    ├── rosplane
    └── <other_ros2_packages>
```

If you don't have the `build`, `install`, and `log` directories, don't worry about creating them.
They will be created when we `colcon build` inside of the container.

Make sure you have installed the Docker engine, as well as the Docker Compose utility.

## Building

1. Navigate to `/path/to/your/rosflight_ws` for the next commands. You could run these commands from the `src/rosflight_ros_pkgs/docker` directory if you want (changing the file paths as necessary).
2. Build with:
    ```bash
    docker compose -f src/rosflight_ros_pkgs/docker/compose.yaml build
    ```

## Running

Start the docker container in the background with:
```bash
docker compose -f src/rosflight_ros_pkgs/docker/compose.yaml up -d
```

Start a session attached to the container:
```bash
docker compose -f src/rosflight_ros_pkgs/docker/compose.yaml exec rosflight zsh
```

Note that you can use the command to attach to the container any number of times.
