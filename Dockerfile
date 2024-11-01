# Base image
ARG ROS_DISTRO=jazzy
FROM docker.io/ros:${ROS_DISTRO}

# Update system
RUN apt update && apt upgrade -y && rosdep update

# Install dependencies
WORKDIR /rosflight_ws
COPY . src/rosflight_ros_pkg
RUN rosdep install --from-paths src --ignore-src -r -y
RUN rm -rf /var/lib/apt/lists/* && rm -rf /root/.ros/rosdep

# Build workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Add workspace to source file to entrypoint
RUN sed -i '/exec "\$@"/i source "\/rosflight_ws\/install\/setup.bash" --' /ros_entrypoint.sh

