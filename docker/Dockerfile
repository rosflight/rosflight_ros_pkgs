ARG ROS_DISTRO=humble
ARG ROS_IMAGE=osrf/ros:${ROS_DISTRO}-desktop

# Base image
FROM ${ROS_IMAGE}
SHELL [ "/bin/bash", "-c" ]
# set default user ID
ARG HOST_UID=1000

# Update system and install utilities/packages
# $ROS_DISTRO is inherited from the base ROS_IMAGE
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install \
    ros-dev-tools \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    zsh \
    vim \
    -y

# Install dependencies and build
WORKDIR /rosflight_ws
COPY src tmp
RUN rosdep install --from-paths . --ignore-src -y \
    && rm -rf tmp

# Create user
RUN useradd -u ${HOST_UID} -ms /usr/bin/zsh docker_user \
    && usermod -aG sudo,dialout,plugdev docker_user \
    && echo "docker_user:pw" | chpasswd
USER docker_user

# Setup ROS environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.zsh" >> /home/docker_user/.zshrc \
    && echo "source /rosflight_ws/install/setup.zsh" >> /home/docker_user/.zshrc

# Add autocompletion for ROS commands with zsh
RUN echo "eval $(register-python-argcomplete3 ros2)" >> /home/docker_user/.zshrc \
    && echo "eval $(register-python-argcomplete3 colcon)" >> /home/docker_user/.zshrc

# Temporary fix for running ROS in Docker
RUN echo "ulimit -n 1024" >> /home/docker_user/.zshrc

ENTRYPOINT [ "/usr/bin/zsh" ]

