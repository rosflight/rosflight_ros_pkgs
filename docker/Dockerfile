ARG ROS_DISTRO=humble

# Base image
FROM osrf/ros:${ROS_DISTRO}-desktop
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL [ "/bin/bash", "-c" ]

# Update system
RUN apt-get update \
    && apt-get upgrade -y

# Install utilities/packages
RUN apt-get install\
    ros-dev-tools\
    ros-${ROS_DISTRO}-plotjuggler-ros\
    zsh\
    vim\
    -y

# Install dependencies and build
WORKDIR /rosflight_ws
COPY src tmp
RUN rosdep install --from-paths . --ignore-src -y \
    && rm -rf tmp

# Create user
RUN useradd -ms /usr/bin/zsh docker_user \
    && usermod -aG sudo docker_user \
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

