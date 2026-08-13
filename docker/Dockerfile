ARG ROS_DISTRO=lyrical
ARG ROS_IMAGE=osrf/ros:${ROS_DISTRO}-desktop

# Base image
FROM ${ROS_IMAGE}
SHELL [ "/bin/bash", "-c" ]

# Set default args
# Use the host UID for the container user to avoid mounted file permission issues
ARG HOST_UID=1000
ARG USER_SHELL=/usr/bin/bash
ENV SHELL=${USER_SHELL}

# Update system and install utilities/packages
# $ROS_DISTRO is inherited from the base ROS_IMAGE
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
    ros-dev-tools \
    zsh \
    vim \
    && if apt-cache show "ros-${ROS_DISTRO}-plotjuggler-ros" >/dev/null 2>&1; then \
        apt-get install -y "ros-${ROS_DISTRO}-plotjuggler-ros"; \
    else \
        apt-get install -y "ros-${ROS_DISTRO}-plotjuggler"; \
    fi

# Install dependencies and build
WORKDIR /rosflight_ws
COPY src tmp
RUN rosdep install --from-paths . --ignore-src -y \
    && rm -rf tmp

# Add user to needed groups with password "pw"
RUN if ! getent passwd ${HOST_UID}; then \
        useradd -u ${HOST_UID} -ms "${USER_SHELL}" ubuntu; \
    fi \
    && usermod -aG sudo,dialout,plugdev ubuntu \
    && echo "ubuntu:pw" | chpasswd
USER ubuntu

# Source ROS environment variables with autocompletion
RUN echo 'source ~/.rosrc' >> /home/ubuntu/.bashrc \
    && cp /etc/zsh/newuser.zshrc.recommended ~/.zshrc \
    && if [[ "${ROS_DISTRO}" == "humble" ]]; then \
         echo 'autoload -U +X bashcompinit && bashcompinit' >> /home/ubuntu/.zshrc; \
       fi \
    && echo 'source ~/.rosrc' >> /home/ubuntu/.zshrc \
    && cat > /home/ubuntu/.rosrc <<EOF
if [ -n "\$BASH" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  if [ -f /rosflight_ws/install/local_setup.bash ]; then
    source /rosflight_ws/install/local_setup.bash
  fi
elif [ -n "\$ZSH_NAME" ]; then
  source /opt/ros/${ROS_DISTRO}/setup.zsh
  if [ -f /rosflight_ws/install/local_setup.zsh ]; then
    source /rosflight_ws/install/local_setup.zsh
  fi
  if command -v register-python-argcomplete >/dev/null 2>&1; then
    eval "\$(register-python-argcomplete ros2)"
    eval "\$(register-python-argcomplete colcon)"
  elif command -v register-python-argcomplete3 >/dev/null 2>&1; then
    eval "\$(register-python-argcomplete3 ros2)"
    eval "\$(register-python-argcomplete3 colcon)"
  fi
else
  source /opt/ros/${ROS_DISTRO}/setup.sh
  if [ -f /rosflight_ws/install/local_setup.sh ]; then
    source /rosflight_ws/install/local_setup.sh
  fi
fi
EOF

# Create dynamic entry point (can set SHELL as an arg when entering container)
ENTRYPOINT exec "$SHELL"
