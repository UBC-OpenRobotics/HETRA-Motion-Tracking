# syntax=docker/dockerfile:1
#
# ROS 2 dev image for the hand-tracking project (ROS 2 Humble, Ubuntu 22.04).
# Base is the official ros image for the chosen distro.
#
# To change distro, change ROS_DISTRO in docker-compose.yml (and check that the
# packages in the "project packages" block exist for that distro).

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    WS=/home/ros/ros2_ws

# ---------------------------------------------------------------------------
# ROS desktop tools (rviz2, rqt, demos like turtlesim) + general dev tools
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-turtlesim \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        bash-completion \
        git \
        less \
        nano \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Project packages (kept in a separate layer so edits here rebuild quickly)
#    If apt reports a package is missing for your distro, delete that line.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-imu-tools \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. Non-root user. Ubuntu 24.04+ images ship a default 'ubuntu' user at UID 1000,
#    so the check below removes it if present (22.04, used by Humble, has none).
# ---------------------------------------------------------------------------
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && (getent group ${USER_GID} || groupadd --gid ${USER_GID} ros) \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ros \
    && echo "ros ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

# Workspace folders. build/install/log are mounted as named volumes in compose;
# creating them here (owned by 'ros') makes the volumes start with the right owner.
RUN mkdir -p ${WS}/src ${WS}/build ${WS}/install ${WS}/log \
    && chown -R ${USER_UID}:${USER_GID} /home/ros

# Entrypoint sources ROS + the workspace so one-off commands
# (e.g. `docker compose run ros ros2 topic list`) also work.
# The sed strips Windows line endings in case git converted them on checkout.
COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh

USER ros
WORKDIR ${WS}

# Interactive shells (docker compose exec ros bash) get the same environment.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ros/.bashrc \
    && echo "[ -f ${WS}/install/setup.bash ] && source ${WS}/install/setup.bash" >> /home/ros/.bashrc \
    && echo "[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ] && source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ros/.bashrc

# Cache rosdep's index so `rosdep install` works later. Not critical if it fails.
RUN rosdep update --rosdistro ${ROS_DISTRO} || true

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]