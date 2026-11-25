# syntax=docker/dockerfile:1
#
# ROS 2 Humble rover dev image
# - Builds natively for arm64 (Raspberry Pi 4/5 running a 64-bit OS) and amd64 (your PC).
#   Every downloaded binary is picked with `dpkg --print-architecture`, so nothing is hard-coded to amd64.
# - code-server and KasmVNC default to the newest GitHub release at build time.
#   Pin them for reproducible builds:  docker build --build-arg CODE_SERVER_VERSION=4.x.y ...
#
FROM ubuntu:jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG CODE_SERVER_VERSION=latest
ARG KASMVNC_VERSION=latest
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ---------------------------------------------------------------------------
# Locale first (ROS 2 needs a UTF-8 locale)
# ---------------------------------------------------------------------------
RUN apt-get update \
 && apt-get install -y --no-install-recommends locales ca-certificates curl \
 && sed -i 's/# en_US.UTF-8/en_US.UTF-8/' /etc/locale.gen \
 && locale-gen \
 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------------------
# Base tools, XFCE desktop, hardware/debug utilities
# ---------------------------------------------------------------------------
RUN apt-get update \
 && apt-get install -y \
    apt-utils build-essential manpages-dev cmake clang \
    xfce4 xfce4-goodies dbus-x11 x11-xserver-utils xterm \
    npm default-jre mono-complete \
    gnupg wget dumb-init git git-lfs htop lsb-release man-db \
    python3 python3-pip python3-serial nano vim-tiny zsh \
    openssh-client openssh-server procps sudo \
    iproute2 net-tools iputils-ping ssl-cert dialog \
    libswitch-perl libyaml-tiny-perl libhash-merge-simple-perl \
    liblist-moreutils-perl libdatetime-timezone-perl \
    software-properties-common \
    usbutils v4l-utils udev minicom screen \
 && git lfs install --system \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# code-server (latest release, native architecture)
# ---------------------------------------------------------------------------
RUN ARCH="$(dpkg --print-architecture)" \
 && VER="${CODE_SERVER_VERSION}" \
 && if [ "$VER" = "latest" ]; then \
      VER="$(curl -fsSL https://api.github.com/repos/coder/code-server/releases/latest \
             | grep -oP '"tag_name":\s*"v\K[^"]+')"; \
    fi \
 && echo "Installing code-server ${VER} (${ARCH})" \
 && curl -fL -o /tmp/code-server.deb \
      "https://github.com/coder/code-server/releases/download/v${VER}/code-server_${VER}_${ARCH}.deb" \
 && apt-get update \
 && apt-get install -y /tmp/code-server.deb \
 && rm -rf /tmp/code-server.deb /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# KasmVNC (latest release, native architecture)
# ---------------------------------------------------------------------------
RUN ARCH="$(dpkg --print-architecture)" \
 && VER="${KASMVNC_VERSION}" \
 && if [ "$VER" = "latest" ]; then \
      VER="$(curl -fsSL https://api.github.com/repos/kasmtech/KasmVNC/releases/latest \
             | grep -oP '"tag_name":\s*"v\K[^"]+')"; \
    fi \
 && echo "Installing KasmVNC ${VER} (${ARCH})" \
 && curl -fL -o /tmp/kasmvnc.deb \
      "https://github.com/kasmtech/KasmVNC/releases/download/v${VER}/kasmvncserver_jammy_${VER}_${ARCH}.deb" \
 && apt-get update \
 && apt-get install -y /tmp/kasmvnc.deb \
 && rm -rf /tmp/kasmvnc.deb /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# ROS 2 apt repository (current official method: ros2-apt-source package,
# which also handles future signing-key rotations)
# ---------------------------------------------------------------------------
RUN add-apt-repository -y universe \
 && VER="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
           | grep -oP '"tag_name":\s*"\K[^"]+')" \
 && CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")" \
 && curl -fL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${CODENAME}_all.deb" \
 && apt-get install -y /tmp/ros2-apt-source.deb \
 && rm -f /tmp/ros2-apt-source.deb

# ---------------------------------------------------------------------------
# ROS 2 Humble + everything for a differential-drive rover
# ---------------------------------------------------------------------------
RUN apt-get update \
 && apt-get upgrade -y \
 && apt-get install -y \
    ros-dev-tools \
    ros-humble-desktop \
    # --- description / visualisation (RViz2 comes with desktop) ---
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf2-tools \
    # --- navigation / mapping / localisation ---
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    # --- ros2_control ---
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    # --- simulation: Gazebo Fortress (the officially paired version for Humble) ---
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    # --- teleop ---
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    # --- sensors ---
    ros-humble-rplidar-ros \
    ros-humble-v4l2-camera \
    ros-humble-image-transport-plugins \
    # --- alternative DDS (often smoother with Nav2 on Humble) ---
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# ---------------------------------------------------------------------------
# micro-ROS agent (built from source – the documented way for Humble)
# Installed to /opt/microros_ws
# ---------------------------------------------------------------------------
RUN source /opt/ros/humble/setup.bash \
 && mkdir -p /opt/microros_ws/src \
 && cd /opt/microros_ws \
 && git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
 && apt-get update \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths src --ignore-src -y --rosdistro humble \
 && colcon build \
 && source install/local_setup.bash \
 && ros2 run micro_ros_setup create_agent_ws.sh \
 && ros2 run micro_ros_setup build_agent.sh \
 && rm -rf build log /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# SSH: listen on 2222 so it doesn't clash with the Pi's own sshd when the
# container uses the host network
# ---------------------------------------------------------------------------
RUN mkdir -p /run/sshd \
 && sed -i -E 's/^#?Port .*/Port 2222/' /etc/ssh/sshd_config

# ---------------------------------------------------------------------------
# User "robot" (uid 1000 matches the default user on Raspberry Pi OS / Ubuntu)
# dialout = serial ports, video = cameras, plugdev = raw USB devices
# ---------------------------------------------------------------------------
RUN useradd -m -s /bin/bash -u 1000 -G sudo,dialout,video,plugdev robot \
 && echo 'robot:robot' | chpasswd \
 && sed -i.bkp -e '/Defaults\tuse_pty/d' \
      -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers \
 && for g in ssl-cert kasmvnc-cert; do \
      if getent group "$g" >/dev/null; then usermod -aG "$g" robot; fi; \
    done

# Startup script
COPY start_services.sh /usr/local/bin/start_services.sh
RUN chmod +x /usr/local/bin/start_services.sh

USER robot
WORKDIR /home/robot

COPY --chown=robot:robot bashrc /home/robot/.bashrc
COPY --chown=robot:robot kasmvnc.yaml /home/robot/.vnc/kasmvnc.yaml

# KasmVNC: start XFCE automatically and skip the interactive "pick a desktop" prompt
RUN printf '%s\n' \
      '#!/bin/sh' \
      'unset SESSION_MANAGER' \
      'unset DBUS_SESSION_BUS_ADDRESS' \
      'exec dbus-launch --exit-with-session startxfce4' \
      > /home/robot/.vnc/xstartup \
 && chmod +x /home/robot/.vnc/xstartup \
 && touch /home/robot/.vnc/.de-was-selected \
 && mkdir -p /home/robot/program/.settings \
 && rosdep update --rosdistro humble

# code-server, KasmVNC, SSH
# (EXPOSE is informational; with --network host the ports are on the Pi directly)
EXPOSE 8080 8444 2222

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
CMD ["/usr/local/bin/start_services.sh"]
