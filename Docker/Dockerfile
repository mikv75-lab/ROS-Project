# Thesis Docker – MoveIt2 + ROS2 Rolling + ros-gz + ZSH + leeres globales venv (/root/.venv) + Volume /root
FROM moveit/moveit2:rolling-source

ENV DEBIAN_FRONTEND=noninteractive

# ---- Grundsystem + Tools ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    zsh git curl wget vim nano less locales gnupg2 lsb-release ca-certificates \
    build-essential cmake pkg-config \
    python3-venv python3-pip \
    net-tools iputils-ping \
    tree htop ccache trimesh numpy \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo (ros-gz Integration – offizieller ROS Weg) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# ---- ccache Performance ----
ENV CCACHE_DIR=/root/.ccache
ENV PATH="/usr/lib/ccache:${PATH}"

# ---- Persistentes Volume (/root speichert App + Workspace + venv) ----
VOLUME ["/root"]

# ---- ENTRYPOINT fix: ROS immer mit *bash* sourcen, dann exec "$@" ----
RUN mv /ros_entrypoint.sh /ros_entrypoint_original.sh
RUN printf '%s\n' \
  '#!/bin/bash' \
  'set -e' \
  'if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then' \
  '  source "/opt/ros/$ROS_DISTRO/setup.bash"' \
  'fi' \
  'if [ -f "/root/ws_moveit/install/setup.bash" ]; then' \
  '  source "/root/ws_moveit/install/setup.bash"' \
  'fi' \
  'exec "$@"' \
  > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

# ---- Globales venv anlegen (leer). Aktivierung in ZSH. ----
RUN python3 -m venv /root/.venv
RUN echo 'source /root/.venv/bin/activate' >> /root/.zshrc

# ---- ZSH-Comfort + Pfade ----
RUN echo 'export PYTHONPATH=/root/app:$PYTHONPATH' >> /root/.zshrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=/root/ws_moveit:/root/ws_moveit/src' >> /root/.zshrc && \
    echo 'export ROS_DOMAIN_ID=0' >> /root/.zshrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.zshrc && \
    echo "alias ll='ls -alF'" >> /root/.zshrc && \
    echo "alias ws='cd /root/ws_moveit'" >> /root/.zshrc && \
    echo "alias app='cd /root/app'" >> /root/.zshrc

# ---- Initiale Projektstruktur im Volume ----
RUN printf '%s\n' \
  '#!/usr/bin/env bash' \
  'mkdir -p /root/app' \
  'mkdir -p /root/ws_moveit/src /root/ws_moveit/build /root/ws_moveit/install /root/ws_moveit/log' \
  'echo "[volume_init] /root/app + /root/ws_moveit bereit"' \
  > /usr/local/bin/volume_init.sh && chmod +x /usr/local/bin/volume_init.sh

# ---- Standard Shell ----
CMD ["zsh"]
