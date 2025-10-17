#!/bin/bash
set -e
source /opt/ros/rolling/setup.bash
source /root/ws_moveit/install/setup.bash
export PYTHONUNBUFFERED=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
python3 -m source.app.main_gui
