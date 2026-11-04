#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
timeout 5 ros2 topic echo --once /io_teleop/joint_states
timeout 5 ros2 topic echo --once /io_teleop/hardware_ready
timeout 5 ros2 topic echo --once /observation/hands/joint_states || true
