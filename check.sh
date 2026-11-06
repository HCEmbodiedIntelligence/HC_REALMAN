#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
timeout 5 ros2 topic echo --once /io_teleop/joint_states
timeout 5 ros2 topic echo --once /io_teleop/hardware_ready
timeout 5 ros2 topic echo --once /status/realman/command_metrics
timeout 5 ros2 topic echo --once /observation/hands/joint_states || true
