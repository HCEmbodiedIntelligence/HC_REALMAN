#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u
if [ -d "$ROOT/vendor/Robotic_Arm" ]; then
  export PYTHONPATH="$ROOT/vendor${PYTHONPATH:+:$PYTHONPATH}"
fi
exec ros2 launch hc_realman_driver hc_realman.launch.py \
  hardware_config_path:="$ROOT/config/hardware.yaml" "$@"
