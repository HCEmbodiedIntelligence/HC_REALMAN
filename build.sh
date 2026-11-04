#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
set -u
cd "$ROOT"
if [ -d "$ROOT/vendor/Robotic_Arm" ]; then
  export PYTHONPATH="$ROOT/vendor${PYTHONPATH:+:$PYTHONPATH}"
fi
/usr/bin/python3 -c 'import Robotic_Arm.rm_robot_interface' 2>/dev/null || {
  echo "missing RealMan API2 runtime; run: /usr/bin/python3 -m pip install -r requirements.txt" >&2
  exit 2
}
colcon build --merge-install --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
