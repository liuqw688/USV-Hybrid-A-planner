#!/usr/bin/env bash
set -e
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
export LD_LIBRARY_PATH="$workspace_dir/.test_deps/root/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"
exec ros2 launch free_water_map fast_planner_open_water.launch.py "$@"
