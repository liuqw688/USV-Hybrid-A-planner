#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to ins_path_replay --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source "$workspace_dir/install/setup.bash"
replay_output="$workspace_dir/artifacts/ins_replay/manual_$(date +%Y%m%d_%H%M%S)"
exec ros2 launch ins_path_replay ins_replay.launch.py \
  "csv_path:=$workspace_dir/data_ins_20261019_153413.csv" \
  "output_dir:=$replay_output" "$@"
