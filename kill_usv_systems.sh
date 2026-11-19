#!/usr/bin/env bash
# 停止本工程的 ROS 节点和脚本。
exec python3 "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)/stop_usv_systems.py" "$@"
