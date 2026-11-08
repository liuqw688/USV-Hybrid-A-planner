#!/usr/bin/env bash
# One command enters the S-57 channel test, replacing the previous demo in this ROS domain.
set -eo pipefail
USV_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  -h|--help)
    cat <<'HELP'
用法：bash /home/l/1015v2/run_usv_channel.sh [ROS launch参数...]
默认重启测试并打开航道RViz，自动规划并驶向航道内约65米处的目标。
手动选点：追加 auto_goal:=false；RViz用2D Goal Pose拖动位置和船头方向。
停止：追加 --stop，或在启动终端按Ctrl+C。
仅检查环境：追加 --check。
默认模型：六状态11参数速度命令模型；不启动其他船只。
HELP
    exit 0 ;;
  --stop|--check)
    USV_ACTION="$1";shift
    exec bash "$USV_WORKSPACE/run_usv_tracking.sh" "$USV_ACTION" environment:=channel "$@" ;;
  --restart)
    shift ;;
esac
exec bash "$USV_WORKSPACE/run_usv_tracking.sh" --restart environment:=channel "$@"
