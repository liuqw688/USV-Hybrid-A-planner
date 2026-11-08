#!/usr/bin/env bash
# Open-water simulation: migrated frontend + existing MPC + vessel + RViz.
set -eo pipefail

USV_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
USV_CHECK_ONLY=false
USV_ACTION=start
case "${1:-}" in
  -h|--help)
    cat <<'HELP'
用法：bash /home/l/1015v2/run_usv_tracking.sh [ROS launch参数...]

默认：打开RViz，自动驶向(40,10)，目标航向0.25弧度。
模型：前端、MPC和仿真统一采用11参数速度命令动力学。
输出状态间隔：默认0.1秒，可追加 output_step:=0.1 设置。
显示：橙色箭头为各轨迹节点船体航向；黄色线为实测航迹。
手动选点：追加 auto_goal:=false，在RViz使用2D Goal Pose。
自定目标：追加 goal_x:=60.0 goal_y:=15.0 goal_yaw:=0.4
环境检查：追加 --check（不启动仿真）。
航道地图：使用 bash /home/l/1015v2/run_usv_channel.sh（自动重启并给出航道内默认目标）。
停止测试：追加 --stop（也适用于原启动终端已关闭时）。
重新开始：追加 --restart auto_goal:=false（重置仿真，等待手动选点）。
退出：在启动终端按Ctrl+C；再次启动会从初始位置重新仿真。
默认ROS域82，仅本机通信；已有实例运行时不会重复启动。
HELP
    exit 0
    ;;
  --check)
    USV_CHECK_ONLY=true
    shift
    ;;
  --stop|--restart)
    USV_ACTION="${1#--}"
    shift
    ;;
esac

if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$USV_WORKSPACE/install/setup.bash" ]]; then
  echo "缺少ROS Humble或工作空间构建环境，请先完成1015v2构建。" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$USV_WORKSPACE/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-82}"
export ROS_LOCALHOST_ONLY=1
if [[ ! "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || (( 10#$ROS_DOMAIN_ID > 232 )); then
  echo "ROS_DOMAIN_ID必须为0至232的整数。" >&2
  exit 1
fi
if ! command -v flock >/dev/null; then
  echo "缺少flock，无法检查重复启动。" >&2
  exit 1
fi
USV_MAP_PACKAGES=(free_water_map)
for USV_ARGUMENT in "$@"; do
  if [[ "$USV_ARGUMENT" == environment:=channel ]]; then
    USV_MAP_PACKAGES=(river_chart local_costmap_generator)
  fi
done
for USV_PACKAGE in usv_kinodynamic_frontend usv_mpc_control "${USV_MAP_PACKAGES[@]}" rviz2; do
  if ! ros2 pkg prefix "$USV_PACKAGE" >/dev/null 2>&1; then
    echo "未找到功能包：$USV_PACKAGE，请先完成1015v2构建。" >&2
    exit 1
  fi
done

echo "工作空间：$USV_WORKSPACE；ROS域：$ROS_DOMAIN_ID（仅本机）"
if [[ "$USV_ACTION" == stop || "$USV_ACTION" == restart ]]; then
  python3 - <<'PY'
import fcntl
import os
from pathlib import Path
import signal
import time

domain = os.environ['ROS_DOMAIN_ID']
lock_path = Path('/tmp/usv_frontend_tracking_domain_' + domain + '.lock')
with lock_path.open('a+') as lock:
    def released():
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        fcntl.flock(lock, fcntl.LOCK_UN)
        return True

    if released():
        print('当前ROS域没有运行中的闭环测试。')
        raise SystemExit(0)
    lock.seek(0)
    try:
        pid = int(lock.read().strip())
        proc = Path('/proc') / str(pid)
        argv = proc.joinpath('cmdline').read_bytes().split(b'\0')
        env = proc.joinpath('environ').read_bytes().split(b'\0')
        expected = [b'launch', b'usv_kinodynamic_frontend', b'open_water_tracking.launch.py']
        valid = any(argv[i:i + 3] == expected for i in range(len(argv) - 2))
        valid = valid and ('ROS_DOMAIN_ID=' + domain).encode() in env
        if pid <= 1 or not valid:
            raise ValueError('owner mismatch')
    except (OSError, ValueError):
        raise SystemExit('无法确认旧实例身份，请在原启动终端按Ctrl+C关闭。')
    print('正在结束当前闭环测试……', flush=True)
    os.kill(pid, signal.SIGINT)
    for _ in range(150):
        if released():
            print('旧测试已结束。')
            break
        time.sleep(0.1)
    else:
        raise SystemExit('旧实例尚未退出，请检查其启动终端；未强制结束进程。')
PY
  if [[ "$USV_ACTION" == stop ]]; then
    exit 0
  fi
fi
# This probe releases its lock immediately. The launch itself holds the lock
# for its lifetime and remains authoritative if two starts race each other.
if ! flock -n "/tmp/usv_frontend_tracking_domain_${ROS_DOMAIN_ID}.lock" true; then
  echo "该ROS域的测试已在运行，请直接在现有RViz中使用2D Goal Pose设置目标。"
  echo "重新开始：bash $USV_WORKSPACE/run_usv_tracking.sh --restart auto_goal:=false"
  exit 0
fi
if "$USV_CHECK_ONLY"; then
  echo "环境检查通过，可以启动空旷水域闭环测试。"
  exit 0
fi

cd "$USV_WORKSPACE"
echo "正在启动所选地图、无人船仿真、规划、MPC和RViz；按Ctrl+C结束。"
exec ros2 launch usv_kinodynamic_frontend open_water_tracking.launch.py "$@"
