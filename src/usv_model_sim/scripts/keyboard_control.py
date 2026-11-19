#!/usr/bin/env python3
"""读取终端按键并向无人船动力学模型发布速度命令."""

import math
import os
from pathlib import Path
import select
import signal
import sys
import termios
import tty

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
import yaml


HELP = """
无人船键盘控制（点击此终端后，使用小写按键）
  w / s  前进 / 后退      a / d  原地左转 / 原地右转
  q / e  前进左转 / 前进右转
  1 / 2  增加 / 减小速度档位
  3 / 4  增加 / 减小角速度档位
  空格   指令归零          t      指令归零并退出
选定方向后持续发送指令，直到切换方向、按空格或退出。
"""

MOTIONS = {
    "w": (1, 0), "s": (-1, 0),
    "a": (0, 1), "d": (0, -1),
    "q": (1, 1), "e": (1, -1),
    " ": (0, 0),
}


class KeyboardControl(Node):
    """管理键盘方向、速度档位及周期命令发布."""

    def __init__(self):
        """读取配置并创建命令发布定时器；无效参数抛出 ValueError."""
        super().__init__("usv_keyboard_control")
        config_path = Path(get_package_share_directory("usv_model_sim")) / "config/usv_model.yaml"
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        model = config["usv_model_node"]["ros__parameters"]
        keyboard = config["usv_keyboard_control"]["ros__parameters"]
        # 从模型配置读取话题和限幅，确保键盘命令与仿真约束一致。
        for name in ("cmd_topic", "max_speed_mps", "max_yaw_rate_deg_s"):
            self.declare_parameter(name, model[name])
        for name, value in keyboard.items():
            self.declare_parameter(name, value)

        self.max_speed = self.number("max_speed_mps")
        self.max_yaw_rate = self.number("max_yaw_rate_deg_s")
        self.speed = min(self.number("speed_mps"), self.max_speed)
        self.yaw_rate = min(self.number("yaw_rate_deg_s"), self.max_yaw_rate)
        self.speed_step = self.number("speed_step_mps")
        self.yaw_rate_step = self.number("yaw_rate_step_deg_s")
        if self.speed_step == 0.0 or self.yaw_rate_step == 0.0:
            raise ValueError("速度和角速度调整步长必须大于零")

        self.motion = (0, 0)
        self.publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10
        )
        # 以 10 Hz 刷新命令，避免触发模型默认的 0.5 s 命令超时。
        self.timer = self.create_timer(0.1, self.publish_command)

    def number(self, name):
        """
        读取并校验非负有限数值参数.

        :param name: 参数名称。

        :return: 浮点参数值，单位由参数名称及配置约定。

        :raises ValueError: 参数不可转换为浮点数，或为负数、非有限值。
        """
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} 必须是非负有限数值")
        return value

    def handle_key(self, key):
        """
        处理方向或档位按键，发布更新后的指令；其他按键不改变状态.

        :param key: 单字符按键，与 MOTIONS 或档位按键表匹配。
        """
        if key in MOTIONS:
            self.motion = MOTIONS[key]
        elif key in ("1", "2"):
            change = self.speed_step if key == "1" else -self.speed_step
            self.speed = min(self.max_speed, max(0.0, self.speed + change))
        elif key in ("3", "4"):
            change = self.yaw_rate_step if key == "3" else -self.yaw_rate_step
            self.yaw_rate = min(self.max_yaw_rate, max(0.0, self.yaw_rate + change))
        else:
            return
        self.publish_command()
        self.show_status()

    def publish_command(self):
        """发布当前档位指令，linear.x 单位 m/s，angular.z 单位 rad/s."""
        message = Twist()
        message.linear.x = self.motion[0] * self.speed
        message.angular.z = self.motion[1] * math.radians(self.yaw_rate)
        self.publisher.publish(message)

    def show_status(self):
        """在终端当前行显示有符号命令及速度档位."""
        speed = self.motion[0] * self.speed
        yaw_rate = self.motion[1] * self.yaw_rate
        print(
            f"\r档位 {self.speed:.2f} m/s、{self.yaw_rate:.1f} °/s | "
            f"指令 {speed:+.2f} m/s、{yaw_rate:+.1f} °/s        ",
            end="", flush=True,
        )

    def stop(self):
        """将运动方向归零并立即发布零指令."""
        self.motion = (0, 0)
        self.publish_command()


def interrupt(_signum, _frame):
    """将终止信号转换为 KeyboardInterrupt，由主循环执行退出清理."""
    raise KeyboardInterrupt


def main(args=None):
    """
    运行交互式键盘控制，退出时归零指令并恢复终端设置.

    :param args: 传递给 rclpy 的可选命令行参数。

    :return: 正常退出返回 0，非交互终端或运行失败返回 1。
    """
    if not sys.stdin.isatty():
        print("请在交互终端中运行：ros2 run usv_model_sim keyboard_control.py", file=sys.stderr)
        return 1

    descriptor = sys.stdin.fileno()
    terminal_settings = termios.tcgetattr(descriptor)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, interrupt)
    node = None
    result = 0
    try:
        node = KeyboardControl()
        print(HELP)
        print(f"上限：{node.max_speed:.2f} m/s、{node.max_yaw_rate:.1f} °/s")
        node.show_status()
        tty.setcbreak(descriptor)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = os.read(descriptor, 1).decode("latin1")
                if key in ("t", "\x04", ""):
                    break
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"\n键盘控制启动或运行失败：{error}", file=sys.stderr)
        result = 1
    finally:
        try:
            if node is not None and rclpy.ok():
                node.stop()
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            termios.tcsetattr(descriptor, termios.TCSADRAIN, terminal_settings)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print("\n控制指令已归零，程序退出。")
    return result


if __name__ == "__main__":
    sys.exit(main())
