#!/usr/bin/env python3
"""生成仅前进的路线，长度单位为米，支持配置 XY 起点偏移."""

from functools import lru_cache
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from publish_bspline_reference import BsplineReferencePublisher  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from scipy.interpolate import BSpline, make_interp_spline  # noqa: E402

from usv_interfaces.msg import UsvBspline, UsvBsplineControlPoint  # noqa: E402

ROUTES = ("straight", "figure_eight", "tight_turns", "circle", "ellipse", "square", "mowing")
ANGLES = (-30, 60, -90, 150, 180)
ROUTE_SPEED_MPS = {
    "straight": 4.0,
    "figure_eight": 4.0,
    "tight_turns": 4.0 / 2.5,
    "circle": 4.0,
    "ellipse": 4.0,
    "square": 4.0 / 2.5,
    "mowing": 4.0 / 1.5,
}
SIM_DT_S = 0.02
SPLINE_SAMPLE_PERIOD_S = 0.5
MAX_YAW_RATE_RADPS = math.radians(15.0)
MAX_YAW_ACCEL_RADPS2 = 0.08
ROUTE_MAX_DECEL_MPS2 = 0.45
# 仅前进模型在纵向速度降为零后，可能停在终点前数米处。
ROUTE_ENDPOINT_TOLERANCE_M = 4.5
GLOBAL_ROUTE_ENDPOINT_TOLERANCE_M = 0.75
MAX_SPEED_COMMAND_MPS = 5.0
MAX_SPEED_COMMAND_ACCEL_MPS2 = 0.65
MAX_SPEED_COMMAND_DECEL_MPS2 = 2.5
MAX_YAW_COMMAND_ACCEL_RADPS2 = 0.12
STOP_DWELL_S = 2.0
MAX_SIM_TIME_S = 1800.0
COMPOSITE_ROUTE = "composite"
ROLLING_SEGMENT_LENGTH_M = 50.0
ROLLING_PUBLISH_PERIOD_S = 0.1
# 10 Hz 下船舶每次更新最多行驶 0.5 m；2 m 容差可覆盖少量丢帧。
PROJECTION_FORWARD_WINDOW_M = 2.0
MAX_GLOBAL_PROJECTION_ERROR_M = 2.0
ROLLING_MODEL_DT_S = 0.1
GLOBAL_MODEL_DT_S = 0.02
GLOBAL_SPLINE_SAMPLE_PERIOD_S = 0.5
GLOBAL_TERMINAL_DWELL_S = 10.0
REFERENCE_YAW_RATE_LIMIT_RADPS = MAX_YAW_RATE_RADPS * 0.4
PUBLISH_TIMER_TICK_S = 0.01
PUBLISH_PHASE_RAD = 0.731
LABELS = dict(
    zip(
        ROUTES,
        (
            "Straight 300m",
            "Figure eight",
            "Tight turns R16m",
            "Circle R70m",
            "Ellipse 100x60m",
            "Square 140m / R20m",
            "Mowing 4x140m",
        ),
    )
)
LABELS[COMPOSITE_ROUTE] = "R7 + 200m + R10 + 200m + R15 + 200m"
ROUTE_SPEED_MPS[COMPOSITE_ROUTE] = 4.0

# 此处系数须与 usv_model_sim/include/usv_model_sim/dynamics.hpp 保持一致。
MODEL = {
    "au": -0.431930955,
    "bu": 0.437133189,
    "av1": -0.313324916,
    "av2": 0.006741061,
    "bv": 0.407958500,
    "bv0": 0.000993619,
    "ar1": -0.166118168,
    "ar2": -0.544509453,
    "br": 0.837536662,
    "br0": 0.000626529,
}


def steady_sway_velocity(yaw_rate):
    """使用辨识得到的三自由度模型求解 v_dot=r_dot=0 的稳态状态."""
    p = MODEL
    numerator = (p["bv"] * p["ar2"] - p["br"] * p["av2"]) * yaw_rate
    numerator += p["bv"] * p["br0"] - p["br"] * p["bv0"]
    denominator = p["av1"] * p["br"] - p["bv"] * p["ar1"]
    return numerator / denominator


def dynamic_reference_metrics(controls, knots, degree=3):
    """检查发布的时间样条是否符合仿真船舶模型."""
    spline = BSpline(knots, controls, degree)
    start, end = knots[degree], knots[len(controls)]
    times = np.linspace(start + 1e-4, end - 1e-4, max(1000, int((end - start) * 10)))
    pose = spline(times)
    velocity = spline.derivative(1)(times)
    acceleration = spline.derivative(2)(times)
    phi = pose[:, 2]
    r = velocity[:, 2]
    u = -np.sin(phi) * velocity[:, 0] + np.cos(phi) * velocity[:, 1]
    v = np.cos(phi) * velocity[:, 0] + np.sin(phi) * velocity[:, 1]
    u_dot = -r * v - np.sin(phi) * acceleration[:, 0] + np.cos(phi) * acceleration[:, 1]
    v_dot = r * u + np.cos(phi) * acceleration[:, 0] + np.sin(phi) * acceleration[:, 1]
    r_dot = acceleration[:, 2]
    p = MODEL
    v_cmd = (u_dot - p["au"] * u) / p["bu"]
    w_from_v = (v_dot - p["av1"] * v - p["av2"] * r - p["bv0"]) / p["bv"]
    w_from_r = (r_dot - p["ar1"] * v - p["ar2"] * r - p["br0"]) / p["br"]
    w_cmd = 0.5 * (w_from_v + w_from_r)
    command_slew = np.diff(v_cmd) / np.diff(times)
    yaw_command_slew = np.diff(w_cmd) / np.diff(times)
    residual_v = v_dot - (p["av1"] * v + p["av2"] * r + p["bv"] * w_cmd + p["bv0"])
    residual_r = r_dot - (p["ar1"] * v + p["ar2"] * r + p["br"] * w_cmd + p["br0"])
    residual_u = u_dot - (p["au"] * u + p["bu"] * v_cmd)
    wrap_delta = np.angle(np.exp(1j * np.diff(phi)))
    return {
        "duration_s": float(end - start),
        "sampled_length_m": float(np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1).sum()),
        "max_abs_yaw_rate_radps": float(np.max(np.abs(r))),
        "max_abs_forward_command_mps": float(np.max(np.abs(v_cmd))),
        "max_abs_yaw_command_radps": float(np.max(np.abs(w_cmd))),
        "max_speed_command_accel_mps2": float(max(0.0, np.max(command_slew))),
        "max_speed_command_decel_mps2": float(max(0.0, -np.min(command_slew))),
        "max_yaw_command_accel_radps2": float(np.max(np.abs(yaw_command_slew))),
        "max_abs_sway_mps": float(np.max(np.abs(v))),
        "rms_sway_mps": float(np.sqrt(np.mean(v * v))),
        "dynamics_residual_p95": float(
            np.percentile(
                np.maximum.reduce((np.abs(residual_u), np.abs(residual_v), np.abs(residual_r))), 95
            )
        ),
        "dynamics_residual_max": float(
            np.max(np.maximum.reduce((np.abs(residual_u), np.abs(residual_v), np.abs(residual_r))))
        ),
        "yaw_rate_command_limit_fraction": float(np.mean(np.abs(w_cmd) > MAX_YAW_RATE_RADPS)),
        "max_spline_heading_step_rad": float(np.max(np.abs(wrap_delta)))
        if len(wrap_delta)
        else 0.0,
    }


def geometry(name):
    """
    生成指定类型的测试路线几何，移除连续重复点.

    :param name: 路线类型名称。

    :return: N×2 平面坐标数组，单位 m。

    :raises ValueError: 路线类型不受支持。
    """
    points = [np.zeros(2)]

    def line(end):
        start = points[-1]
        for p in np.linspace(start, end, max(2, int(np.linalg.norm(end - start) / 0.3) + 1))[1:]:
            points.append(p)

    def curve(values):
        points.extend(np.asarray(values))

    if name == COMPOSITE_ROUTE:
        def circle(radius):
            start = points[-1].copy()
            count = max(100, int(math.ceil(2.0 * math.pi * radius / 0.1)))
            angle = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, count + 1)[1:]
            curve(np.c_[start[0] + radius * np.cos(angle),
                        start[1] + radius + radius * np.sin(angle)])

        def straight(length):
            line(points[-1] + np.array([length, 0.0]))

        circle(7.0)
        straight(200.0)
        circle(10.0)
        straight(200.0)
        circle(15.0)
        straight(200.0)
    elif name == "straight":
        line(np.array([300.0, 0.0]))
    elif name in ("tight_turns", "mowing"):
        radius = 16.0 if name == "tight_turns" else 35.0
        length = 140.0
        for lane in range(4):
            direction = 1 if lane % 2 == 0 else -1
            x = length if direction > 0 else 0.0
            y = 2 * radius * lane
            line(np.array([x, y]))
            if lane < 3:
                theta = np.linspace(0, np.pi, 700)[1:]
                curve(
                    np.c_[x + direction * radius * np.sin(theta), y + radius * (1 - np.cos(theta))]
                )
    else:
        # 设置入口和出口切线，使测试覆盖完整闭合曲线，
        # 同时避免把起始接近段与末端制动段混在一起。
        line(np.array([60.0, 0.0]))
        if name == "figure_eight":
            t = np.linspace(-np.pi / 2, 3 * np.pi / 2, 5001)[1:]
            curve(np.c_[60 - 55 * np.sin(2 * t), 100 * (1 + np.sin(t))])
        elif name in ("circle", "ellipse"):
            a, b = (70.0, 70.0) if name == "circle" else (100.0, 60.0)
            t = np.linspace(0, 2 * np.pi, 5001)[1:]
            curve(np.c_[60 + a * np.sin(t), b * (1 - np.cos(t))])
        elif name == "square":
            for target, centre, start in (
                ([160.0, 0.0], [160.0, 20.0], -np.pi / 2),
                ([180.0, 120.0], [160.0, 120.0], 0.0),
                ([60.0, 140.0], [60.0, 120.0], np.pi / 2),
                ([40.0, 20.0], [60.0, 20.0], np.pi),
            ):
                line(np.array(target))
                t = np.linspace(start, start + np.pi / 2, 401)[1:]
                curve(np.array(centre) + 20 * np.c_[np.cos(t), np.sin(t)])
        else:
            raise ValueError(name)
        line(np.array([120.0, 0.0]))
    raw = np.array(points)
    keep = np.r_[True, np.linalg.norm(np.diff(raw, axis=0), axis=1) > 1e-8]
    return raw[keep]


def _wrap(angle):
    """将角度归一化至 [-pi, pi]，输入和输出单位均为 rad."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _odom_navigation_yaw(q):
    """从 /odom 的 RFU 四元数读取模型使用的北零航向角."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _model_derivative(state, command):
    """
    按辨识模型计算六状态导数，用于生成动力学测试真值.

    :param state: [x, y, phi, u, v, r]，单位依次为 m、m、rad、m/s、m/s、rad/s。
    :param command: [V_cmd, W_cmd]，单位依次为 m/s、rad/s。

    :return: 六状态的时间导数；x 向东、y 向北，phi 北零向西为正。
    """
    x, y, phi, u, v, r = state
    speed_cmd, yaw_cmd = command
    p = MODEL
    return np.array(
        [
            -u * math.sin(phi) + v * math.cos(phi),
            u * math.cos(phi) + v * math.sin(phi),
            r,
            p["au"] * u + p["bu"] * speed_cmd,
            p["av1"] * v + p["av2"] * r + p["bv"] * yaw_cmd + p["bv0"],
            p["ar1"] * v + p["ar2"] * r + p["br"] * yaw_cmd + p["br0"],
        ]
    )


def _rk4(state, command, dt):
    """
    用定值命令进行一步四阶 Runge-Kutta 积分.

    :param state: 六状态数组，顺序与 _model_derivative 一致。
    :param command: [V_cmd, W_cmd]。
    :param dt: 积分步长，单位 s。

    :return: 积分后的六状态数组。
    """
    k1 = _model_derivative(state, command)
    k2 = _model_derivative(state + 0.5 * dt * k1, command)
    k3 = _model_derivative(state + 0.5 * dt * k2, command)
    k4 = _model_derivative(state + dt * k3, command)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0


def _speed_command_stop_distance(speed, speed_command):
    """计算推力降至零并减速到路线目标速度所需的距离."""
    drag = -MODEL["au"]
    if drag <= 0.0:
        return math.inf
    speed = max(0.0, float(speed))
    speed_command = max(0.0, float(speed_command))
    ramp_duration = speed_command / MAX_SPEED_COMMAND_DECEL_MPS2
    if ramp_duration <= 1e-9:
        return max(0.0, speed - 0.08) / drag

    input_slope = MAX_SPEED_COMMAND_DECEL_MPS2
    coefficient = (
        speed - MODEL["bu"] * speed_command / drag -
        MODEL["bu"] * input_slope / (drag * drag)
    )
    exp_term = math.exp(-drag * ramp_duration)
    speed_after_ramp = (
        MODEL["bu"] * input_slope / (drag * drag) + coefficient * exp_term
    )
    ramp_distance = (
        MODEL["bu"] / drag * speed_command * ramp_duration / 2.0 +
        MODEL["bu"] * speed_command / (drag * drag) +
        coefficient * (1.0 - exp_term) / drag
    )
    coast_distance = max(0.0, speed_after_ramp - 0.08) / drag
    return max(0.0, ramp_distance + coast_distance)


def restrict_cubic_bspline(spline, start_time, end_time):
    """返回给定时间区间的精确三次 B 样条表示."""
    degree = spline.k
    if degree != 3 or not start_time < end_time:
        raise ValueError("局部样条要求有效的三次 B 样条时间区间")
    domain_start = spline.t[degree]
    domain_end = spline.t[len(spline.c)]
    if start_time < domain_start - 1e-9 or end_time > domain_end + 1e-9:
        raise ValueError("局部样条区间超出全局 B 样条定义域")

    interior_knots = np.unique(
        spline.t[(spline.t > start_time + 1e-10) & (spline.t < end_time - 1e-10)]
    )
    breakpoints = np.r_[start_time, interior_knots, end_time]
    values = np.asarray(spline(breakpoints))
    derivatives = np.asarray(spline.derivative()(breakpoints))
    controls = []
    for index, (left, right) in enumerate(zip(breakpoints[:-1], breakpoints[1:])):
        duration = right - left
        segment = np.vstack((
            values[index],
            values[index] + duration * derivatives[index] / 3.0,
            values[index + 1] - duration * derivatives[index + 1] / 3.0,
            values[index + 1],
        ))
        controls.extend(segment if index == 0 else segment[1:])

    local_knots = [start_time] * 4
    for knot in interior_knots:
        local_knots.extend([float(knot)] * 3)
    local_knots.extend([end_time] * 4)
    return np.asarray(controls), np.asarray(local_knots, dtype=float) - start_time


def modulated_publish_period(elapsed_s, base_period_s, frequency_hz, fraction):
    """根据指定平均值计算正弦变化率的下一个时间区间."""
    if frequency_hz <= 0.0 or fraction == 0.0:
        return float(base_period_s)
    omega = 2.0 * math.pi * frequency_hz

    def accumulated_cycles(time_s):
        phase_cycles = time_s / base_period_s
        phase_cycles += fraction * (
            math.cos(PUBLISH_PHASE_RAD) -
            math.cos(omega * time_s + PUBLISH_PHASE_RAD)
        ) / (base_period_s * omega)
        return phase_cycles

    target_cycles = accumulated_cycles(elapsed_s) + 1.0
    low = elapsed_s
    high = elapsed_s + 1.5 * base_period_s / (1.0 - fraction)
    for _ in range(40):
        middle = 0.5 * (low + high)
        if accumulated_cycles(middle) < target_cycles:
            low = middle
        else:
            high = middle
    return float(high - elapsed_s)


def random_publish_period(rng, minimum_hz=1.0, maximum_hz=10.0):
    """均匀随机采样发布频率，并换算为对应的时间间隔."""
    if minimum_hz <= 0.0 or maximum_hz < minimum_hz:
        raise ValueError("发布频率范围无效")
    return 1.0 / float(rng.uniform(minimum_hz, maximum_hz))


def apply_lateral_replan(controls, lateral_offset_m, ramp_length_m=8.0):
    """平滑偏移局部样条，同时保持首个位置点不变."""
    result = np.asarray(controls, dtype=float).copy()
    if len(result) < 2 or abs(lateral_offset_m) <= 1e-12:
        return result
    if ramp_length_m <= 0.0:
        raise ValueError("ramp_length_m 必须为正数")

    control_steps = np.diff(result[:, :2], axis=0)
    control_distances = np.r_[0.0, np.cumsum(np.linalg.norm(control_steps, axis=1))]
    phase = np.clip(control_distances / ramp_length_m, 0.0, 1.0)
    envelope = phase * phase * (3.0 - 2.0 * phase)
    original_xy = result[:, :2].copy()
    for index in range(len(result)):
        low = max(0, index - 1)
        high = min(len(result) - 1, index + 1)
        tangent = original_xy[high] - original_xy[low]
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm <= 1e-12:
            continue
        normal = np.array([-tangent[1], tangent[0]]) / tangent_norm
        result[index, :2] += lateral_offset_m * envelope[index] * normal

    tangents = np.gradient(result[:, :2], axis=0)
    heading = np.unwrap(np.arctan2(tangents[:, 1], tangents[:, 0]) - math.pi / 2.0)
    result[:, 2] = heading
    return result


def _simulate_route(name, speed, x_offset_m, y_offset_m, initial_heading_deg):
    """从指定初始航向积分测试路线，拟合位姿样条并返回生成质量指标."""
    raw = geometry(name)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(raw, axis=0), axis=1))]
    centreline = make_interp_spline(arc, raw, k=3, axis=0)
    dense_s = np.arange(0.0, arc[-1], 0.2)
    dense_s = np.r_[dense_s, arc[-1]]
    dense_xy = centreline(dense_s)
    dense_xy += np.array([x_offset_m, y_offset_m])

    # 从模型测试初始状态出发，再接入偏移后的路线。
    r0 = 0.0
    v0 = 0.0
    phi0 = -math.radians(initial_heading_deg)
    state = np.array([0.0, 0.0, phi0, 0.0, v0, r0])
    progress = 0.0
    time_s = 0.0
    next_sample = 0.0
    sample_times = []
    samples = []
    initial_pose_derivative = None
    saturated = 0
    total_steps = 0
    max_time = min(MAX_SIM_TIME_S, arc[-1] / max(speed, 0.1) * 3.0 + 120.0)
    lookahead = max(6.0, 2.5 * speed)
    previous_speed_command = 0.0
    previous_yaw_command = 0.0

    while time_s <= max_time:
        low = max(0, int(np.searchsorted(dense_s, max(0.0, progress - 2.0))) - 1)
        high = min(
            len(dense_s),
            int(np.searchsorted(dense_s, min(arc[-1], progress + lookahead + 8.0))) + 1,
        )
        candidates = dense_xy[low:high]
        offsets = candidates - state[:2]
        nearest = low + int(np.argmin(np.einsum("ij,ij->i", offsets, offsets)))
        progress = max(progress, float(dense_s[nearest]))
        remaining = max(0.0, arc[-1] - progress)
        target_speed = min(speed, math.sqrt(2.0 * ROUTE_MAX_DECEL_MPS2 * remaining))
        desired_u_dot = max(-ROUTE_MAX_DECEL_MPS2, min(0.25, 1.5 * (target_speed - state[3])))
        speed_command = (desired_u_dot - MODEL["au"] * state[3]) / MODEL["bu"]
        speed_command = max(0.0, min(MAX_SPEED_COMMAND_MPS, speed_command))
        speed_step = (MAX_SPEED_COMMAND_ACCEL_MPS2 if speed_command >= previous_speed_command
                      else MAX_SPEED_COMMAND_DECEL_MPS2) * SIM_DT_S
        speed_command = max(
            previous_speed_command - speed_step,
            min(previous_speed_command + speed_step, speed_command),
        )
        target_s = min(arc[-1], progress + lookahead)
        target_index = min(len(dense_s) - 1, int(np.searchsorted(dense_s, target_s)))
        target = dense_xy[target_index]
        line_course = math.atan2(target[1] - state[1], target[0] - state[0])
        ground_course = math.atan2(
            state[4] * math.sin(state[2]) + state[3] * math.cos(state[2]),
            -state[3] * math.sin(state[2]) + state[4] * math.cos(state[2]),
        )

        ground_speed = math.hypot(state[3], state[4])
        course_error = _wrap(line_course - ground_course)
        desired_r = 2.0 * ground_speed * math.sin(course_error) / lookahead
        desired_r = max(-0.85 * MAX_YAW_RATE_RADPS, min(0.85 * MAX_YAW_RATE_RADPS, desired_r))
        r_dot_target = max(
            -MAX_YAW_ACCEL_RADPS2, min(MAX_YAW_ACCEL_RADPS2, 1.5 * (desired_r - state[5]))
        )
        yaw_command = (
            r_dot_target - MODEL["ar1"] * state[4] - MODEL["ar2"] * state[5] - MODEL["br0"]
        ) / MODEL["br"]
        yaw_command = max(-MAX_YAW_RATE_RADPS, min(MAX_YAW_RATE_RADPS, yaw_command))
        yaw_step = MAX_YAW_COMMAND_ACCEL_RADPS2 * SIM_DT_S
        yaw_command = max(
            previous_yaw_command - yaw_step,
            min(previous_yaw_command + yaw_step, yaw_command),
        )
        if abs(yaw_command) >= MAX_YAW_RATE_RADPS - 1e-9:
            saturated += 1
        command = (speed_command, yaw_command)
        previous_speed_command = speed_command
        previous_yaw_command = yaw_command
        if initial_pose_derivative is None:
            initial_pose_derivative = _model_derivative(state, command)[:3].copy()

        if time_s + 1e-9 >= next_sample:
            sample_times.append(time_s)
            samples.append(state.copy())
            next_sample += SPLINE_SAMPLE_PERIOD_S
        if (
            progress >= arc[-1] - 0.2
            and math.hypot(state[3], state[4]) < 0.05
            and abs(state[5]) < 0.02
        ):
            break
        state = _rk4(state, command, SIM_DT_S)
        time_s += SIM_DT_S
        total_steps += 1

    if time_s > max_time:
        raise RuntimeError(f"{name}: 动力学仿真超时，路径进度 {progress:.1f}/{arc[-1]:.1f} m")
    if len(samples) < 4:
        raise RuntimeError(f"{name}: 动力学仿真采样不足")
    # 通过模型积分加入一段停留，使样条平稳到达零速终点，
    # 避免末端节点间隔过短或出现人为制动尖峰。
    dwell_end = next_sample + STOP_DWELL_S
    while time_s + 1e-9 < dwell_end:
        state = _rk4(state, (0.0, 0.0), SIM_DT_S)
        time_s += SIM_DT_S
        total_steps += 1
        if time_s + 1e-9 >= next_sample and next_sample < dwell_end - 1e-9:
            sample_times.append(next_sample)
            samples.append(state.copy())
            next_sample += SPLINE_SAMPLE_PERIOD_S
    sample_times.append(dwell_end)
    samples.append(state.copy())
    time_s = dwell_end
    samples = np.asarray(samples)
    spline = make_interp_spline(
        np.asarray(sample_times),
        samples[:, :3],
        k=3,
        axis=0,
        bc_type=([(1, initial_pose_derivative)], [(1, np.zeros(3))]),
    )
    metrics = dynamic_reference_metrics(spline.c, spline.t, spline.k)
    metrics["simulation_steps"] = int(total_steps)
    metrics["yaw_command_saturated_fraction"] = float(saturated / max(total_steps, 1))
    return spline.c[:, :2].copy(), spline.t.copy(), spline.c[:, 2].copy(), metrics


@lru_cache(maxsize=24)
def _cached_simulation(name, speed, x_offset_m, y_offset_m, initial_heading_deg):
    """按初始状态键复用完整路线的动力学积分结果."""
    return _simulate_route(name, speed, x_offset_m, y_offset_m, initial_heading_deg)


def spline_data(
    name, speed=None, x_offset_m=30.0 / math.sqrt(2.0), y_offset_m=30.0 / math.sqrt(2.0),
    initial_heading_deg=0.0,
):
    """返回插值样条的控制点和节点向量，使用固定测试采样周期."""
    speed = ROUTE_SPEED_MPS[name] if speed is None else float(speed)
    controls, knots, phi, _ = _cached_simulation(
        name, speed, float(x_offset_m), float(y_offset_m), float(initial_heading_deg))
    return controls, knots, phi


def route_info(
    name, x_offset_m=30.0 / math.sqrt(2.0), y_offset_m=30.0 / math.sqrt(2.0),
    initial_heading_deg=0.0,
):
    """返回路线长度、目标航速、类型及动力学生成质量等测试元数据."""
    if name == COMPOSITE_ROUTE:
        return {
            "name": name,
            "label": LABELS[name],
            "length_m": 600.0 + 2.0 * math.pi * (7.0 + 10.0 + 15.0),
            "nominal_speed_mps": 4.0,
            "duration_s": 600.0 / 3.0 + 2.0 * math.pi * (7.0 + 10.0 + 15.0) / 1.5,
            "initial_heading_deg": initial_heading_deg,
            "start_offset_x_m": x_offset_m,
            "start_offset_y_m": y_offset_m,
        }
    speed = ROUTE_SPEED_MPS[name]
    controls, knots, phi, metrics = _cached_simulation(
        name, speed, float(x_offset_m), float(y_offset_m), float(initial_heading_deg)
    )
    spline = BSpline(knots, np.c_[controls, phi], 3)
    t = np.linspace(knots[3], knots[len(controls)], 10001)
    p, dp, ddp = spline(t)[:, :2], spline.derivative()(t)[:, :2], spline.derivative(2)(t)[:, :2]
    curvature = np.abs(dp[:, 0] * ddp[:, 1] - dp[:, 1] * ddp[:, 0]) / np.maximum(
        np.linalg.norm(dp, axis=1) ** 3, 1e-9
    )
    return dict(
        name=name,
        label=LABELS[name],
        length_m=float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
        minimum_radius_m=float(1 / max(curvature)),
        initial_tangent_enu_deg=float(np.degrees(np.arctan2(dp[0, 1], dp[0, 0]))),
        nominal_speed_mps=speed,
        initial_heading_deg=initial_heading_deg,
        start_offset_x_m=x_offset_m,
        start_offset_y_m=y_offset_m,
        start_gap_m=float(math.hypot(x_offset_m, y_offset_m)),
        square_corner_radius_m=20.0 if name == "square" else None,
        **metrics,
    )


class ComplexRoutePublisher(BsplineReferencePublisher):
    """沿完整全局动力学样条裁出 50 m 局部片段，支持发布周期及改道扰动."""

    # 以 100 Hz 轮询，使发布器能够实现低于 10 Hz 的周期变化。
    PUBLISH_PERIOD_S = PUBLISH_TIMER_TICK_S
    START_DELAY_S = 0.0

    def __init__(self):
        """加载发布扰动参数，创建复合路线模型并初始化全局投影进度."""
        super().__init__()
        self.interval = 0.1
        self.publish_jitter_hz = float(os.environ.get("USV_NMPC_PUBLISH_JITTER_HZ", "0"))
        self.publish_rate_mode = os.environ.get("USV_NMPC_PUBLISH_RATE_MODE", "sinusoidal")
        self.publish_seed = int(os.environ.get("USV_NMPC_PUBLISH_SEED", "20261108"))
        self.publish_rate_min_hz = float(
            os.environ.get("USV_NMPC_PUBLISH_RATE_MIN_HZ", "1"))
        self.publish_rate_max_hz = float(
            os.environ.get("USV_NMPC_PUBLISH_RATE_MAX_HZ", "10"))
        self.interval_jitter_fraction = float(
            os.environ.get("USV_NMPC_PUBLISH_INTERVAL_JITTER_FRACTION", "0.3"))
        self.local_deviation_stddev_m = float(
            os.environ.get("USV_NMPC_LOCAL_DEVIATION_STDDEV_M", "0"))
        self.local_deviation_seed = int(
            os.environ.get("USV_NMPC_LOCAL_DEVIATION_SEED", "20261107"))
        if self.publish_jitter_hz < 0.0:
            raise ValueError("publish_jitter_hz 不能为负数")
        if self.publish_rate_mode not in ("fixed", "sinusoidal", "random"):
            raise ValueError("publish_rate_mode 必须是 fixed、sinusoidal 或 random")
        if self.publish_rate_min_hz <= 0.0 or self.publish_rate_max_hz < self.publish_rate_min_hz:
            raise ValueError("发布频率范围无效")
        if not 0.0 <= self.interval_jitter_fraction < 1.0:
            raise ValueError("interval_jitter_fraction 必须在 [0, 1) 内")
        if self.local_deviation_stddev_m < 0.0:
            raise ValueError("local_deviation_stddev_m 不能为负数")
        random_samples = int(math.ceil(MAX_SIM_TIME_S / self.interval)) + 2
        random_source = np.random.default_rng(self.local_deviation_seed)
        self.publish_random_source = np.random.default_rng(self.publish_seed)
        self.local_deviation_samples = random_source.normal(
            0.0, self.local_deviation_stddev_m, random_samples)
        if self.local_deviation_stddev_m > 0.0:
            limit = 2.0 * self.local_deviation_stddev_m
            self.local_deviation_samples = np.clip(
                self.local_deviation_samples, -limit, limit)
        self.publisher_start_time = time.monotonic()
        self.next_publish_time = None
        self.route = CompositeRouteModel()
        self.global_spline = None
        self.global_spline_epoch = None
        self.global_sample_times = None
        self.global_sample_xy = None
        self.global_station = None
        self.progress_m = 0.0
        self.terminal_message = None
        self.last_path_publish_time = 0.0
        self.get_logger().info(
            "发布复合路线 R7/直线200m/R10/直线200m/R15/直线200m；"
            "全局路线按船体动力学生成，局部段长 50 m；"
            f"发布模式={self.publish_rate_mode}，"
            f"随机频率范围={self.publish_rate_min_hz:g}～{self.publish_rate_max_hz:g} Hz，"
            f"发布随机种子={self.publish_seed}，正弦调制={self.publish_jitter_hz:g} Hz，"
            f"随机改道标准差={self.local_deviation_stddev_m:.2f} m")

    def _on_reset(self, message):
        """复位父发布器与全局样条缓存，使下一次发布从当前船位重新生成路线."""
        super()._on_reset(message)
        self.progress_m = 0.0
        self.global_spline = None
        self.global_spline_epoch = None
        self.global_sample_times = None
        self.global_sample_xy = None
        self.global_station = None
        self.terminal_message = None

    def _on_odom(self, message):
        """缓存坐标系匹配的里程计，并据最新船位更新全局投影进度."""
        if message.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f"忽略坐标系为 {message.header.frame_id!r} 的里程计，"
                f"期望 {self.frame_id!r}")
            return
        self.odom = message
        if self.global_spline is not None:
            position = message.pose.pose.position
            self._project_to_global(np.array([position.x, position.y]))

    def _message_start_stamp(self, spline_time):
        """
        把样条参数时间映射为共享绝对时间戳，滚动发布时保持相位一致.

        :param spline_time: 局部段在全局样条中的起始参数时间，单位 s。

        :return: ROS 时间消息；首次调用建立全局时间基准。
        """
        if self.global_spline_epoch is None:
            self.global_spline_epoch = self.get_clock().now() - Duration(
                seconds=float(spline_time))
        return (self.global_spline_epoch + Duration(
            seconds=float(spline_time))).to_msg()

    def _make_trajectory(self, odom):
        """按当前船位投影裁出前方局部样条；投影超限或片段无效时抛出 RuntimeError."""
        if self.terminal_message is not None:
            return self.terminal_message
        position = odom.pose.pose.position
        yaw = _odom_navigation_yaw(odom.pose.pose.orientation)
        twist = odom.twist.twist
        state = np.array([
            position.x, position.y, yaw, twist.linear.y,
            twist.linear.x, twist.angular.z,
        ])
        if self.global_spline is None:
            self.global_spline, generation = self.route.generate_global_spline(state)
            self._index_global_spline()
            self.progress_m = 0.0
            self._save_global_spline(generation)
            duration = self.global_spline.t[len(self.global_spline.c)] - \
                self.global_spline.t[3]
            self.get_logger().info(
                f"完整全局动力学样条已生成：{len(self.global_spline.c)} 个控制点，"
                f"长度 {self.global_station[-1]:.1f} m，"
                f"时长 {duration:.1f} s")

        _, start_gap = self._project_to_global(state[:2])
        if start_gap > MAX_GLOBAL_PROJECTION_ERROR_M:
            raise RuntimeError(
                f"当前船位距全局样条最近点 {start_gap:.2f} m，超过 "
                f"{MAX_GLOBAL_PROJECTION_ERROR_M:.1f} m 起点接受阈值")
        remaining = max(0.0, self.global_station[-1] - self.progress_m)
        start_station = min(self.progress_m, max(0.0, self.global_station[-1] - 0.1))
        start_time = self._time_at_station(start_station)
        end_time = self._time_at_station(min(
            self.global_station[-1], self.progress_m + ROLLING_SEGMENT_LENGTH_M))
        global_end = self.global_spline.t[len(self.global_spline.c)]
        if remaining < ROLLING_SEGMENT_LENGTH_M:
            end_time = global_end
        if end_time - start_time <= 1e-4:
            raise RuntimeError("全局样条剩余时间不足，无法发布有效局部片段")
        controls, knots = restrict_cubic_bspline(self.global_spline, start_time, end_time)
        elapsed = max(0.0, time.monotonic() - self.publisher_start_time)
        deviation_index = min(
            len(self.local_deviation_samples) - 1,
            int(elapsed / ROLLING_PUBLISH_PERIOD_S),
        )
        lateral_deviation = float(self.local_deviation_samples[deviation_index])
        if self.local_deviation_stddev_m > 0.0:
            controls = apply_lateral_replan(controls, lateral_deviation)

        message = UsvBspline()
        message.order = 3
        message.traj_id = self.traj_id
        message.start_time = self._message_start_stamp(start_time)
        message.knots = [float(value) for value in knots]
        message.control_points = [
            UsvBsplineControlPoint(
                x=float(point[0]), y=float(point[1]), phi=float(point[2]))
            for point in controls
        ]
        if remaining <= 0.1:
            self.terminal_message = message
        return message

    def _index_global_spline(self):
        """按固定时间间隔采样全局样条，建立位置、参数时间及累计弧长索引."""
        start = self.global_spline.t[self.global_spline.k]
        end = self.global_spline.t[len(self.global_spline.c)]
        self.global_sample_times = np.arange(start, end, GLOBAL_MODEL_DT_S)
        self.global_sample_times = np.r_[self.global_sample_times, end]
        self.global_sample_xy = np.asarray(self.global_spline(self.global_sample_times))[:, :2]
        steps = np.linalg.norm(np.diff(self.global_sample_xy, axis=0), axis=1)
        self.global_station = np.r_[0.0, np.cumsum(steps)]

    def _project_to_global(self, xy):
        """
        在单调前向窗口内投影船位，仅距离合格时推进全局进度.

        :param xy: 当前船位 [东向, 北向]，单位 m。

        :return: (候选投影参数时间(s), 投影距离(m))；被拒候选不改变已接受进度。
        """
        xy = np.asarray(xy, dtype=float)
        total = float(self.global_station[-1])
        if self.progress_m >= total:
            end_time = float(self.global_sample_times[-1])
            end_xy = self.global_sample_xy[-1]
            return end_time, float(np.linalg.norm(end_xy - xy))

        window_end = min(total, self.progress_m + PROJECTION_FORWARD_WINDOW_M)
        low = max(0, int(np.searchsorted(
            self.global_station, self.progress_m, side="right")) - 1)
        high = min(
            len(self.global_station) - 1,
            int(np.searchsorted(self.global_station, window_end, side="left")),
        )
        high = max(low + 1, high)
        segment_indices = np.arange(low, high)
        segments = self.global_sample_xy[segment_indices + 1] - \
            self.global_sample_xy[segment_indices]
        squared_lengths = np.einsum("ij,ij->i", segments, segments)
        offsets = xy - self.global_sample_xy[segment_indices]
        spans = self.global_station[segment_indices + 1] - self.global_station[segment_indices]
        min_ratios = np.clip(
            (self.progress_m - self.global_station[segment_indices]) /
            np.maximum(spans, 1e-12), 0.0, 1.0)
        max_ratios = np.clip(
            (window_end - self.global_station[segment_indices]) /
            np.maximum(spans, 1e-12), 0.0, 1.0)
        ratios = np.einsum("ij,ij->i", offsets, segments) / np.maximum(squared_lengths, 1e-12)
        ratios = np.minimum(np.maximum(ratios, min_ratios), max_ratios)
        projections = self.global_sample_xy[segment_indices] + ratios[:, None] * segments
        distances = np.linalg.norm(projections - xy, axis=1)
        candidate_stations = self.global_station[segment_indices] + ratios * spans
        index = int(np.argmin(distances))
        candidate = float(np.clip(candidate_stations[index], self.progress_m, window_end))
        projected_time = self._time_at_station(candidate)
        projected_xy = projections[index]
        distance = float(np.linalg.norm(projected_xy - xy))
        if distance <= MAX_GLOBAL_PROJECTION_ERROR_M:
            self.progress_m = candidate
        return float(projected_time), distance

    def _time_at_station(self, station):
        """在线性弧长索引上插值得到全局样条参数时间，输入 m，输出 s."""
        if station >= self.global_station[-1] - 1e-9:
            return float(self.global_sample_times[-1])
        index = int(np.searchsorted(self.global_station, station, side="right") - 1)
        index = max(0, min(index, len(self.global_station) - 2))
        span = self.global_station[index + 1] - self.global_station[index]
        ratio = (station - self.global_station[index]) / max(span, 1e-12)
        return float(
            self.global_sample_times[index] +
            ratio * (self.global_sample_times[index + 1] - self.global_sample_times[index]))

    def _save_global_spline(self, generation):
        """启用测试输出目录时保存全局样条及生成质量指标."""
        output = os.environ.get("USV_NMPC_TRACKING_OUTPUT")
        if not output:
            return
        path = Path(output) / "global_reference_spline.json"
        path.write_text(json.dumps({
            "degree": self.global_spline.k,
            "knots": self.global_spline.t.tolist(),
            "controls": np.asarray(self.global_spline.c).tolist(),
            "generation": generation,
        }, ensure_ascii=False, indent=2))

    def _publish(self):
        """按配置发布周期裁剪和发布参考，并以较低频率刷新显示路径."""
        if self.odom is None:
            return
        now_s = time.monotonic()
        if self.next_publish_time is not None and now_s < self.next_publish_time:
            return
        try:
            self.trajectory = self._make_trajectory(self.odom)
        except (ValueError, RuntimeError) as exc:
            self.get_logger().error(f"滚动 B 样条生成失败：{exc}")
            return
        self.publisher.publish(self.trajectory)
        published_at = time.monotonic()
        scheduled_elapsed = published_at - self.publisher_start_time
        if self.publish_rate_mode == "random":
            period = random_publish_period(
                self.publish_random_source,
                self.publish_rate_min_hz,
                self.publish_rate_max_hz,
            )
        elif self.publish_rate_mode == "fixed":
            period = self.interval
        else:
            period = modulated_publish_period(
                scheduled_elapsed, self.interval, self.publish_jitter_hz,
                self.interval_jitter_fraction)
        next_publish_time = published_at + period
        completion_time = time.monotonic()
        while next_publish_time <= completion_time:
            next_elapsed = next_publish_time - self.publisher_start_time
            if self.publish_rate_mode == "random":
                next_period = random_publish_period(
                    self.publish_random_source,
                    self.publish_rate_min_hz,
                    self.publish_rate_max_hz,
                )
            elif self.publish_rate_mode == "fixed":
                next_period = self.interval
            else:
                next_period = modulated_publish_period(
                    next_elapsed, self.interval, self.publish_jitter_hz,
                    self.interval_jitter_fraction)
            next_publish_time += next_period
        self.next_publish_time = next_publish_time
        now_s = time.monotonic()
        if now_s - self.last_path_publish_time >= 1.0:
            self._publish_reference_path()
            self.last_path_publish_time = now_s


class CompositeRouteModel:
    """维护复合设计路线和速度包络，以辨识模型积分并拟合完整全局样条."""

    def __init__(self):
        """建立圆形和直线组成的设计路线，并按转艏率及加减速限制生成速度包络."""
        points = [np.zeros(2)]
        curvature = [1.0 / 7.0]

        def append_line(length):
            start = points[-1].copy()
            count = int(math.ceil(length / 0.1))
            for point in np.linspace(start, start + np.array([length, 0.0]), count + 1)[1:]:
                points.append(point)
                curvature.append(0.0)

        def append_circle(radius):
            start = points[-1].copy()
            count = max(100, int(math.ceil(2.0 * math.pi * radius / 0.1)))
            angles = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, count + 1)[1:]
            for angle in angles:
                points.append(np.array([
                    start[0] + radius * math.cos(angle),
                    start[1] + radius + radius * math.sin(angle),
                ]))
                curvature.append(1.0 / radius)

        append_circle(7.0)
        append_line(200.0)
        append_circle(10.0)
        append_line(200.0)
        append_circle(15.0)
        append_line(200.0)
        self.xy = np.asarray(points)
        self.station = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.xy, axis=0), axis=1))]
        self.curve = make_interp_spline(self.station, self.xy, k=3, axis=0)
        self.total_length = float(self.station[-1])
        speed_limit = np.minimum(
            4.0,
            REFERENCE_YAW_RATE_LIMIT_RADPS / np.maximum(np.abs(curvature), 1e-9),
        )
        speed_limit[-1] = 0.0
        for index in range(len(speed_limit) - 2, -1, -1):
            ds = self.station[index + 1] - self.station[index]
            speed_limit[index] = min(
                speed_limit[index],
                math.sqrt(speed_limit[index + 1] ** 2 + 2.0 * ROUTE_MAX_DECEL_MPS2 * ds),
            )
        for index in range(1, len(speed_limit)):
            ds = self.station[index] - self.station[index - 1]
            speed_limit[index] = min(
                speed_limit[index],
                math.sqrt(speed_limit[index - 1] ** 2 + 2.0 * 0.25 * ds),
            )
        self.speed_limit = speed_limit

    def nearest_progress(self, x, y, progress, forward=12.0):
        """
        在局部窗口内投影设计路线，并返回不回退的几何进度.

        :param x: 东向位置，单位 m。
        :param y: 北向位置，单位 m。
        :param progress: 已接受的弧长进度，单位 m。
        :param forward: 向前搜索窗口长度，单位 m。

        :return: 单调不减的弧长进度，单位 m。
        """
        segment_count = len(self.station) - 1
        low = min(
            segment_count - 1,
            max(
                0,
                int(np.searchsorted(
                    self.station, max(0.0, progress - 2.0), side="right")) - 1,
            ),
        )
        high = min(
            segment_count,
            int(np.searchsorted(
                self.station, min(self.total_length, progress + forward), side="left")) + 1,
        )
        high = max(low + 1, high)
        segments = self.xy[low + 1:high + 1] - self.xy[low:high]
        lengths_squared = np.einsum("ij,ij->i", segments, segments)
        offsets = np.array([x, y]) - self.xy[low:high]
        ratios = np.clip(
            np.einsum("ij,ij->i", offsets, segments) /
            np.maximum(lengths_squared, 1e-12),
            0.0,
            1.0,
        )
        projections = self.xy[low:high] + ratios[:, None] * segments
        distances_squared = np.einsum("ij,ij->i", projections - [x, y], projections - [x, y])
        index = int(np.argmin(distances_squared))
        candidate = self.station[low + index] + ratios[index] * math.sqrt(lengths_squared[index])
        return max(progress, float(candidate))

    def _command(self, state, progress, previous_command, dt, alignment_pending):
        """根据几何前视目标生成模型积分命令，包含起步对齐、限速和终点减速."""
        index = min(len(self.station) - 1, int(np.searchsorted(self.station, progress)))
        route_point = self.curve(progress)
        cross_track_error = float(np.linalg.norm(route_point - state[:2]))
        lookahead = max(
            3.0,
            1.5 * max(1.0, math.hypot(state[3], state[4])),
            3.0 * cross_track_error,
        )
        target_station = min(self.total_length, progress + lookahead)
        target = self.curve(target_station)
        delta = target - state[:2]
        if np.linalg.norm(delta) < 1e-8:
            tangent = self.curve.derivative(1)(target_station)
            line_course = math.atan2(tangent[1], tangent[0])
        else:
            line_course = math.atan2(delta[1], delta[0])
        desired_heading = line_course - math.pi / 2.0
        heading_error = _wrap(desired_heading - state[2])
        ground_speed = math.hypot(state[3], state[4])
        aligning_from_rest = alignment_pending and ground_speed < 0.35 and \
            abs(heading_error) > math.radians(5.0)
        target_speed = 0.0 if aligning_from_rest else float(self.speed_limit[index])
        remaining = max(0.0, self.total_length - progress)
        target_speed = min(target_speed, math.sqrt(2.0 * ROUTE_MAX_DECEL_MPS2 * remaining))
        accel_limit = 0.0 if aligning_from_rest else 0.25
        desired_accel = max(
            -ROUTE_MAX_DECEL_MPS2,
            min(accel_limit, 1.5 * (target_speed - state[3])),
        )
        speed_command = (desired_accel - MODEL["au"] * state[3]) / MODEL["bu"]
        speed_command = max(0.0, min(MAX_SPEED_COMMAND_MPS, speed_command))
        if remaining <= _speed_command_stop_distance(state[3], previous_command[0]) + 0.2:
            speed_command = 0.0
        speed_step = (MAX_SPEED_COMMAND_ACCEL_MPS2 if speed_command >= previous_command[0]
                      else MAX_SPEED_COMMAND_DECEL_MPS2) * dt
        speed_command = max(
            previous_command[0] - speed_step,
            min(previous_command[0] + speed_step, speed_command),
        )

        if aligning_from_rest:
            desired_r = 1.5 * heading_error
        else:
            east_dot = -state[3] * math.sin(state[2]) + state[4] * math.cos(state[2])
            north_dot = state[3] * math.cos(state[2]) + state[4] * math.sin(state[2])
            ground_course = (
                math.atan2(north_dot, east_dot)
                if math.hypot(east_dot, north_dot) > 1e-6 else state[2] + math.pi / 2.0
            )
            course_error = _wrap(line_course - ground_course)
            ground_speed = math.hypot(state[3], state[4])
            desired_r = 2.0 * ground_speed * math.sin(course_error) / lookahead
        desired_r = max(-0.75 * MAX_YAW_RATE_RADPS, min(0.75 * MAX_YAW_RATE_RADPS, desired_r))
        r_dot = max(
            -MAX_YAW_ACCEL_RADPS2,
            min(MAX_YAW_ACCEL_RADPS2, 1.5 * (desired_r - state[5])),
        )
        yaw_command = (
            r_dot - MODEL["ar1"] * state[4] - MODEL["ar2"] * state[5] - MODEL["br0"]
        ) / MODEL["br"]
        yaw_command = max(-MAX_YAW_RATE_RADPS, min(MAX_YAW_RATE_RADPS, yaw_command))
        yaw_step = MAX_YAW_COMMAND_ACCEL_RADPS2 * dt
        yaw_command = max(
            previous_command[1] - yaw_step,
            min(previous_command[1] + yaw_step, yaw_command),
        )
        return np.array([speed_command, yaw_command]), heading_error

    def generate_global_spline(self, initial_state):
        """完整仿真一次路线，再拟合模型生成的位姿序列."""
        state = np.asarray(initial_state, dtype=float).copy()
        command = np.zeros(2)
        progress = self.nearest_progress(state[0], state[1], 0.0, forward=25.0)
        sample_times = [0.0]
        states = [state.copy()]
        initial_derivative = _model_derivative(state, command)[:3].copy()
        next_sample = GLOBAL_SPLINE_SAMPLE_PERIOD_S
        time_s = 0.0
        terminal = False
        alignment_pending = True
        max_time = min(MAX_SIM_TIME_S, self.total_length / 1.5 + 300.0)
        while time_s < max_time:
            progress = self.nearest_progress(
                state[0], state[1], progress,
                forward=max(12.0, 2.5 * max(1.0, state[3]) + 8.0),
            )
            command, heading_error = self._command(
                state, progress, command, GLOBAL_MODEL_DT_S, alignment_pending)
            if alignment_pending and (
                math.hypot(state[3], state[4]) >= 0.35 or
                abs(heading_error) <= math.radians(5.0)
            ):
                alignment_pending = False
            state = _rk4(state, command, GLOBAL_MODEL_DT_S)
            state[3] = max(-MAX_SPEED_COMMAND_MPS, min(MAX_SPEED_COMMAND_MPS, state[3]))
            state[5] = max(-MAX_YAW_RATE_RADPS, min(MAX_YAW_RATE_RADPS, state[5]))
            time_s += GLOBAL_MODEL_DT_S
            at_route_end = progress >= self.total_length - 0.2 and \
                np.linalg.norm(state[:2] - self.xy[-1]) <= GLOBAL_ROUTE_ENDPOINT_TOLERANCE_M
            if at_route_end and math.hypot(state[3], state[4]) < 0.08 and abs(state[5]) < 0.03:
                terminal = True
                break
            if time_s + 1e-9 >= next_sample:
                sample_times.append(time_s)
                states.append(state.copy())
                next_sample += GLOBAL_SPLINE_SAMPLE_PERIOD_S
        if not terminal:
            raise RuntimeError(
                f"全局动力学路线仿真超时，进度 {progress:.1f}/{self.total_length:.1f} m，"
                f"位置 ({state[0]:.2f},{state[1]:.2f})、终点偏差 "
                f"{np.linalg.norm(state[:2] - self.xy[-1]):.2f} m，"
                f"u={state[3]:.3f} m/s、v={state[4]:.3f} m/s、r={state[5]:.3f} rad/s")

        dwell_end = max(
            time_s + GLOBAL_TERMINAL_DWELL_S,
            next_sample + GLOBAL_TERMINAL_DWELL_S,
        )
        while time_s + 1e-9 < dwell_end:
            state = _rk4(state, (0.0, 0.0), GLOBAL_MODEL_DT_S)
            state[3] = max(-MAX_SPEED_COMMAND_MPS, min(MAX_SPEED_COMMAND_MPS, state[3]))
            state[5] = max(-MAX_YAW_RATE_RADPS, min(MAX_YAW_RATE_RADPS, state[5]))
            time_s += GLOBAL_MODEL_DT_S
            if time_s + 1e-9 >= next_sample:
                sample_times.append(time_s)
                states.append(state.copy())
                next_sample += GLOBAL_SPLINE_SAMPLE_PERIOD_S
        end_derivative = _model_derivative(state, np.zeros(2))[:3].copy()
        if time_s - sample_times[-1] > 1e-8:
            sample_times.append(time_s)
            states.append(state.copy())
        spline = make_interp_spline(
            np.asarray(sample_times), np.asarray(states)[:, :3], k=3, axis=0,
            bc_type=([(1, initial_derivative)], [(1, end_derivative)]),
        )
        metrics = dynamic_reference_metrics(spline.c, spline.t, spline.k)
        metrics["geometry_progress_m"] = float(progress)
        metrics["geometry_length_m"] = self.total_length
        metrics["endpoint_error_m"] = float(np.linalg.norm(state[:2] - self.xy[-1]))
        metrics["initial_heading_rad"] = float(initial_state[2])
        metrics["start_east_m"] = float(initial_state[0])
        metrics["start_north_m"] = float(initial_state[1])
        metrics["dynamics_time_s"] = np.asarray(sample_times).tolist()
        metrics["dynamics_trajectory_xy"] = np.asarray(states)[:, :2].tolist()
        return spline, metrics


def main():
    """运行复合路线滚动样条发布节点，并在退出时释放 ROS 资源."""
    import rclpy

    rclpy.init()
    node = ComplexRoutePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
