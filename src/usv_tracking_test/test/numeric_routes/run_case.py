#!/usr/bin/env python3
"""使用工程数值无人船模型独立运行一条闭环路线测试."""

import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ["ROS_DOMAIN_ID"] = os.environ.get("USV_NMPC_ROS_DOMAIN_ID", "188")
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from nav_msgs.msg import Path as RosPath  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from scipy.interpolate import BSpline  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Bool, Float64MultiArray, String  # noqa: E402
from usv_interfaces.msg import UsvBspline  # noqa: E402
import yaml  # noqa: E402

ROOT = Path(__file__).resolve().parents[4]
OUTPUT = Path(os.environ["USV_NMPC_TRACKING_OUTPUT"])
LAT, LON = 29.506912001, 106.716974974
MAX_DURATION_S = float(os.environ.get("USV_NMPC_MAX_DURATION_S", "900"))
AFTER_FINISH_S = float(os.environ.get("USV_NMPC_AFTER_FINISH_S", "2"))


def write_json(path, value):
    """
    先写临时文件再替换目标，避免状态读取方看到未完成的 JSON.

    :param path: 目标文件路径。
    :param value: 可由 JSON 序列化的数据。
    """
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temporary.replace(path)


def main():
    """启动单次仿真跟踪测试，记录状态、参考、诊断和完成结果."""
    OUTPUT.mkdir(parents=True, exist_ok=True)
    route = {
        "mode": os.environ["USV_COMPLEX_ROUTE"],
        "speed_mps": float(os.environ.get("USV_NMPC_ROUTE_SPEED_MPS", "4")),
        "length_m": 600.0 + 2.0 * math.pi * 32.0,
        "lanes": int(os.environ.get("USV_NMPC_ROUTE_LANES", "1")),
        "spacing_m": float(os.environ.get("USV_NMPC_ROUTE_SPACING_M", "80")),
        "turn_bulge_m": float(os.environ.get("USV_NMPC_ROUTE_BULGE_M", "40")),
        "curve_amplitude_m": float(os.environ.get("USV_NMPC_ROUTE_CURVE_AMPLITUDE_M", "90")),
        "curve_half_wave_m": float(os.environ.get("USV_NMPC_ROUTE_CURVE_HALF_WAVE_M", "120")),
        "start_offset_m": float(os.environ.get("USV_NMPC_ROUTE_START_OFFSET_M", "0")),
        "start_offset_x_m": float(os.environ.get("USV_NMPC_ROUTE_START_OFFSET_X_M", "0")),
        "start_offset_y_m": float(os.environ.get("USV_NMPC_ROUTE_START_OFFSET_Y_M", "0")),
        "initial_heading_deg": float(os.environ.get("USV_NMPC_INITIAL_HEADING_DEG", "90")),
        "initial_phi_deg": float(os.environ.get("USV_NMPC_INITIAL_PHI_DEG", "0")),
        "reference_heading_deg": float(os.environ.get("USV_NMPC_REFERENCE_HEADING_DEG", "90")),
        "direction": -1 if os.environ["USV_NMPC_ROUTE_MODE"] == "reverse_straight" else 1,
    }
    source_configs = {
        "controller": Path(
            os.environ.get(
                "USV_NMPC_CONTROLLER_CONFIG",
                str(ROOT / "src/usv_nmpc_controller/config/usv_nmpc.yaml"),
            )
        ),
        "model": ROOT / "src/usv_model_sim/config/usv_model.yaml",
        "odom": ROOT / "src/usv_map/config/usv_map.yaml",
    }
    configs = {name: OUTPUT / f"{name}_config.yaml" for name in source_configs}
    for name, path in source_configs.items():
        configs[name].write_bytes(path.read_bytes())
    controller = yaml.safe_load(configs["controller"].read_text())
    controller_params = controller["/**"]["ros__parameters"]
    # 默认沿用工程配置中的预测时域和指令限值；仅显式指定的测试选项可覆盖这些参数。
    if "USV_NMPC_WEIGHT_LATERAL" in os.environ:
        controller_params["nmpc"]["weight_lateral"] = float(
            os.environ["USV_NMPC_WEIGHT_LATERAL"])
    configs["controller"].write_text(
        yaml.safe_dump(controller, allow_unicode=True, sort_keys=False)
    )
    model = yaml.safe_load(configs["model"].read_text())
    model_params = model["usv_model_node"]["ros__parameters"]
    if "USV_NMPC_MODEL_POSITION_NOISE_STDDEV_M" in os.environ:
        model_params["position_noise_stddev_m"] = float(
            os.environ["USV_NMPC_MODEL_POSITION_NOISE_STDDEV_M"])
    if "USV_NMPC_NOISE_SEED" in os.environ:
        model_params["noise_seed"] = int(os.environ["USV_NMPC_NOISE_SEED"])
    model_params["initial_east_m"] = route["start_offset_x_m"]
    model_params["initial_north_m"] = route["start_offset_y_m"]
    model_params["initial_heading_deg"] = route["initial_heading_deg"]
    configs["model"].write_text(yaml.safe_dump(model, allow_unicode=True, sort_keys=False))
    odom = yaml.safe_load(configs["odom"].read_text())
    odom_params = odom["usv_odometry_node"]["ros__parameters"]
    if "USV_NMPC_ODOM_POSITION_ALPHA" in os.environ:
        alpha = float(os.environ["USV_NMPC_ODOM_POSITION_ALPHA"])
        if not 0.0 < alpha <= 1.0:
            raise ValueError("USV_NMPC_ODOM_POSITION_ALPHA 必须在 (0, 1] 内")
        odom_params["position_measurement_alpha"] = alpha
    configs["odom"].write_text(yaml.safe_dump(odom, allow_unicode=True, sort_keys=False))

    executables = {
        "model": ROOT / "install/usv_model_sim/lib/usv_model_sim/usv_model_node",
        "odom": ROOT / "install/usv_map/lib/usv_map/usv_odometry_node",
        "controller": ROOT / "install/usv_nmpc_controller/lib/usv_nmpc_controller/usv_nmpc_node",
    }
    write_json(
        OUTPUT / "test_setup.json",
        {
            "simulation": "usv_model_sim numeric dynamics; not Gazebo",
            "ros_domain_id": int(os.environ["ROS_DOMAIN_ID"]),
            "noise_stddev_m": model_params["position_noise_stddev_m"],
            "noise_seed": model_params["noise_seed"],
            "odom_position_measurement_alpha": odom_params["position_measurement_alpha"],
            "heading_noise_stddev_deg": model_params["heading_noise_stddev_deg"],
            "control_frequency_hz": controller_params["control_frequency"],
            "nmpc_horizon_steps": controller_params["nmpc"]["horizon_steps"],
            "nmpc_dt_s": controller_params["nmpc"]["dt"],
            "nmpc_horizon_s": (
                controller_params["nmpc"]["horizon_steps"] * controller_params["nmpc"]["dt"]
            ),
            "max_speed_mps": model_params["max_speed_mps"],
            "min_speed_command_mps": controller_params["nmpc"]["min_speed_command_mps"],
            "max_reverse_speed_mps": controller_params["nmpc"]["max_reverse_speed_mps"],
            "weight_surge_delta": controller_params["nmpc"]["weight_surge_delta"],
            "weight_yaw_rate_delta": controller_params["nmpc"]["weight_yaw_rate_delta"],
            "max_speed_command_accel_mps2": controller_params["nmpc"][
                "max_speed_command_accel_mps2"
            ],
            "max_speed_command_decel_mps2": controller_params["nmpc"][
                "max_speed_command_decel_mps2"
            ],
            "max_yaw_rate_command_accel_deg_s2": math.degrees(
                controller_params["nmpc"]["max_yaw_rate_command_accel_radps2"]
            ),
            "reference": route,
            "initial_heading_deg": route["initial_heading_deg"],
            "initial_phi_deg": route["initial_phi_deg"],
            "initial_east_m": model_params["initial_east_m"],
            "initial_north_m": model_params["initial_north_m"],
            "initial_position_offset_m": math.hypot(
                model_params["initial_east_m"], model_params["initial_north_m"]
            ),
            "relative_initial_heading_clockwise_deg": float(
                os.environ["USV_NMPC_RELATIVE_HEADING_DEG"]
            )
            if "USV_NMPC_RELATIVE_HEADING_DEG" in os.environ
            else None,
            "reference_heading_deg": route["reference_heading_deg"],
            "reference_start_offset_m": route["start_offset_m"],
            "reference_start_offset_x_m": route["start_offset_x_m"],
            "reference_start_offset_y_m": route["start_offset_y_m"],
            "reference_start_offset_distance_m": math.hypot(
                route["start_offset_x_m"], route["start_offset_y_m"]
            ),
            "rolling_segment_length_m": 50.0,
            "rolling_publish_period_s": 0.1,
            "publish_jitter_hz": float(os.environ.get(
                "USV_NMPC_PUBLISH_JITTER_HZ", "0")),
            "publish_rate_mode": os.environ.get(
                "USV_NMPC_PUBLISH_RATE_MODE", "sinusoidal"),
            "publish_rate_min_hz": float(os.environ.get(
                "USV_NMPC_PUBLISH_RATE_MIN_HZ", "1")),
            "publish_rate_max_hz": float(os.environ.get(
                "USV_NMPC_PUBLISH_RATE_MAX_HZ", "10")),
            "publish_seed": int(os.environ.get(
                "USV_NMPC_PUBLISH_SEED", "20261108")),
            "publish_interval_jitter_fraction": float(os.environ.get(
                "USV_NMPC_PUBLISH_INTERVAL_JITTER_FRACTION", "0.3")),
            "local_deviation_stddev_m": float(os.environ.get(
                "USV_NMPC_LOCAL_DEVIATION_STDDEV_M", "0")),
            "local_deviation_seed": int(os.environ.get(
                "USV_NMPC_LOCAL_DEVIATION_SEED", "20261107")),
            "max_reference_start_distance_m": controller_params[
                "max_reference_start_distance_m"
            ],
            "finish_distance_m": controller_params["finish_distance_m"],
            "finish_speed_mps": controller_params["finish_speed_mps"],
            "finish_stable_duration_s": controller_params["finish_stable_duration_s"],
            "complex_route": os.environ["USV_COMPLEX_ROUTE"],
            "max_duration_s": MAX_DURATION_S,
            "observe_after_finish_s": AFTER_FINISH_S,
            "model_noise_seed": model_params["noise_seed"]
            if model_params["noise_seed"] >= 0 else "random_device",
            "executable_sha256": {
                name: hashlib.sha256(path.read_bytes()).hexdigest()
                for name, path in executables.items()
            },
        },
    )
    driver = OUTPUT / "reference_driver.py"
    driver.write_text(
        "import sys\n"
        f"sys.path.insert(0, {str(Path(__file__).resolve().parent)!r})\n"
        "from route_shapes import main\n"
        "main()\n"
    )

    rclpy.init()
    node = rclpy.create_node("nmpc_route_tracking_probe")
    origin_pub = node.create_publisher(NavSatFix, "/fix", 1)
    origin = NavSatFix(latitude=LAT, longitude=LON, altitude=0.0)
    start = time.monotonic()

    def elapsed():
        return time.monotonic() - start

    truth, odometry, commands, diagnostics, reference, states = [], [], [], [], [], []
    spline_starts = []
    global_reference_tree = None
    global_reference_points = None
    global_reference_tangents = None
    global_reference_station = None
    last_global_station = None
    last_spline_receive_time = None
    tracking_transitions = []
    latest_odom = (0.0, 0.0)
    finished_at = None
    error = None

    def on_state(message):
        if len(message.data) != 19:
            return
        d = message.data
        states.append([elapsed(), *d])
        scale = 6378137.0 * math.cos(math.radians(LAT))
        east = (d[14] - math.radians(LON)) * scale
        north = (d[13] - math.radians(LAT)) * 6378137.0
        measured_east = (d[7] - math.radians(LON)) * scale
        measured_north = (d[6] - math.radians(LAT)) * 6378137.0
        truth.append(
            [
                elapsed(),
                east,
                north,
                math.remainder(d[16], 2 * math.pi),
                d[10],
                d[9],
                d[12],
                measured_east,
                measured_north,
                measured_east - east,
                measured_north - north,
            ]
        )

    def on_odom(message):
        nonlocal latest_odom
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        latest_odom = (message.pose.pose.position.x, message.pose.pose.position.y)
        odometry.append(
            [
                elapsed(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                math.remainder(yaw, 2 * math.pi),
                message.twist.twist.linear.y,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            ]
        )

    def on_finished(message):
        nonlocal finished_at
        if message.data and finished_at is None:
            finished_at = elapsed()

    def on_path(message):
        reference[:] = [[pose.pose.position.x, pose.pose.position.y] for pose in message.poses]

    def on_bspline(message):
        nonlocal global_reference_tree, global_reference_points, global_reference_station
        nonlocal global_reference_tangents, last_global_station, last_spline_receive_time
        controls = np.array([
            [point.x, point.y, point.phi] for point in message.control_points
        ])
        start_time = float(message.knots[message.order])
        end_time = float(message.knots[len(message.control_points)])
        spline = BSpline(message.knots, controls, message.order)
        sampled = spline(np.linspace(start_time, end_time, 101))
        segment_length = float(np.linalg.norm(np.diff(sampled[:, :2], axis=0), axis=1).sum())
        start_xy = spline(start_time)[:2]
        odom_xy = latest_odom if latest_odom is not None else (math.nan, math.nan)
        start_gap = math.hypot(start_xy[0] - odom_xy[0], start_xy[1] - odom_xy[1])
        received_at = elapsed()
        receive_interval = (
            received_at - last_spline_receive_time
            if last_spline_receive_time is not None else math.nan
        )
        last_spline_receive_time = received_at
        global_spline_path = OUTPUT / "global_reference_spline.json"
        if global_reference_tree is None and global_spline_path.exists():
            global_data = json.loads(global_spline_path.read_text())
            global_spline = BSpline(
                global_data["knots"], np.asarray(global_data["controls"]),
                global_data["degree"])
            global_start = global_data["knots"][global_data["degree"]]
            global_end = global_data["knots"][len(global_data["controls"])]
            global_sample_times = np.linspace(global_start, global_end, 20000)
            global_reference_points = global_spline(global_sample_times)[:, :2]
            global_reference_tangents = global_spline.derivative()(global_sample_times)[:, :2]
            tangent_norms = np.maximum(
                np.linalg.norm(global_reference_tangents, axis=1, keepdims=True), 1e-12)
            global_reference_tangents = global_reference_tangents / tangent_norms
            global_reference_station = np.r_[
                0.0,
                np.cumsum(np.linalg.norm(np.diff(global_reference_points, axis=0), axis=1)),
            ]
            global_reference_tree = True
        if global_reference_tree is None:
            lookahead_deviation = math.nan
            path_deviation_p95 = math.nan
            path_deviation_max = math.nan
        else:
            local_station = np.r_[
                0.0, np.cumsum(np.linalg.norm(np.diff(sampled[:, :2], axis=0), axis=1))]
            previous_station = last_global_station if last_global_station is not None else 0.0
            station_low = max(0.0, previous_station - 1.0)
            station_high = station_low + 8.0
            low_index = int(np.searchsorted(global_reference_station, station_low, side="left"))
            high_index = min(
                len(global_reference_station),
                int(np.searchsorted(global_reference_station, station_high, side="right")),
            )
            start_candidates = global_reference_points[low_index:high_index]
            start_index = low_index + int(np.argmin(np.linalg.norm(
                start_candidates - start_xy, axis=1)))
            start_station = float(global_reference_station[start_index])
            last_global_station = max(previous_station, start_station)
            signed_deviations = []
            for local_s, local_xy in zip(local_station, sampled[:, :2]):
                expected_station = start_station + float(local_s)
                expected_low = max(0.0, expected_station - 2.0)
                expected_high = expected_station + 2.0
                low = int(np.searchsorted(
                    global_reference_station, expected_low, side="left"))
                high = min(
                    len(global_reference_station),
                    int(np.searchsorted(
                        global_reference_station, expected_high, side="right")),
                )
                candidates = global_reference_points[low:high]
                nearest = low + int(np.argmin(np.linalg.norm(candidates - local_xy, axis=1)))
                offset = local_xy - global_reference_points[nearest]
                tangent = global_reference_tangents[nearest]
                signed_deviations.append(
                    -tangent[1] * offset[0] + tangent[0] * offset[1])
            signed_deviations = np.asarray(signed_deviations)
            lookahead_index = int(np.argmin(np.abs(local_station - 10.0)))
            lookahead_deviation = float(signed_deviations[lookahead_index])
            path_deviation_p95 = float(np.percentile(np.abs(signed_deviations), 95))
            path_deviation_max = float(np.max(np.abs(signed_deviations)))
        spline_starts.append([
            received_at, start_xy[0], start_xy[1], odom_xy[0], odom_xy[1],
            start_gap, end_time - start_time, segment_length, len(message.control_points),
            receive_interval,
            1.0 / receive_interval if receive_interval > 0.0 else math.nan,
            lookahead_deviation, path_deviation_p95, path_deviation_max,
            message.start_time.sec + message.start_time.nanosec * 1e-9,
        ])
        write_json(
            OUTPUT / "reference_spline.json",
            {
                "degree": message.order,
                "knots": list(message.knots),
                "controls": [[point.x, point.y, point.phi] for point in message.control_points],
            },
        )

    node.create_subscription(Float64MultiArray, "/USV_state", on_state, 100)
    node.create_subscription(Odometry, "/odom", on_odom, 100)
    node.create_subscription(
        Twist, "/cmd", lambda m: commands.append([elapsed(), m.linear.x, m.angular.z]), 100
    )
    node.create_subscription(
        Float64MultiArray,
        "/nmpc_tracking_diagnostics",
        lambda m: diagnostics.append([elapsed(), *m.data]),
        100,
    )
    node.create_subscription(
        String,
        "/nmpc_tracking_state",
        lambda m: tracking_transitions.append({"wall_s": elapsed(), "state": m.data}),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        Bool,
        "/USV_tracking_finished",
        on_finished,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        RosPath,
        "/planning/bspline_path",
        on_path,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        UsvBspline,
        "/planning/bspline",
        on_bspline,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )

    processes, logs = [], []

    def launch(name, args):
        log = open(OUTPUT / f"{name}.log", "w")
        logs.append(log)
        processes.append(
            subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        )

    def status():
        last = diagnostics[-1] if diagnostics else None
        # FINISHED 状态会再次发布上次代价值，但不会再次执行求解。
        solve_rows = [row for row in diagnostics if len(row) < 70 or row[68] != 6.0]
        return {
            "elapsed_s": elapsed(),
            "finished_at_s": finished_at,
            "state_samples": len(truth),
            "diagnostic_samples": len(diagnostics),
            "bspline_messages": len(spline_starts),
            "publish_rate_hz": (
                (len(spline_starts) - 1) /
                (spline_starts[-1][0] - spline_starts[0][0])
                if len(spline_starts) > 1 else None
            ),
            "start_gap_max_m": max((row[5] for row in spline_starts), default=None),
            "start_gap_over_2m_count": sum(row[5] > 2.0 for row in spline_starts),
            "segment_length_min_m": min((row[7] for row in spline_starts), default=None),
            "segment_length_max_m": max((row[7] for row in spline_starts), default=None),
            "successful_solves": sum(row[14] == 1.0 for row in solve_rows),
            "tracking_transitions": tracking_transitions,
            "progress_m": last[4] if last else None,
            "endpoint_distance_m": last[55] if last else None,
            "latest_state": truth[-1] if truth else None,
            "latest_command": commands[-1] if commands else None,
            "error": error,
            "route_name": os.environ["USV_COMPLEX_ROUTE"],
            "relative_angle_deg": float(os.environ["USV_NMPC_RELATIVE_HEADING_DEG"]),
        }

    def save():
        diagnostic_count = len(diagnostics[0]) - 1 if diagnostics else 69
        arrays = {
            "state": (states, "wall_s," + ",".join(f"s{i}" for i in range(19))),
            "truth": (
                truth,
                "wall_s,east_m,north_m,psi_north_zero_rad,u_mps,"
                "v_right_mps,r_radps,measured_east_m,measured_north_m,"
                "noise_east_m,noise_north_m",
            ),
            "odom": (
                odometry,
                "wall_s,east_m,north_m,psi_north_zero_rad,u_mps,v_right_mps,r_radps",
            ),
            "commands": (commands, "wall_s,speed_mps,yaw_rate_radps"),
            "diagnostics": (
                diagnostics, "wall_s," + ",".join(f"d{i}" for i in range(diagnostic_count)),
            ),
            "reference": (reference, "east_m,north_m"),
            "spline_starts": (
                spline_starts,
                "wall_s,start_east_m,start_north_m,odom_east_m,odom_north_m,"
                "start_gap_m,duration_s,segment_length_m,control_point_count,"
                "receive_interval_s,receive_rate_hz,"
                "local_path_signed_deviation_at_10m_m,local_path_deviation_p95_m,"
                "local_path_deviation_max_m,trajectory_start_stamp_s",
            ),
        }
        for name, (rows, header) in arrays.items():
            if rows:
                np.savetxt(OUTPUT / f"{name}.csv", rows, delimiter=",", header=header, comments="")
        write_json(OUTPUT / "run_status.json", status())

    try:
        for name, binary in executables.items():
            launch(name, [str(binary), "--ros-args", "--params-file", str(configs[name])])
        launch("reference", ["/usr/bin/python3", str(driver)])
        next_origin = next_progress = next_save = 0.0
        while elapsed() < MAX_DURATION_S:
            now = elapsed()
            if now >= next_origin:
                origin.header.stamp = node.get_clock().now().to_msg()
                origin_pub.publish(origin)
                next_origin = now + 0.5
            rclpy.spin_once(node, timeout_sec=0.01)
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"test process {process.pid} exited with {process.returncode}"
                    )
            if elapsed() >= next_progress:
                snapshot = status()
                write_json(OUTPUT / "live_status.json", snapshot)
                print(json.dumps(snapshot, ensure_ascii=False), flush=True)
                next_progress = elapsed() + 30.0
            if elapsed() >= next_save:
                save()
                next_save = elapsed() + 30.0
            if finished_at is not None and elapsed() >= finished_at + AFTER_FINISH_S:
                break
            if elapsed() > 20.0 and not diagnostics:
                raise RuntimeError("No NMPC diagnostics received within 20 seconds")
    except BaseException as exc:
        error = str(exc)
        raise
    finally:
        save()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    print(f"Saved route tracking data to {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
