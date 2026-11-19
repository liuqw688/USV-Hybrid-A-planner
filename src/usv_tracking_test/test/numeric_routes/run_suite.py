#!/usr/bin/env python3
"""在隔离的 ROS Domain 中运行路线与初始航向组合测试."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from route_shapes import COMPOSITE_ROUTE, route_info, ROUTE_SPEED_MPS

COMPOSITE_HEADINGS_DEG = (0, 30, -60, -90, 180)


def write_json(path, value):
    """
    先写临时文件再替换目标，避免状态读取方看到未完成的 JSON.

    :param path: 目标文件路径。
    :param value: 可由 JSON 序列化的数据。
    """
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2))
    temporary.replace(path)


def main():
    """按命令行配置调度路线测试进程，并持续保存套件状态."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--parallel", type=int, default=3)
    parser.add_argument("--routes", nargs="+", choices=[COMPOSITE_ROUTE],
                        default=[COMPOSITE_ROUTE])
    parser.add_argument("--angles", nargs="+", type=int,
                        default=list(COMPOSITE_HEADINGS_DEG),
                        help="初始状态 phi，单位度；0 指北、-90 指东")
    parser.add_argument("--start-offset-x-m", type=float, default=2.0)
    parser.add_argument("--start-offset-y-m", type=float, default=2.0)
    parser.add_argument(
        "--max-duration-s", type=float, default=None,
        help="可选仿真时限；未设置时按路线长度自动设置")
    parser.add_argument(
        "--position-noise-stddev-m", type=float, default=None,
        help="可选覆盖仿真定位噪声，用于区分噪声与跟踪误差")
    parser.add_argument(
        "--weight-lateral", type=float, default=None,
        help="可选覆盖 NMPC 横向误差权重，用于参数试验")
    parser.add_argument(
        "--odom-position-alpha", type=float, default=None,
        help="可选覆盖 /odom 位置预测器测量校正系数，范围 (0, 1]")
    parser.add_argument(
        "--noise-seed", type=int, default=None,
        help="可选固定仿真噪声种子，便于精确比较参数变化")
    parser.add_argument(
        "--publish-jitter-hz", type=int, nargs="+", choices=(0, 1, 5), default=(0,),
        help="局部 B 样条发布周期抖动频率；0 表示稳定 10 Hz")
    parser.add_argument(
        "--random-publish-rate", action="store_true",
        help="每条局部样条发布前随机抽取 1～10 Hz 频率")
    parser.add_argument(
        "--publish-seed", type=int, default=20261108,
        help="随机发布频率的固定种子")
    parser.add_argument(
        "--publish-interval-jitter-fraction", type=float, default=0.3,
        help="发布间隔的正弦调制比例，范围 [0, 1)")
    parser.add_argument(
        "--local-deviation-stddev-m", type=float, default=0.0,
        help="每次滚动局部路径的随机侧向改道标准差；0 表示不改道")
    parser.add_argument(
        "--local-deviation-seed", type=int, default=20261107,
        help="局部随机改道种子")
    args = parser.parse_args()
    if args.position_noise_stddev_m is not None and args.position_noise_stddev_m < 0.0:
        parser.error("--position-noise-stddev-m 不能为负数")
    if args.weight_lateral is not None and args.weight_lateral <= 0.0:
        parser.error("--weight-lateral 必须为正数")
    if args.odom_position_alpha is not None and not 0.0 < args.odom_position_alpha <= 1.0:
        parser.error("--odom-position-alpha 必须在 (0, 1] 内")
    if args.noise_seed is not None and not 0 <= args.noise_seed <= 4294967295:
        parser.error("--noise-seed 必须在 [0, 4294967295] 内")
    if not 0 <= args.publish_seed <= 4294967295:
        parser.error("--publish-seed 必须在 [0, 4294967295] 内")
    if args.random_publish_rate and any(args.publish_jitter_hz):
        parser.error("--random-publish-rate 不能与正弦发布调制同时使用")
    if not 0.0 <= args.publish_interval_jitter_fraction < 1.0:
        parser.error("--publish-interval-jitter-fraction 必须在 [0, 1) 内")
    if args.local_deviation_stddev_m < 0.0:
        parser.error("--local-deviation-stddev-m 不能为负数")
    if not 0 <= args.local_deviation_seed <= 4294967295:
        parser.error("--local-deviation-seed 必须在 [0, 4294967295] 内")
    output = args.output.resolve()
    (output / "runs").mkdir(parents=True, exist_ok=True)
    start_gap_m = math.hypot(args.start_offset_x_m, args.start_offset_y_m)
    initial_heading_deg = -args.angles[0]
    geometries = {
        route: route_info(
            route, args.start_offset_x_m, args.start_offset_y_m, initial_heading_deg)
        for route in args.routes
    }
    write_json(output / "route_geometry.json", geometries)
    write_json(
        output / "suite_config.json",
        {
            "routes": args.routes,
            "initial_state_phi_deg": args.angles,
            "model_compass_heading_deg": [-angle for angle in args.angles],
            "start_offset_x_m": args.start_offset_x_m,
            "start_offset_y_m": args.start_offset_y_m,
            "start_gap_m": start_gap_m,
            "max_duration_s": args.max_duration_s,
            "position_noise_stddev_m": args.position_noise_stddev_m,
            "weight_lateral": args.weight_lateral,
            "odom_position_alpha": args.odom_position_alpha,
            "noise_seed": args.noise_seed,
            "publish_jitter_hz": args.publish_jitter_hz,
            "publish_interval_jitter_fraction": args.publish_interval_jitter_fraction,
            "publish_rate_mode": "random" if args.random_publish_rate else None,
            "publish_rate_min_hz": 1.0,
            "publish_rate_max_hz": 10.0,
            "publish_seed": args.publish_seed,
            "local_deviation_stddev_m": args.local_deviation_stddev_m,
            "local_deviation_seed": args.local_deviation_seed,
            "nominal_speed_mps_by_route": {route: ROUTE_SPEED_MPS[route] for route in args.routes},
            "parallel": args.parallel,
        },
    )
    cases = []
    for route in args.routes:
        for angle in args.angles:
            for jitter_hz in args.publish_jitter_hz:
                rate_mode = (
                    "random" if args.random_publish_rate
                    else "sinusoidal" if jitter_hz > 0
                    else "fixed")
                name = (
                    f"{route}_{'m' if angle < 0 else 'p'}{abs(angle):03d}_"
                    f"{'random1to10hz' if rate_mode == 'random' else f'j{jitter_hz:02d}hz'}"
                )
                initial_heading = -angle
                folder = output / "runs" / name
                status = folder / "run_status.json"
                cases.append(
                    {
                        "name": name,
                        "route": route,
                        "relative_heading_deg": angle,
                        "initial_heading_deg": initial_heading,
                        "initial_phi_deg": angle,
                        "start_offset_x_m": args.start_offset_x_m,
                        "start_offset_y_m": args.start_offset_y_m,
                        "start_gap_m": start_gap_m,
                        "nominal_speed_mps": ROUTE_SPEED_MPS[route],
                        "publish_jitter_hz": jitter_hz,
                        "publish_rate_mode": rate_mode,
                        "publish_rate_min_hz": 1.0,
                        "publish_rate_max_hz": 10.0,
                        "publish_seed": args.publish_seed,
                        "publish_interval_jitter_fraction": (
                            args.publish_interval_jitter_fraction),
                        "local_deviation_stddev_m": args.local_deviation_stddev_m,
                        "local_deviation_seed": args.local_deviation_seed,
                        "ros_domain_id": 190 + len(cases),
                        "run_dir": str(folder),
                        "state": "completed" if status.exists() else "pending",
                    }
                )
    started = time.time()
    active = {}

    def snapshot():
        write_json(
            output / "suite_status.json",
            {
                "started_unix": started,
                "elapsed_s": time.time() - started,
                "completed": sum(c["state"] == "completed" for c in cases),
                "parallel": args.parallel,
                "cases": cases,
            },
        )

    try:
        while any(c["state"] != "completed" for c in cases):
            for index, case in enumerate(cases):
                if len(active) >= args.parallel:
                    break
                if case["state"] != "pending":
                    continue
                folder = Path(case["run_dir"])
                folder.mkdir(exist_ok=True)
                env = dict(
                    os.environ,
                    OPENBLAS_NUM_THREADS="1",
                    OMP_NUM_THREADS="1",
                    USV_NMPC_TRACKING_OUTPUT=str(folder),
                    USV_NMPC_ROUTE_MODE="straight",
                    USV_COMPLEX_ROUTE=case["route"],
                    USV_NMPC_ROUTE_START_OFFSET_M=str(case["start_gap_m"]),
                    USV_NMPC_ROUTE_START_OFFSET_X_M=str(case["start_offset_x_m"]),
                    USV_NMPC_ROUTE_START_OFFSET_Y_M=str(case["start_offset_y_m"]),
                    USV_NMPC_INITIAL_EAST_M=str(case["start_offset_x_m"]),
                    USV_NMPC_INITIAL_NORTH_M=str(case["start_offset_y_m"]),
                    USV_NMPC_ROUTE_SPEED_MPS=str(case["nominal_speed_mps"]),
                    USV_NMPC_RELATIVE_HEADING_DEG=str(case["relative_heading_deg"]),
                    USV_NMPC_INITIAL_PHI_DEG=str(case["initial_phi_deg"]),
                    USV_NMPC_INITIAL_HEADING_DEG=str(case["initial_heading_deg"]),
                    USV_NMPC_PUBLISH_JITTER_HZ=str(case["publish_jitter_hz"]),
                    USV_NMPC_PUBLISH_RATE_MODE=case["publish_rate_mode"],
                    USV_NMPC_PUBLISH_RATE_MIN_HZ=str(case["publish_rate_min_hz"]),
                    USV_NMPC_PUBLISH_RATE_MAX_HZ=str(case["publish_rate_max_hz"]),
                    USV_NMPC_PUBLISH_SEED=str(case["publish_seed"]),
                    USV_NMPC_PUBLISH_INTERVAL_JITTER_FRACTION=str(
                        case["publish_interval_jitter_fraction"]),
                    USV_NMPC_LOCAL_DEVIATION_STDDEV_M=str(
                        case["local_deviation_stddev_m"]),
                    USV_NMPC_LOCAL_DEVIATION_SEED=str(case["local_deviation_seed"]),
                    USV_NMPC_MAX_DURATION_S=str(
                        args.max_duration_s if args.max_duration_s is not None
                        else geometries[case["route"]]["duration_s"] + 240),
                    USV_NMPC_AFTER_FINISH_S="10",
                    USV_NMPC_ROS_DOMAIN_ID=str(case["ros_domain_id"]),
                )
                if args.position_noise_stddev_m is not None:
                    env["USV_NMPC_MODEL_POSITION_NOISE_STDDEV_M"] = str(
                        args.position_noise_stddev_m)
                if args.weight_lateral is not None:
                    env["USV_NMPC_WEIGHT_LATERAL"] = str(args.weight_lateral)
                if args.odom_position_alpha is not None:
                    env["USV_NMPC_ODOM_POSITION_ALPHA"] = str(args.odom_position_alpha)
                if args.noise_seed is not None:
                    env["USV_NMPC_NOISE_SEED"] = str(args.noise_seed)
                log = open(folder / "runner.log", "w")
                process = subprocess.Popen(
                    ["/usr/bin/python3", "-u", str(Path(__file__).with_name("run_case.py"))],
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
                case.update(state="running", started_unix=time.time(), runner_pid=process.pid)
                active[index] = process, log
                print(f"Start {case['name']}, domain {case['ros_domain_id']}", flush=True)
            for index, (process, log) in list(active.items()):
                code = process.poll()
                if code is None:
                    continue
                case = cases[index]
                log.close()
                status_path = Path(case["run_dir"]) / "run_status.json"
                result = json.loads(status_path.read_text()) if status_path.exists() else {}
                case.update(
                    state="completed",
                    returncode=code,
                    ended_unix=time.time(),
                    finished_at_s=result.get("finished_at_s"),
                    error=result.get("error"),
                )
                del active[index]
                print(
                    f"End {case['name']}: finish={case['finished_at_s']}, exit={code}", flush=True
                )
            snapshot()
            time.sleep(1)
    finally:
        for process, _ in active.values():
            process.send_signal(signal.SIGINT)
        for process, log in active.values():
            process.wait(timeout=20)
            log.close()
        snapshot()
    print(f"Completed {len(cases)} cases in {(time.time() - started) / 60:.1f} min", flush=True)


if __name__ == "__main__":
    main()
