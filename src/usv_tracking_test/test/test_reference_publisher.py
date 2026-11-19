"""验证测试样条的节点结构、末端停车及完整可视化采样."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace

from nav_msgs.msg import Odometry
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from publish_bspline_reference import BsplineReferencePublisher  # noqa: E402
import pytest  # noqa: E402
from rclpy.time import Time  # noqa: E402
from scipy.interpolate import BSpline  # noqa: E402


def make_publisher(route_mode, heading_deg):
    """构造无需 ROS 通信的默认测试配置，并保留实际发布的路径."""
    defaults = BsplineReferencePublisher
    paths = []
    publisher = SimpleNamespace(
        speed=defaults.SPEED_MPS,
        interval=defaults.CONTROL_INTERVAL_S,
        route_mode=route_mode,
        lane_length=defaults.PATH_LENGTH_M,
        lane_count=defaults.LANE_COUNT,
        lane_spacing=defaults.LANE_SPACING_M,
        turn_bulge=defaults.TURN_BULGE_M,
        curve_amplitude=defaults.CURVE_AMPLITUDE_M,
        curve_half_wave=defaults.CURVE_HALF_WAVE_M,
        start_offset=0.0,
        start_delay=defaults.START_DELAY_S,
        traj_id=1,
        frame_id=defaults.FRAME_ID,
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        path_publisher=SimpleNamespace(publish=paths.append),
        _deboor=defaults._deboor,
    )
    odom = Odometry()
    odom.header.frame_id = defaults.FRAME_ID
    odom.pose.pose.position.x = 100.0
    odom.pose.pose.position.y = -25.0
    phi = math.radians(heading_deg)
    odom.pose.pose.orientation.z = math.sin(phi / 2.0)
    odom.pose.pose.orientation.w = math.cos(phi / 2.0)
    publisher.trajectory = defaults._make_trajectory(publisher, odom)
    return publisher, paths


@pytest.mark.parametrize("route_mode", [
    "straight", "reverse_straight", "arc_left", "arc_right", "s_curve", "mowing",
])
@pytest.mark.parametrize("heading_deg", [0.0, 30.0, -90.0])
def test_generated_spline_matches_independent_evaluation(
    monkeypatch, route_mode, heading_deg,
):
    """含追加停车控制点的样条应在整个有效时间内与 SciPy 求值一致."""
    monkeypatch.delenv("USV_NMPC_REFERENCE_HEADING_DEG", raising=False)
    publisher, _ = make_publisher(route_mode, heading_deg)
    message = publisher.trajectory
    controls = np.array([(p.x, p.y, p.phi) for p in message.control_points])
    assert len(message.knots) == len(controls) + message.order + 1
    reference = BSpline(message.knots, controls, message.order)
    times = np.linspace(message.knots[message.order], message.knots[len(controls)], 101)
    actual = np.array([
        publisher._deboor(controls, message.knots, message.order, t) for t in times
    ])
    np.testing.assert_allclose(actual, reference(times), rtol=0.0, atol=1e-10)
    np.testing.assert_allclose(actual[0, :2], [100.0, -25.0], atol=1e-10)
    np.testing.assert_allclose(reference.derivative()(times[-1]), 0.0, atol=1e-10)
    np.testing.assert_allclose(reference.derivative(2)(times[-1]), 0.0, atol=1e-10)


def test_default_mowing_path_includes_endpoint_without_index_error(monkeypatch):
    """默认割草路线的完整 Path 采样包含终点，且不会访问节点向量之外."""
    monkeypatch.delenv("USV_NMPC_REFERENCE_HEADING_DEG", raising=False)
    publisher, paths = make_publisher("mowing", 0.0)
    BsplineReferencePublisher._publish_reference_path(publisher)
    assert len(paths) == 1
    path = paths[0]
    assert path.header.frame_id == "odom"
    assert len(path.poses) > 100
    message = publisher.trajectory
    controls = np.array([(p.x, p.y, p.phi) for p in message.control_points])
    reference = BSpline(message.knots, controls, message.order)
    start = message.knots[message.order]
    end = message.knots[len(controls)]
    times = np.linspace(start, end, len(path.poses))
    actual = np.array([(p.pose.position.x, p.pose.position.y) for p in path.poses])
    np.testing.assert_allclose(actual, reference(times)[:, :2], rtol=0.0, atol=1e-10)
