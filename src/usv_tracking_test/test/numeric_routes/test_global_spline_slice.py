#!/usr/bin/env python3
"""验证滚动 B 样条切片与全局轨迹完全一致."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from route_shapes import (
    apply_lateral_replan,
    ComplexRoutePublisher,
    CompositeRouteModel,
    GLOBAL_ROUTE_ENDPOINT_TOLERANCE_M,
    MAX_YAW_RATE_RADPS,
    modulated_publish_period,
    random_publish_period,
    restrict_cubic_bspline,
)
from scipy.interpolate import BSpline, make_interp_spline


class SampledPolyline:
    def __init__(self, stations, points):
        self.stations = stations
        self.points = points

    def __call__(self, station):
        return np.array([
            np.interp(station, self.stations, self.points[:, axis])
            for axis in range(self.points.shape[1])
        ])


def make_projection_fixture(points, progress_m):
    points = np.asarray(points, dtype=float)
    station = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    publisher = SimpleNamespace(
        global_sample_xy=points,
        global_sample_times=station.copy(),
        global_station=station,
        global_spline=SampledPolyline(station, points),
        progress_m=progress_m,
    )
    publisher._time_at_station = lambda value: ComplexRoutePublisher._time_at_station(
        publisher, value)
    return publisher


@pytest.mark.parametrize("start, end", [(0.37, 2.61), (1.2, 4.73), (3.0, 5.0)])
def test_cubic_slice_matches_global_spline(start, end):
    times = np.linspace(0.0, 5.0, 51)
    phases = np.column_stack((
        3.0 * np.sin(0.7 * times),
        2.0 * np.cos(0.4 * times),
        0.3 * times + 0.08 * np.sin(times),
    ))
    global_spline = make_interp_spline(times, phases, k=3, axis=0)

    controls, knots = restrict_cubic_bspline(global_spline, start, end)
    local_spline = BSpline(knots, controls, 3)
    local_times = np.linspace(0.0, end - start, 301)

    np.testing.assert_allclose(
        local_spline(local_times), global_spline(start + local_times),
        rtol=0.0, atol=2e-10)
    np.testing.assert_allclose(
        local_spline.derivative()(local_times),
        global_spline.derivative()(start + local_times),
        rtol=0.0, atol=2e-9)


def test_cubic_slice_rejects_invalid_interval():
    times = np.linspace(0.0, 2.0, 9)
    spline = make_interp_spline(times, np.c_[times, times, times], k=3)

    with pytest.raises(ValueError):
        restrict_cubic_bspline(spline, 1.0, 1.0)


def test_rolling_spline_stamps_share_the_first_segment_time_origin():
    now = [Time(nanoseconds=100_000_000_000, clock_type=ClockType.ROS_TIME)]
    clock = SimpleNamespace(now=lambda: now[0])
    publisher = SimpleNamespace(global_spline_epoch=None, get_clock=lambda: clock)

    first = ComplexRoutePublisher._message_start_stamp(publisher, 2.0)
    now[0] = Time(nanoseconds=150_000_000_000, clock_type=ClockType.ROS_TIME)
    second = ComplexRoutePublisher._message_start_stamp(publisher, 3.0)

    assert first.sec == 100
    assert first.nanosec == 0
    assert second.sec == 101
    assert second.nanosec == 0


@pytest.mark.parametrize("publish_period_s", [1.0, 0.2])
def test_rolling_slices_keep_the_same_reference_phase_at_any_publish_rate(
    publish_period_s,
):
    times = np.linspace(0.0, 15.0, 151)
    values = np.column_stack((
        2.0 * np.sin(times / 5.0),
        times,
        0.1 * np.sin(times / 4.0),
    ))
    global_spline = make_interp_spline(times, values, k=3, axis=0)
    epoch_s = 100.0
    sample_period_s = 0.05

    publish_elapsed_s = 0.0
    while publish_elapsed_s < 8.0:
        segment_end_s = min(15.0, publish_elapsed_s + 5.0)
        controls, knots = restrict_cubic_bspline(
            global_spline, publish_elapsed_s, segment_end_s)
        local_spline = BSpline(knots, controls, 3)
        segment_start_stamp_s = epoch_s + publish_elapsed_s
        next_publish_elapsed_s = min(8.0, publish_elapsed_s + publish_period_s)
        query_elapsed = np.arange(
            publish_elapsed_s, next_publish_elapsed_s, sample_period_s)
        local_elapsed = (epoch_s + query_elapsed) - segment_start_stamp_s

        np.testing.assert_allclose(
            local_spline(local_elapsed), global_spline(query_elapsed),
            rtol=0.0, atol=2e-9)
        publish_elapsed_s = next_publish_elapsed_s


@pytest.mark.parametrize("frequency_hz", [1.0, 5.0])
def test_publish_rate_modulation_preserves_long_term_ten_hz_mean(frequency_hz):
    event_times = [0.0]
    while event_times[-1] < 60.0:
        event_times.append(
            event_times[-1] + modulated_publish_period(
                event_times[-1], 0.1, frequency_hz, 0.3
            )
        )

    periods = np.diff(event_times)
    mean_rate = (len(event_times) - 1) / (event_times[-1] - event_times[0])

    assert mean_rate == pytest.approx(10.0, abs=0.03)
    assert periods.min() >= 0.1 / 1.3 - 1e-6
    assert periods.max() <= 0.1 / 0.7 + 1e-6
    assert periods.min() < 0.09
    assert periods.max() > 0.11
    assert periods.max() - periods.min() > 0.02


def test_random_publish_period_is_bounded_and_reproducible():
    first_rng = np.random.default_rng(20261108)
    second_rng = np.random.default_rng(20261108)
    first = np.array([random_publish_period(first_rng) for _ in range(1000)])
    second = np.array([random_publish_period(second_rng) for _ in range(1000)])

    np.testing.assert_array_equal(first, second)
    assert np.min(first) >= 0.1
    assert np.max(first) <= 1.0
    assert np.min(first) < 0.11
    assert np.max(first) > 0.9


def test_random_publish_period_rejects_invalid_frequency_range():
    with pytest.raises(ValueError):
        random_publish_period(np.random.default_rng(1), 0.0, 10.0)


def test_random_local_replan_is_anchored_and_smoothly_reaches_offset():
    controls = np.zeros((7, 3))
    controls[:, 0] = np.linspace(0.0, 12.0, len(controls))
    controls[:, 2] = -math.pi / 2.0

    shifted = apply_lateral_replan(controls, 1.2, ramp_length_m=8.0)

    np.testing.assert_array_equal(shifted[0, :2], controls[0, :2])
    assert shifted[-1, 1] == pytest.approx(1.2)
    assert shifted[-1, 0] == pytest.approx(controls[-1, 0])
    assert np.all(np.isfinite(shifted))


def test_global_projection_does_not_backtrack_on_noisy_odom():
    publisher = make_projection_fixture([(x, 0.0) for x in range(11)], 5.0)

    projected_time, distance = ComplexRoutePublisher._project_to_global(
        publisher, np.array([4.6, 0.0]))

    assert publisher.progress_m == 5.0
    assert projected_time == 5.0
    assert distance == pytest.approx(0.4)


def test_global_projection_does_not_jump_to_a_nearby_future_branch():
    points = [(2.0, 0.0), (2.0, 1.0), (2.0, 2.0), (0.0, 2.0),
              (0.0, 1.0), (2.0, 1.0), (4.0, 1.0)]
    publisher = make_projection_fixture(points, 1.0)

    _, distance = ComplexRoutePublisher._project_to_global(
        publisher, np.array([2.01, 1.001]))

    assert publisher.progress_m < 2.5
    assert distance < 0.02


def test_global_projection_catches_up_after_skipped_publish_intervals():
    publisher = make_projection_fixture([(x, 0.0) for x in range(21)], 1.0)

    first_time, first_distance = ComplexRoutePublisher._project_to_global(
        publisher, np.array([5.0, 0.0]))
    assert publisher.progress_m == pytest.approx(3.0)
    assert first_time == pytest.approx(3.0)
    assert first_distance == pytest.approx(2.0)

    projected_time, distance = ComplexRoutePublisher._project_to_global(
        publisher, np.array([5.0, 0.0]))

    assert publisher.progress_m == pytest.approx(5.0)
    assert projected_time == pytest.approx(5.0)
    assert distance == pytest.approx(0.0)


def test_global_projection_stays_inside_window_between_samples():
    publisher = make_projection_fixture(
        [(0.0, 0.0), (1.3, 0.0), (2.6, 0.0), (3.9, 0.0), (5.2, 0.0)],
        0.7,
    )

    projected_time, distance = ComplexRoutePublisher._project_to_global(
        publisher, np.array([3.7, 0.0]))

    assert publisher.progress_m == pytest.approx(2.7)
    assert projected_time == pytest.approx(2.7)
    assert distance == pytest.approx(1.0)


def test_global_projection_does_not_advance_on_repeated_stale_odom():
    publisher = make_projection_fixture([(x, 0.0) for x in range(11)], 1.0)
    odom_xy = np.array([1.4, 0.0])

    first_time, _ = ComplexRoutePublisher._project_to_global(publisher, odom_xy)
    first_progress = publisher.progress_m
    second_time, _ = ComplexRoutePublisher._project_to_global(publisher, odom_xy)

    assert first_progress == pytest.approx(1.4)
    assert publisher.progress_m == pytest.approx(first_progress)
    assert second_time == pytest.approx(first_time)


def test_rejected_far_projection_never_pushes_global_progress_forward():
    publisher = make_projection_fixture([(x, 0.0) for x in range(21)], 1.0)
    far_odom_xy = np.array([20.0, 0.0])

    for _ in range(10):
        _, distance = ComplexRoutePublisher._project_to_global(
            publisher, far_odom_xy)
        assert distance > 2.0
        assert publisher.progress_m == pytest.approx(1.0)


def test_global_projection_clamps_to_route_endpoint():
    publisher = make_projection_fixture([(x, 0.0) for x in range(4)], 2.5)

    projected_time, distance = ComplexRoutePublisher._project_to_global(
        publisher, np.array([4.0, 0.0]))

    assert publisher.progress_m == pytest.approx(3.0)
    assert projected_time == pytest.approx(3.0)
    assert distance == pytest.approx(1.0)


@pytest.mark.parametrize("heading_deg", [0, 30, -60, -90, 180])
def test_model_generated_global_route_is_dynamically_consistent(heading_deg):
    route = CompositeRouteModel()
    initial_state = np.array([2.0, 2.0, math.radians(heading_deg), 0.0, 0.0, 0.0])

    spline, metrics = route.generate_global_spline(initial_state)
    endpoint = spline(spline.t[len(spline.c)])[:2]

    assert metrics["geometry_progress_m"] >= metrics["geometry_length_m"] - 0.2
    assert metrics["endpoint_error_m"] <= GLOBAL_ROUTE_ENDPOINT_TOLERANCE_M
    assert np.linalg.norm(endpoint - route.xy[-1]) <= GLOBAL_ROUTE_ENDPOINT_TOLERANCE_M
    assert metrics["max_abs_yaw_rate_radps"] <= MAX_YAW_RATE_RADPS
    assert metrics["dynamics_residual_p95"] < 0.002
    assert metrics["dynamics_residual_max"] < 0.01
