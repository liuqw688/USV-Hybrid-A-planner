"""Independent numerical checks for the USV grid design; does not build/run C++ or ROS.

Run: python3 tests/check_usv_grid_math.py
Requires the existing Python numpy/OpenCV installation.
"""
import math
import unittest
from pathlib import Path

import cv2
import numpy as np

cv2.setNumThreads(1)
ROOT = Path(__file__).resolve().parents[1]


def build_field(raw, resolution, inflation=0.0, unknown_blocked=True, cap=10.0):
    occupied = raw >= 75
    occupied[raw < 0] = unknown_blocked
    free = np.where(occupied, 0, 255).astype(np.uint8)
    initial = cv2.distanceTransform(np.pad(free, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    inflated = occupied | ((initial - 0.5) * resolution <= inflation)
    free = np.where(inflated, 0, 255).astype(np.uint8)
    positive = cv2.distanceTransform(np.pad(free, 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    field = np.maximum(0, (positive - 0.5) * resolution)
    if not np.any(free):
        field[:] = -math.hypot(raw.shape[1] * resolution, raw.shape[0] * resolution)
    else:
        negative = cv2.distanceTransform(255 - free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        field[inflated] = -np.maximum(0, (negative[inflated] - 0.5) * resolution)
    return inflated, np.clip(field, -cap, cap)


def sample_field(field, resolution, inflation, origin, yaw, position, cap=10.0):
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    local = rotation.T @ (np.asarray(position[:2]) - origin)
    height, width = np.array(field.shape) * resolution
    if local[0] < 0 or local[1] < 0 or local[0] >= width or local[1] >= height:
        delta = np.clip(local, [0, 0], [width, height]) - local
        length = np.linalg.norm(delta)
        if length:
            gradient = rotation @ delta / length
        else:
            direction = np.array([-float(local[0] >= width), -float(local[1] >= height)])
            gradient = rotation @ direction / np.linalg.norm(direction)
        value = length + inflation
        if value >= cap:
            gradient[:] = 0
        return -min(value, cap), gradient
    grid = local / resolution - 0.5
    clipped = np.clip(grid, [0, 0], [field.shape[1] - 1, field.shape[0] - 1])
    x, y = np.floor(clipped).astype(int)
    x1, y1 = min(x + 1, field.shape[1] - 1), min(y + 1, field.shape[0] - 1)
    tx, ty = clipped - [x, y]
    d00, d10, d01, d11 = map(float, [field[y, x], field[y, x1], field[y1, x], field[y1, x1]])
    value = (1 - ty) * ((1 - tx) * d00 + tx * d10) + ty * ((1 - tx) * d01 + tx * d11)
    gradient = np.array([((1 - ty) * (d10 - d00) + ty * (d11 - d01)) / resolution,
                         ((1 - tx) * (d01 - d00) + tx * (d11 - d10)) / resolution])
    gradient[grid != clipped] = 0
    edges = [local[0], width - local[0], local[1], height - local[1]]
    index = int(np.argmin(edges))
    edge = edges[index] - inflation
    if edge < value:
        value = edge
        gradient = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]][index], dtype=float)
    return value, rotation @ gradient


class GridChecks(unittest.TestCase):
    def test_distance_against_exhaustive_nearest_cell(self):
        raw = np.zeros((7, 9), dtype=np.int8)
        raw[3, 4] = 100
        raw[1, 6] = -1
        blocked, field = build_field(raw, 0.25)
        # Independent exhaustive Euclidean nearest-seed oracle, including the blocked exterior ring.
        padded = np.pad(blocked, 1, constant_values=True)
        obstacle_seeds = np.argwhere(padded)
        free_seeds = np.argwhere(~blocked)
        for y in range(7):
            for x in range(9):
                if blocked[y, x]:
                    distance = np.min(np.linalg.norm(free_seeds - [y, x], axis=1))
                    expected = -max(0, (distance - 0.5) * 0.25)
                else:
                    distance = np.min(np.linalg.norm(obstacle_seeds - [y + 1, x + 1], axis=1))
                    expected = max(0, (distance - 0.5) * 0.25)
                self.assertAlmostEqual(float(field[y, x]), expected, places=6)

    def test_unknown_threshold_and_inflation(self):
        raw = np.zeros((15, 15), dtype=np.int8)
        raw[7, 7] = 100
        raw[4, 4] = -1
        raw[4, 10] = 74
        raw[10, 4] = 75
        blocked, field = build_field(raw, 0.25, 0.25)
        self.assertTrue(blocked[4, 4])
        self.assertFalse(blocked[4, 10])
        self.assertTrue(blocked[10, 4])
        self.assertTrue(blocked[7, 8])
        self.assertFalse(blocked[7, 9])
        self.assertLess(field[7, 7], 0)
        allowed, _ = build_field(raw, 0.25, unknown_blocked=False)
        self.assertFalse(allowed[4, 4])

    def test_subthreshold_values_do_not_seed_obstacles(self):
        raw = np.zeros((41, 41), dtype=np.int8)
        raw[20, 20] = 100
        raw[10, 10] = 1
        raw[10, 20] = 60
        raw[10, 30] = 74
        raw[30, 20] = 75
        reference = raw.copy()
        reference[(reference > 0) & (reference < 75)] = 0
        blocked, field = build_field(raw, 0.25)
        reference_blocked, reference_field = build_field(reference, 0.25)
        np.testing.assert_array_equal(blocked, reference_blocked)
        np.testing.assert_allclose(field, reference_field)
        self.assertFalse(blocked[10, 30])
        self.assertTrue(blocked[30, 20])

    def test_fixed_frame_timer_threshold_contracts(self):
        grid = (ROOT / 'fast_planner/plan_env/src/sdf_map_grid.cpp').read_text()
        fsm = (ROOT / 'fast_planner/plan_manage/src/kino_replan_fsm.cpp').read_text()
        server = (ROOT / 'fast_planner/plan_manage/src/traj_server.cpp').read_text()
        schema = (ROOT / 'fast_planner/plan_manage/msg/Bspline.msg').read_text()
        self.assertIn('std::chrono::milliseconds(100)', grid)
        self.assertIn('if (msg->header.frame_id != mp_.frame_id_)', grid)
        self.assertIn('"sdf_map/frame_id", "map"', grid)
        self.assertIn('"sdf_map/grid_occupied_threshold", 75', grid)
        self.assertNotIn('string frame_id', schema)
        self.assertNotIn('bspline.frame_id', fsm)
        self.assertNotIn('msg->frame_id', server)

    def test_three_meter_inflation(self):
        raw = np.zeros((81, 101), dtype=np.int8)
        raw[40, 50] = 100
        blocked, _ = build_field(raw, 0.25, 3.0)
        self.assertTrue(blocked[40, 62])   # 3m center separation, half-cell convention.
        self.assertFalse(blocked[40, 63])  # 3.25m center separation.
        self.assertTrue(blocked[40, 0])    # Map boundary also inflated.

    def test_empty_full_and_small_maps(self):
        for raw in [np.zeros((9, 11), dtype=np.int8), np.full((9, 11), 100, dtype=np.int8),
                    np.full((9, 11), -1, dtype=np.int8), np.zeros((1, 1), dtype=np.int8)]:
            blocked, field = build_field(raw, 0.1)
            self.assertTrue(np.isfinite(field).all())
            self.assertTrue(np.all(field[blocked] < 0))
            self.assertTrue(np.all(field[~blocked] > 0))

    def test_rotated_origin_and_row_major_index(self):
        origin = np.array([-250.0, 32.5])
        yaw = 0.73
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        for x, y in [(0, 0), (7, 3), (19, 12)]:
            world = origin + rotation @ (np.array([x + 0.5, y + 0.5]) * 0.25)
            local = rotation.T @ (world - origin)
            np.testing.assert_array_equal(np.floor(local / 0.25).astype(int), [x, y])
            self.assertEqual(np.arange(13 * 20).reshape(13, 20)[y, x], y * 20 + x)

    def test_gradient_matches_finite_difference_in_world_frame(self):
        raw = np.zeros((35, 41), dtype=np.int8)
        raw[12:18, 15:22] = 100
        _, field = build_field(raw, 0.25, 0.1)
        origin, yaw = np.array([-10.0, 30.0]), 0.73
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        # Free space, obstacle interior, boundary strip and exterior; avoid nonsmooth ties.
        for local in [[3.07, 2.18], [4.07, 3.66], [0.07, 1.38], [-0.3, 1.38], [11.0, 9.0]]:
            world = origin + rotation @ local
            _, gradient = sample_field(field, 0.25, 0.1, origin, yaw, world)
            finite_difference = []
            for axis in range(2):
                delta = np.zeros(2)
                delta[axis] = 1e-5
                hi = sample_field(field, 0.25, 0.1, origin, yaw, world + delta)[0]
                lo = sample_field(field, 0.25, 0.1, origin, yaw, world - delta)[0]
                finite_difference.append((hi - lo) / 2e-5)
            np.testing.assert_allclose(gradient, finite_difference, atol=2e-6)

    def test_saturated_distance_has_zero_far_gradient(self):
        _, field = build_field(np.zeros((201, 201), dtype=np.int8), 0.25, 3)
        for position, expected in [([25, 25], 10), ([-30, 25], -10)]:
            value, gradient = sample_field(field, 0.25, 3, np.zeros(2), 0, position)
            self.assertEqual(value, expected)
            np.testing.assert_array_equal(gradient, 0)

    def test_heading_independence_and_exterior(self):
        _, field = build_field(np.zeros((20, 20), dtype=np.int8), 0.25)
        for heading in [-100, 0, math.pi, 100]:
            value, gradient = sample_field(field, 0.25, 0, np.zeros(2), 0, [2.1, 2.2, heading])
            reference = sample_field(field, 0.25, 0, np.zeros(2), 0, [2.1, 2.2, 0])
            self.assertEqual(value, reference[0])
            np.testing.assert_array_equal(gradient, reference[1])
        for point in [[-1, 2], [5, 2], [5, 5]]:
            value, gradient = sample_field(field, 0.25, 0, np.zeros(2), 0, point)
            self.assertLessEqual(value, 0)
            self.assertTrue(np.isfinite(gradient).all())

    def test_500m_map_with_025m_cells(self):
        raw = np.zeros((2000, 2000), dtype=np.int8)
        raw[:, 1000] = 100
        blocked, field = build_field(raw, 0.25, 0.099)
        self.assertEqual(field.shape, (2000, 2000))
        self.assertLess(field[1000, 1000], 0)
        self.assertGreater(field[1000, 900], 0)
        self.assertLessEqual(raw.nbytes + blocked.nbytes + field.nbytes, 24_000_000)

    def test_source_integration_contracts(self):
        header = (ROOT / 'fast_planner/plan_env/include/plan_env/sdf_map.h').read_text()
        source = (ROOT / 'fast_planner/plan_env/src/sdf_map.cpp').read_text()
        grid = (ROOT / 'fast_planner/plan_env/src/sdf_map_grid.cpp').read_text()
        environment = (ROOT / 'fast_planner/plan_env/src/edt_environment.cpp').read_text()
        self.assertLess(source.index('initGridMap();'), source.index('int buffer_size ='))
        self.assertIn('cv::DIST_MASK_PRECISE', grid)
        self.assertIn('.reliable().transient_local()', grid)
        self.assertIn('grid_map_ready_ = true;', grid)
        self.assertIn('gridDistanceWithGrad(pos, grad)', header)
        self.assertIn('if (sdf_map_->usesGridMap())', environment)
        self.assertIn('time < 0.0 || sdf_map_->usesGridMap()', environment)


if __name__ == '__main__':
    unittest.main(verbosity=2)
