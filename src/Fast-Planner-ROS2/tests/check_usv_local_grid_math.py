"""Numerical/source checks for local-grid fusion, without building/running C++ or ROS."""
import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from check_usv_grid_math import build_field

ROOT = Path(__file__).resolve().parents[1]
cv2.setNumThreads(1)


def partial_update(raw, previous_inflated, previous_distance, core, resolution, inflation=3.0, cap=10.0, mask=None):
    """ROI model checked below against independently recomputed full-map results."""
    x, y, width, height = core
    halo = math.ceil((cap + inflation) / resolution) + 2
    x0, y0 = max(0, x - halo), max(0, y - halo)
    x1, y1 = min(raw.shape[1], x + width + halo), min(raw.shape[0], y + height + halo)
    values = raw[y0:y1, x0:x1]
    occupied = (values < 0) | (values >= 75)
    free = np.where(occupied, 0, 255).astype(np.uint8)

    def padding(image):
        padded = np.pad(image, 1, constant_values=255)
        if x0 == 0:
            padded[:, 0] = 0
        if y0 == 0:
            padded[0, :] = 0
        if x1 == raw.shape[1]:
            padded[:, -1] = 0
        if y1 == raw.shape[0]:
            padded[-1, :] = 0
        return padded

    distance = cv2.distanceTransform(padding(free), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    inflated = occupied | ((distance - 0.5) * resolution <= inflation)
    free = np.where(inflated, 0, 255).astype(np.uint8)
    padded = padding(free)
    positive = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    negative = cv2.distanceTransform(255 - padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    signed = np.minimum(cap, np.maximum(0, (positive - 0.5) * resolution))
    signed[inflated] = -np.minimum(cap, np.maximum(0, (negative[inflated] - 0.5) * resolution))
    out_i, out_d = previous_inflated.copy(), previous_distance.copy()
    crop = (slice(y - y0, y + height - y0), slice(x - x0, x + width - x0))
    if mask is None:
        mask = np.ones((height, width), dtype=bool)
    target_i, target_d = out_i[y:y + height, x:x + width], out_d[y:y + height, x:x + width]
    target_i[mask] = inflated[crop][mask]
    target_d[mask] = signed[crop][mask]
    return out_i, out_d


def overlap_area(corners, x, y):
    polygon = [np.array(p, dtype=float) for p in corners]
    for axis, lower, bound in [(0, True, x), (0, False, x + 1), (1, True, y), (1, False, y + 1)]:
        clipped = []
        for i, b in enumerate(polygon):
            a = polygon[i - 1]
            ai = a[axis] >= bound if lower else a[axis] <= bound
            bi = b[axis] >= bound if lower else b[axis] <= bound
            if ai != bi:
                clipped.append(a + (bound - a[axis]) / (b[axis] - a[axis]) * (b - a))
            if bi:
                clipped.append(b)
        polygon = clipped
        if not polygon:
            return 0.0
    return abs(sum(np.cross(a, polygon[(i + 1) % len(polygon)]) for i, a in enumerate(polygon))) * 0.5


def fuse_cells(base, layer, local, local_resolution, local_origin, local_yaw, global_resolution=1.0,
               global_origin=(0, 0), global_yaw=0, active=None):
    """Conservative polygon-overlap model for differing grid origins/resolutions."""
    gc, gs = math.cos(global_yaw), math.sin(global_yaw)
    lc, ls = math.cos(local_yaw), math.sin(local_yaw)
    global_rotation = np.array([[gc, -gs], [gs, gc]])
    local_rotation = np.array([[lc, -ls], [ls, lc]])
    occupied = np.full(base.shape, -1, dtype=np.int16)
    free_area = np.zeros(base.shape)
    if active is None:
        active = np.ones(base.shape, dtype=bool)
    for ly, lx in np.ndindex(local.shape):
        value = int(local[ly, lx])
        if value < 0:
            continue
        corners = []
        for dx, dy in [(0, 0), (1, 0), (1, 1), (0, 1)]:
            world = np.array(local_origin) + local_rotation @ (np.array([lx + dx, ly + dy]) * local_resolution)
            corners.append(global_rotation.T @ (world - global_origin) / global_resolution)
        corners = np.array(corners)
        minimum, maximum = np.floor(corners.min(axis=0)).astype(int), np.ceil(corners.max(axis=0)).astype(int)
        for gy in range(max(0, minimum[1]), min(base.shape[0], maximum[1])):
            for gx in range(max(0, minimum[0]), min(base.shape[1], maximum[0])):
                if not active[gy, gx]:
                    continue
                area = overlap_area(corners, gx, gy)
                if area <= 1e-8:
                    continue
                if value >= 75:
                    occupied[gy, gx] = max(occupied[gy, gx], value)
                else:
                    free_area[gy, gx] += area
    layer = layer.copy()
    for y, x in np.ndindex(base.shape):
        if not active[y, x]:
            continue
        if occupied[y, x] >= 75:
            layer[y, x] = occupied[y, x]
        elif free_area[y, x] >= 1 - 1e-5:
            layer[y, x] = 0
    fused = np.where(base >= 75, base, np.where(layer >= 0, layer, base))
    return layer, fused


class LocalGridChecks(unittest.TestCase):
    def test_local_esdf_matches_full_with_addition_and_removal(self):
        rng = np.random.default_rng(42)
        raw = np.zeros((281, 301), dtype=np.int8)
        for y, x in rng.integers([10, 10], [270, 290], size=(30, 2)):
            raw[y, x] = 100
        old_i, old_d = build_field(raw, 0.5, 3.0)
        raw[125:131, 130:136] = 100
        raw[170:175, 181:189] = -1
        core = (110, 100, 100, 100)
        for clearing in [False, True]:
            if clearing:
                raw[125:131, 130:136] = 0
            new_i, new_d = partial_update(raw, old_i, old_d, core, 0.5)
            full_i, full_d = build_field(raw, 0.5, 3.0)
            x, y, w, h = core
            np.testing.assert_array_equal(new_i[y:y+h, x:x+w], full_i[y:y+h, x:x+w])
            np.testing.assert_allclose(new_d[y:y+h, x:x+w], full_d[y:y+h, x:x+w], atol=1e-5)
            exterior = np.ones(raw.shape, dtype=bool)
            exterior[y:y+h, x:x+w] = False
            np.testing.assert_array_equal(new_i[exterior], old_i[exterior])
            np.testing.assert_array_equal(new_d[exterior], old_d[exterior])
            old_i, old_d = new_i, new_d

    def test_outside_window_obstacle_influences_inside(self):
        raw = np.zeros((260, 260), dtype=np.int8)
        old_i, old_d = build_field(raw, 0.5, 3)
        raw[130, 96] = 100  # just outside core x=100, still affects core after inflation
        updated_i, updated_d = partial_update(raw, old_i, old_d, (100, 100, 60, 60), 0.5)
        full_i, full_d = build_field(raw, 0.5, 3)
        np.testing.assert_array_equal(updated_i[100:160, 100:160], full_i[100:160, 100:160])
        np.testing.assert_allclose(updated_d[100:160, 100:160], full_d[100:160, 100:160], atol=1e-5)
        self.assertTrue(updated_i[130, 100])

    def test_no_artificial_window_wall_and_true_global_edge(self):
        raw = np.zeros((1000, 1000), dtype=np.int8)
        old_i, old_d = build_field(raw, 0.25, 3)
        updated_i, updated_d = partial_update(raw, old_i, old_d, (300, 300, 400, 400), 0.25)
        self.assertFalse(updated_i[300:700, 300:700].any())
        np.testing.assert_array_equal(updated_d[300:700, 300:700], 10)
        # Same 100m square clipped to a real global boundary must preserve the edge wall.
        updated_i, updated_d = partial_update(raw, old_i, old_d, (0, 100, 400, 400), 0.25)
        np.testing.assert_array_equal(updated_i[100:500, :400], old_i[100:500, :400])
        np.testing.assert_allclose(updated_d[100:500, :400], old_d[100:500, :400], atol=1e-5)

    def test_rotated_commit_mask_preserves_outside(self):
        raw = np.zeros((130, 130), dtype=np.int8)
        old_i, old_d = build_field(raw, 0.5, 3)
        raw[65, 65] = 100
        yy, xx = np.mgrid[:80, :80]
        mask = abs(xx - 39.5) + abs(yy - 39.5) < 40
        new_i, new_d = partial_update(raw, old_i, old_d, (25, 25, 80, 80), 0.5, mask=mask)
        np.testing.assert_array_equal(new_i[25:105, 25:105][~mask], old_i[25:105, 25:105][~mask])
        np.testing.assert_array_equal(new_d[25:105, 25:105][~mask], old_d[25:105, 25:105][~mask])

    def test_static_preserved_dynamic_cleared_unknown_retained(self):
        base = np.array([[100, 0, -1], [0, 0, 0]], dtype=np.int8)
        layer = np.full(base.shape, -1, dtype=np.int8)
        local = np.array([[0, 100, 0], [0, 0, 0]], dtype=np.int8)
        layer, fused = fuse_cells(base, layer, local, 1, (0, 0), 0)
        self.assertEqual(fused[0, 0], 100)
        self.assertEqual(fused[0, 1], 100)
        self.assertEqual(fused[0, 2], 0)
        unknown = np.full(base.shape, -1, dtype=np.int8)
        next_layer, next_fused = fuse_cells(base, layer, unknown, 1, (0, 0), 0)
        np.testing.assert_array_equal(next_layer, layer)
        np.testing.assert_array_equal(next_fused, fused)
        free = np.zeros(base.shape, dtype=np.int8)
        _, fused = fuse_cells(base, layer, free, 1, (0, 0), 0)
        self.assertEqual(fused[0, 1], 0)
        self.assertEqual(fused[0, 0], 100)

    def test_aligned_direct_mapping_matches_overlap_fusion(self):
        rng = np.random.default_rng(7)
        base = rng.choice([-1, 0, 60, 100], (8, 9)).astype(np.int8)
        previous = rng.choice([-1, 0, 100], (8, 9)).astype(np.int8)
        local = rng.choice([-1, 0, 74, 75, 100], (5, 6)).astype(np.int8)
        active = np.zeros(base.shape, dtype=bool)
        active[1:7, 2:8] = True
        for offset in [(0, 0), (-2, 3), (3, -1)]:
            layer = previous.copy()
            for y, x in np.ndindex(base.shape):
                lx, ly = x - offset[0], y - offset[1]
                if active[y, x] and 0 <= ly < local.shape[0] and 0 <= lx < local.shape[1]:
                    value = int(local[ly, lx])
                    if value >= 0:
                        layer[y, x] = value if value >= 75 else 0
            direct = np.where(base >= 75, base, np.where(layer >= 0, layer, base))
            slow_layer, slow = fuse_cells(base, previous, local, 1, offset, 0, active=active)
            np.testing.assert_array_equal(layer, slow_layer)
            np.testing.assert_array_equal(direct, slow)

    def test_fine_obstacle_and_partial_free_observation(self):
        base = np.zeros((3, 3), dtype=np.int8)
        layer = np.full(base.shape, -1, dtype=np.int8)
        local = np.array([[100]], dtype=np.int8)
        layer, fused = fuse_cells(base, layer, local, 0.1, (1.02, 1.02), 0)
        self.assertEqual(fused[1, 1], 100)  # center sampling would miss this obstacle
        partially_known = np.array([[0, 0], [0, -1]], dtype=np.int8)
        layer, fused = fuse_cells(base, layer, partially_known, 0.5, (1, 1), 0)
        self.assertEqual(fused[1, 1], 100)
        layer, fused = fuse_cells(base, layer, np.zeros((2, 2), dtype=np.int8), 0.5, (1, 1), 0)
        self.assertEqual(fused[1, 1], 0)

    def test_rotated_sensor_geometry_and_area_conservation(self):
        yaw = 0.37
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        corners = np.array([rotation @ p + [3.2, 4.1] for p in [[0, 0], [1.2, 0], [1.2, 1.2], [0, 1.2]]])
        total = sum(overlap_area(corners, x, y) for y in range(10) for x in range(10))
        self.assertAlmostEqual(total, 1.2 * 1.2, places=10)
        base = np.zeros((10, 10), dtype=np.int8)
        layer = np.full(base.shape, -1, dtype=np.int8)
        _, fused = fuse_cells(base, layer, np.array([[100]], dtype=np.int8), 1.2, (3.2, 4.1), yaw)
        for y, x in np.ndindex(base.shape):
            self.assertEqual(fused[y, x] >= 75, overlap_area(corners, x, y) > 1e-8)

    def test_global_and_local_origin_rotation_consistency(self):
        base = np.zeros((12, 12), dtype=np.int8)
        layer = np.full(base.shape, -1, dtype=np.int8)
        local = np.array([[0, 100], [-1, 0]], dtype=np.int8)
        _, reference = fuse_cells(base, layer, local, 0.7, (3.2, 4.1), 0.37)
        yaw = 0.5
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        origin = np.array([7.0, -3.0])
        local_origin = origin + rotation @ [3.2, 4.1]
        _, rotated = fuse_cells(base, layer, local, 0.7, local_origin, yaw + 0.37,
                                global_origin=origin, global_yaw=yaw)
        np.testing.assert_array_equal(rotated, reference)

    def test_old_region_history_and_active_window(self):
        base = np.zeros((4, 8), dtype=np.int8)
        layer = np.full(base.shape, -1, dtype=np.int8)
        left = np.zeros(base.shape, dtype=bool)
        left[:, :4] = True
        local = np.full(base.shape, 100, dtype=np.int8)
        layer, fused = fuse_cells(base, layer, local, 1, (0, 0), 0, active=left)
        self.assertTrue(np.all(fused[:, :4] == 100))
        self.assertTrue(np.all(fused[:, 4:] == 0))
        layer, moved = fuse_cells(base, layer, np.zeros(base.shape, dtype=np.int8), 1, (0, 0), 0, active=~left)
        np.testing.assert_array_equal(moved[:, :4], fused[:, :4])

    def test_message_patches_and_build_wiring(self):
        source = (ROOT / 'fast_planner/plan_env/src/sdf_map_grid_local.cpp').read_text()
        header = (ROOT / 'fast_planner/plan_env/include/plan_env/sdf_map.h').read_text()
        grid = (ROOT / 'fast_planner/plan_env/src/sdf_map_grid.cpp').read_text()
        self.assertIn('grid_local_values_', header)
        self.assertIn('grid_fused_.at<signed char>(id(1), id(0))', header)
        self.assertIn('map_msgs::msg::OccupancyGridUpdate', source)
        self.assertIn('grid_fused_update_pub_->publish(fused);', source)
        self.assertIn('grid_inflate_update_pub_->publish(inflated);', source)
        self.assertIn('if (!full && !insideGridWindow(x, y))', grid)
        self.assertIn('grid_distance_cap_ + mp_.obstacles_inflation_', grid)
        self.assertIn('src/sdf_map_grid_local.cpp', (ROOT / 'fast_planner/plan_env/CMakeLists.txt').read_text())
        dependencies = [e.text for e in ET.parse(ROOT / 'fast_planner/plan_env/package.xml').getroot().findall('depend')]
        self.assertIn('map_msgs', dependencies)
        tree = ast.parse((ROOT / 'fast_planner/plan_manage/launch/kino_replan.launch.py').read_text())
        values = {}
        wanted = ['sdf_map/grid_local_range', 'sdf_map/grid_distance_cap', 'sdf_map/grid_snapshot_period']
        for node in ast.walk(tree):
            if isinstance(node, ast.Dict):
                for key, value in zip(node.keys, node.values):
                    if isinstance(key, ast.Constant) and key.value in wanted:
                        values[key.value] = ast.literal_eval(value)
        self.assertEqual(values['sdf_map/grid_local_range'], 50)
        self.assertEqual(values['sdf_map/grid_distance_cap'], 10)
        self.assertGreater(values['sdf_map/grid_snapshot_period'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
