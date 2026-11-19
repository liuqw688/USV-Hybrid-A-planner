"""验证闭环航点顺序、入口前移和非法航道拒绝。"""

import importlib.util
from pathlib import Path
import unittest


script = Path(__file__).resolve().parents[1] / 'scripts' / 'publish_clockwise_waypoints.py'
spec = importlib.util.spec_from_file_location('clockwise_waypoints', script)
waypoints = importlib.util.module_from_spec(spec)
spec.loader.exec_module(waypoints)


def lane(index, x, y, orient):
    """构造带方向的方形航道段。"""
    return {'lane_id': index, 'direction': 'CW', 'orient_deg': orient,
            'points': [[x - 1, y - 1], [x + 1, y - 1],
                       [x + 1, y + 1], [x - 1, y + 1]]}


class ClockwiseRouteTest(unittest.TestCase):
    """只验证航道几何，不替代规划器动力学和避碰测试。"""

    def setUp(self):
        self.chart = {'lanes': [lane(0, 0, 10, 135), lane(1, 10, 0, 225),
                                lane(2, 0, -10, 315), lane(3, -10, 0, 45)]}

    def test_closure_and_entry(self):
        route = waypoints.clockwise_route(self.chart, (0, 10), 1)
        self.assertEqual(route[0], (10.0, 0.0))
        self.assertEqual(route[0], route[-1])
        self.assertEqual(len(route), 5)
        self.assertEqual(len(set(route[:-1])), 4)

    def test_reject_inconsistent_direction(self):
        self.chart['lanes'][1]['orient_deg'] = 45
        with self.assertRaises(ValueError):
            waypoints.clockwise_route(self.chart, (0, 10), 0)

    def test_reject_degenerate_polygon(self):
        self.chart['lanes'][0]['points'] = [[0, 0], [1, 0], [2, 0]]
        with self.assertRaises(ValueError):
            waypoints.clockwise_route(self.chart, (0, 10), 0)


if __name__ == '__main__':
    unittest.main()
