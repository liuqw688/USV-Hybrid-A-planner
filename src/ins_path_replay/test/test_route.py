"""Geodesy, arc sampling and recorded-data contract checks (no ROS required)."""
import csv
import math
from pathlib import Path
import tempfile
import unittest

from ins_path_replay.route import load_route, local_enu, sample_equal_arc


class RouteChecks(unittest.TestCase):
    def test_first_point_is_exact_origin(self):
        p=(math.radians(29.5),math.radians(106.7),247.)
        self.assertEqual(local_enu(*p,p),(0.,0.,0.))

    def test_east_north_sign_and_scale_at_equator(self):
        origin=(0.,0.,0.)
        east=local_enu(0.,1e-6,0.,origin)
        north=local_enu(1e-6,0.,0.,origin)
        self.assertAlmostEqual(east[0],6.378137,places=6)
        self.assertAlmostEqual(east[1],0.,places=8)
        self.assertAlmostEqual(north[0],0.,places=8)
        self.assertAlmostEqual(north[1],6.335439327,places=6)

    def test_arc_sampling_follows_bent_polyline_instead_of_rows(self):
        points,yaws,arcs=sample_equal_arc([(0,0),(1,0),(4,0),(4,6)],6)
        self.assertEqual(points,((0.,0.),(2.,0.),(4.,0.),(4.,2.),(4.,4.),(4,6)))
        self.assertEqual(arcs,(0.,2.,4.,6.,8.,10.))
        self.assertAlmostEqual(yaws[1],0.)
        self.assertAlmostEqual(yaws[2],math.pi/2)
        self.assertEqual(yaws[-1],yaws[-2])

    def test_duplicates_do_not_add_arc(self):
        p,_,s=sample_equal_arc([(0,0),(0,0),(0,3),(0,3)],4)
        self.assertEqual(p,((0.,0.),(0.,1.),(0.,2.),(0,3)))
        self.assertEqual(s[-1],3.)

    def test_stationary_and_invalid_data_are_rejected(self):
        for points in ([],[(0,0)],[(0,0),(0,0)],[(0,0),(math.nan,0)]):
            with self.assertRaises(ValueError): sample_equal_arc(points,30)
        with self.assertRaises(ValueError): sample_equal_arc([(0,0),(1,1)],1)

    def test_degrees_and_radians_agree(self):
        with tempfile.TemporaryDirectory() as tmp:
            for unit in ('degrees','radians'):
                with open(Path(tmp)/f'{unit}.csv','w') as f:
                    w=csv.writer(f);w.writerow(['INS_LAT','INS_LON','ins_altitude'])
                    for lat,lon in [(29.5,106.7),(29.5001,106.7001)]:
                        w.writerow([lat if unit=='degrees' else math.radians(lat),
                                    lon if unit=='degrees' else math.radians(lon),200.])
            a=load_route(Path(tmp)/'degrees.csv',angle_unit='degrees')
            b=load_route(Path(tmp)/'radians.csv',angle_unit='radians')
            for pa,pb in zip(a.points,b.points): self.assertLess(math.dist(pa,pb),1e-7)

    def test_dataset_has_30_ordered_positions_and_forward_headings(self):
        csv_path=Path(__file__).resolve().parents[1]/'data/data_ins_20261019_153413.csv'
        route=load_route(csv_path)
        self.assertEqual(len(route.points),30)
        self.assertEqual(route.points[0],(0.,0.))
        self.assertEqual(route.points[-1],route.raw[-1])
        self.assertEqual(route.metadata['rows'],17342)
        self.assertAlmostEqual(route.metadata['raw_length_m'],958.41844,places=4)
        for i in range(29):
            self.assertAlmostEqual(route.sample_arcs[i+1]-route.sample_arcs[i],route.metadata['sample_arc_spacing_m'])
            dx=route.points[i+1][0]-route.points[i][0];dy=route.points[i+1][1]-route.points[i][1]
            self.assertAlmostEqual(route.headings[i],math.atan2(dy,dx))


if __name__=='__main__': unittest.main()
