import math
import unittest

from robotics_nav.icp_localization import (
    ICPConfig,
    point_to_point_icp,
    transform_point,
)
from robotics_nav.mcl_localization import OccupancyGridMap, wrap_angle


class ICPLocalizationTests(unittest.TestCase):
    def make_map(self) -> OccupancyGridMap:
        return OccupancyGridMap(
            width=20,
            height=20,
            resolution_m=0.1,
            origin_x_m=0.0,
            origin_y_m=0.0,
            occupied_cells=frozenset(
                {
                    (5, 5),
                    (6, 5),
                    (7, 5),
                    (8, 5),
                    (9, 5),
                    (5, 6),
                    (9, 6),
                    (5, 7),
                    (9, 7),
                    (5, 8),
                    (6, 8),
                    (7, 8),
                    (8, 8),
                    (9, 8),
                }
            ),
        )

    def test_point_to_point_icp_recovers_a_small_pose_offset(self) -> None:
        occupancy_map = self.make_map()
        true_pose = (0.80, 0.70, 0.05)
        initial_pose = (0.76, 0.67, 0.08)
        scan_points = []
        cosine = math.cos(true_pose[2])
        sine = math.sin(true_pose[2])
        for map_x, map_y in occupancy_map.occupied_centres:
            dx = map_x - true_pose[0]
            dy = map_y - true_pose[1]
            scan_points.append(
                (cosine * dx + sine * dy, -sine * dx + cosine * dy)
            )

        result = point_to_point_icp(
            scan_points,
            occupancy_map,
            initial_pose,
            config=ICPConfig(
                max_iterations=30,
                maximum_correspondence_distance_m=0.20,
            ),
        )

        self.assertTrue(result.converged)
        self.assertEqual(result.status, "converged")
        self.assertGreaterEqual(result.correspondence_count, 6)
        self.assertLess(result.mean_residual_m, 0.02)
        self.assertAlmostEqual(true_pose[0], result.pose[0], places=2)
        self.assertAlmostEqual(true_pose[1], result.pose[1], places=2)
        self.assertLess(abs(wrap_angle(true_pose[2] - result.pose[2])), 0.02)

    def test_transform_point_uses_planar_pose(self) -> None:
        point = transform_point((1.0, 0.0), (2.0, 3.0, math.pi / 2.0))
        self.assertAlmostEqual(2.0, point[0], places=8)
        self.assertAlmostEqual(4.0, point[1], places=8)
