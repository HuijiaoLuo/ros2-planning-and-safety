import math
import sys
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.lidar_localization import (
    LidarMapMatcher,
    compose_pose,
    interpolate_pose,
    inverse_pose,
    map_odom_from_poses,
    transform_pose,
)


class LidarMapMatcherTests(unittest.TestCase):
    def make_matcher(self) -> LidarMapMatcher:
        matcher = LidarMapMatcher(
            search_radius_m=0.30,
            search_step_m=0.025,
            scan_stride=1,
            max_match_distance_m=0.30,
            prior_weight=0.02,
            minimum_points=3,
        )
        # A vertical occupied wall at x=1.0, with cell centres at y=-1, 0, 1.
        matcher.update_map(
            width=5,
            height=5,
            resolution=0.5,
            origin_x=-0.25,
            origin_y=-1.25,
            data=[
                0, 0, 100, 0, 0,
                0, 0, 100, 0, 0,
                0, 0, 100, 0, 0,
                0, 0, 100, 0, 0,
                0, 0, 100, 0, 0,
            ],
        )
        return matcher

    def test_matcher_corrects_a_local_x_offset(self) -> None:
        matcher = self.make_matcher()
        ranges = [0.75, 0.75, 0.75]
        corrected_x, corrected_y, score, points = matcher.match(
            0.20,
            0.0,
            0.0,
            ranges,
            angle_min=-0.1,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
        )

        self.assertEqual(points, 3)
        self.assertAlmostEqual(corrected_x, 0.10, delta=0.06)
        self.assertAlmostEqual(corrected_y, 0.0, delta=0.06)
        self.assertTrue(math.isfinite(score))

    def test_matcher_search_radius_is_euclidean(self) -> None:
        matcher = self.make_matcher()
        matcher.search_radius_m = 0.15
        corrected_x, corrected_y, _, _, _ = matcher.match_pose(
            0.20,
            0.0,
            0.0,
            [0.75, 0.75, 0.75],
            angle_min=-0.1,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
            yaw_search_radius_rad=0.0,
        )

        correction = math.hypot(corrected_x - 0.20, corrected_y)
        self.assertLessEqual(correction, 0.15 + 1.0e-12)

    def test_matcher_reports_free_and_occupied_cells(self) -> None:
        matcher = self.make_matcher()

        self.assertTrue(matcher.is_free(0.0, 0.0))
        self.assertFalse(matcher.is_free(1.0, 0.0))
        self.assertFalse(matcher.is_free(10.0, 0.0))
        self.assertFalse(matcher.is_free(0.4, 0.0, clearance_radius_m=0.35))

    def test_diagnostic_score_modes_return_finite_scores(self) -> None:
        """The optional endpoint models must remain usable on the fixture map."""
        for score_mode in ("endpoint", "boundary", "point_to_line"):
            matcher = LidarMapMatcher(
                search_radius_m=0.30,
                search_step_m=0.025,
                scan_stride=1,
                max_match_distance_m=0.30,
                prior_weight=0.02,
                score_mode=score_mode,
                minimum_points=3,
            )
            matcher.update_map(
                width=5,
                height=5,
                resolution=0.5,
                origin_x=-0.25,
                origin_y=-1.25,
                data=[
                    0, 0, 100, 0, 0,
                    0, 0, 100, 0, 0,
                    0, 0, 100, 0, 0,
                    0, 0, 100, 0, 0,
                    0, 0, 100, 0, 0,
                ],
            )
            _, _, score, points = matcher.match(
                0.20,
                0.0,
                0.0,
                [0.75, 0.75, 0.75],
                angle_min=-0.1,
                angle_increment=0.1,
                range_min=0.1,
                range_max=5.0,
            )
            self.assertEqual(points, 3)
            self.assertTrue(math.isfinite(score), score_mode)

    def test_coarse_to_fine_optimizer_reports_ranked_candidates(self) -> None:
        """The optional search refinement remains deterministic and auditable."""
        matcher = self.make_matcher()
        matcher.optimizer_mode = "coarse_to_fine"
        corrected = matcher.match_pose(
            0.20,
            0.0,
            0.0,
            [0.75, 0.75, 0.75],
            angle_min=-0.1,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
            yaw_search_radius_rad=0.0,
        )

        self.assertTrue(math.isfinite(corrected[3]))
        self.assertGreater(
            int(matcher.last_match_diagnostics["evaluated_candidate_count"]),
            0,
        )
        self.assertIn("score_margin_m", matcher.last_match_diagnostics)
        self.assertTrue(matcher.last_match_diagnostics["top_candidates"])

    def test_insufficient_returns_keep_the_prior_pose(self) -> None:
        matcher = self.make_matcher()
        corrected = matcher.match(
            0.8,
            -0.2,
            0.0,
            [float("inf")],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=5.0,
        )

        self.assertEqual(corrected[:2], (0.8, -0.2))
        self.assertEqual(corrected[3], 0)

    def test_map_odom_transform_reproduces_a_map_pose(self) -> None:
        odom_pose = (1.0, -0.2, 0.3)
        map_pose = (1.4, 0.5, 0.6)
        transform = map_odom_from_poses(map_pose, odom_pose)
        recovered = transform_pose(transform, odom_pose)

        for value, expected in zip(recovered, map_pose):
            self.assertAlmostEqual(value, expected, places=9)

    def test_pose_composition_and_inverse_cancel(self) -> None:
        pose = (0.7, -0.4, 1.1)
        identity = compose_pose(pose, inverse_pose(pose))

        self.assertAlmostEqual(identity[0], 0.0, places=9)
        self.assertAlmostEqual(identity[1], 0.0, places=9)
        self.assertAlmostEqual(identity[2], 0.0, places=9)

    def test_transform_interpolation_wraps_heading(self) -> None:
        interpolated = interpolate_pose(
            (0.0, 0.0, math.pi - 0.1),
            (1.0, 2.0, -math.pi + 0.1),
            0.5,
        )

        self.assertAlmostEqual(interpolated[0], 0.5)
        self.assertAlmostEqual(interpolated[1], 1.0)
        self.assertAlmostEqual(abs(abs(interpolated[2]) - math.pi), 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
