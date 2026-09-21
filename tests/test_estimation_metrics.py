import math
import sys
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.estimation_metrics import PoseErrorStats, wrap_angle


class EstimationMetricTests(unittest.TestCase):
    def test_heading_error_wraps_across_pi(self) -> None:
        self.assertAlmostEqual(wrap_angle(-math.pi - 0.1), math.pi - 0.1, places=7)

    def test_summary_reports_position_and_heading_rmse(self) -> None:
        stats = PoseErrorStats()
        stats.add(0.0, 0.0, math.pi - 0.1, 0.3, 0.4, -math.pi + 0.1)
        summary = stats.summary("estimate")

        self.assertEqual(summary["estimate_samples"], 1)
        self.assertAlmostEqual(summary["estimate_x_rmse_m"], 0.3)
        self.assertAlmostEqual(summary["estimate_y_rmse_m"], 0.4)
        self.assertAlmostEqual(summary["estimate_position_rmse_m"], 0.5)
        self.assertAlmostEqual(summary["estimate_heading_rmse_rad"], 0.2)

    def test_empty_summary_is_explicit(self) -> None:
        summary = PoseErrorStats().summary("wheel")
        self.assertEqual(summary["wheel_samples"], 0)
        self.assertIsNone(summary["wheel_position_rmse_m"])


if __name__ == "__main__":
    unittest.main()
