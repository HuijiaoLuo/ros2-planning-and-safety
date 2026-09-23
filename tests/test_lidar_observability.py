import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from analyze_lidar_observability import analyze  # noqa: E402


class LidarObservabilityTests(unittest.TestCase):
    def test_fixture_reports_axis_and_gate_diagnostics(self) -> None:
        rows = analyze(
            REPO_ROOT / "tests" / "fixtures" / "lidar_audit_minimal.jsonl"
        )

        self.assertEqual(len(rows), 1)
        self.assertIn("x_curvature", rows[0])
        self.assertIn("y_curvature", rows[0])
        self.assertIn("hessian_eigen_min", rows[0])
        self.assertIn("hessian_condition_abs", rows[0])
        self.assertIn("mahalanobis_gate_ratio", rows[0])
        self.assertIn("top_candidate_span_m", rows[0])

    def test_point_to_line_is_a_supported_audit_override(self) -> None:
        rows = analyze(
            REPO_ROOT / "tests" / "fixtures" / "lidar_audit_minimal.jsonl",
            score_mode="point_to_line",
        )

        self.assertEqual(len(rows), 1)
        self.assertIsNotNone(rows[0]["prior_score_m"])


if __name__ == "__main__":
    unittest.main()
