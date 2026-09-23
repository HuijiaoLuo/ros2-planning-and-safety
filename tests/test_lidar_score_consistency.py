import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from validate_lidar_score_consistency import analyze  # noqa: E402


class LidarScoreConsistencyTests(unittest.TestCase):
    def test_fixture_runs_all_score_models(self) -> None:
        rows = analyze(
            REPO_ROOT / "tests" / "fixtures" / "lidar_audit_minimal.jsonl",
            {0},
        )

        self.assertEqual(len(rows), 4)
        self.assertEqual(
            {str(row["score_mode"]) for row in rows},
            {"range", "endpoint", "boundary", "point_to_line"},
        )
        for row in rows:
            self.assertIn("synthetic_correction_m", row)
            self.assertIn("recorded_correction_m", row)


if __name__ == "__main__":
    unittest.main()
