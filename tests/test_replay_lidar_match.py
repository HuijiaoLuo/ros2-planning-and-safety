import sys
import unittest
from pathlib import Path


TOOLS_ROOT = Path(__file__).parents[1] / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from replay_lidar_match import replay  # noqa: E402


class LidarAuditReplayTests(unittest.TestCase):
    def test_replay_matches_recorded_pure_matcher_result(self) -> None:
        audit_path = (
            Path(__file__).parent / "fixtures" / "lidar_audit_minimal.jsonl"
        )
        report = replay(audit_path)

        self.assertEqual(report["replayed_records"], 1)
        self.assertEqual(report["mismatch_count"], 0)
        self.assertEqual(report["point_count_mismatches"], 0)
        self.assertEqual(report["status_counts"], {"waiting_for_consecutive_matches": 1})
        self.assertEqual(report["quality_valid_records"], 1)
        self.assertEqual(report["accepted_records"], 0)
        self.assertEqual(report["candidate_not_applied_records"], 1)
        self.assertEqual(
            report["rejection_reason_counts"],
            {"minimum_consecutive_matches": 1},
        )
        self.assertAlmostEqual(report["max_pose_error_m"], 0.0)
        self.assertAlmostEqual(report["max_score_error_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
