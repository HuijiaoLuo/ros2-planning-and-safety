import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(REPO_ROOT / "tools"))

from analyze_localization_drift import (  # noqa: E402
    record_from_trace,
    summarize,
)


class LocalizationDriftTests(unittest.TestCase):
    def test_record_compares_estimates_against_odom_reference(self) -> None:
        row = {
            "x_m": "2.2",
            "y_m": "0.0",
            "navigation_x_m": "2.0",
            "navigation_y_m": "0.0",
            "state_estimate_x_m": "2.1",
            "state_estimate_y_m": "0.0",
            "ground_truth_goal_error_m": "0.2",
            "navigation_goal_error_m": "0.0",
            "state_estimate_goal_error_m": "0.1",
            "navigation_timestamp_offset_s": "-0.03",
            "state_estimate_timestamp_offset_s": "-0.02",
            "localization_match_status": "correction_too_large",
            "localization_candidate_correction_m": "0.2",
            "localization_applied_correction_m": "0.0",
        }

        record = record_from_trace(
            "baseline_obstacle__v4__seed_000",
            row,
            {"success": "False", "termination_reason": "goal_reached"},
        )

        self.assertAlmostEqual(record.navigation_truth_error_m, 0.2)
        self.assertAlmostEqual(record.state_estimate_truth_error_m, 0.1)
        self.assertAlmostEqual(record.navigation_goal_error_gap_m, 0.2)
        self.assertFalse(record.success)

    def test_summary_preserves_backend_and_status_groups(self) -> None:
        base_row = {
            "x_m": "2.2",
            "y_m": "0.0",
            "navigation_x_m": "2.0",
            "navigation_y_m": "0.0",
            "state_estimate_x_m": "2.1",
            "state_estimate_y_m": "0.0",
            "ground_truth_goal_error_m": "0.2",
            "navigation_goal_error_m": "0.0",
            "state_estimate_goal_error_m": "0.1",
            "navigation_timestamp_offset_s": "-0.03",
            "state_estimate_timestamp_offset_s": "-0.02",
            "localization_match_status": "correction_too_large",
        }
        records = [
            record_from_trace(
                "baseline_obstacle__v4__seed_000",
                base_row,
                {"success": "False", "termination_reason": "goal_reached"},
            ),
            record_from_trace(
                "baseline_obstacle__v4__seed_001",
                {**base_row, "localization_match_status": "accepted"},
                {"success": "True", "termination_reason": "goal_reached"},
            ),
        ]

        summary = summarize(records)

        self.assertEqual(len(summary), 1)
        self.assertEqual(summary[0]["runs"], 2)
        self.assertEqual(summary[0]["successes"], 1)
        self.assertEqual(
            summary[0]["localization_statuses"],
            "accepted=1;correction_too_large=1",
        )


if __name__ == "__main__":
    unittest.main()
