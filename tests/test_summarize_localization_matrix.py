import sys
from pathlib import Path
import unittest


TOOLS_ROOT = Path(__file__).parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from summarize_localization_matrix import summarize  # noqa: E402


class LocalizationMatrixSummaryTests(unittest.TestCase):
    def test_groups_by_scenario_and_backend(self) -> None:
        rows = [
            {
                "scenario": "baseline_obstacle",
                "localization_backend": "mcl",
                "configured_scan_noise_seed": "0",
                "success": "True",
                "termination_reason": "goal_reached",
                "ground_truth_final_error_m": "0.04",
                "navigation_pose_final_error_m": "0.03",
                "state_estimate_final_error_m": "0.05",
                "time_to_goal_s": "60.0",
                "collision": "false",
                "safety_override_ratio": "0.01",
            },
            {
                "scenario": "baseline_obstacle",
                "localization_backend": "mcl",
                "configured_scan_noise_seed": "1",
                "success": "False",
                "termination_reason": "experiment_timeout",
                "ground_truth_final_error_m": "0.12",
                "navigation_pose_final_error_m": "0.02",
                "state_estimate_final_error_m": "0.09",
                "time_to_goal_s": "",
                "collision": "false",
                "safety_override_ratio": "0.20",
            },
        ]

        result = summarize(rows)

        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["runs"], 2)
        self.assertEqual(result[0]["seeds"], "0,1")
        self.assertEqual(result[0]["successes"], 1)
        self.assertEqual(result[0]["experiment_timeout_count"], 1)
        self.assertAlmostEqual(result[0]["success_rate"], 0.5)
        self.assertAlmostEqual(result[0]["mean_final_error_m"], 0.08)
        self.assertAlmostEqual(result[0]["mean_time_to_goal_s"], 60.0)


if __name__ == "__main__":
    unittest.main()
