import unittest

from tools.diagnose_navigation_trace import diagnose


class NavigationDiagnosisTests(unittest.TestCase):
    def test_historical_goal_entry_is_not_terminal_success(self) -> None:
        evaluation = {
            "success": "True",
            "controller_goal_latched": "False",
            "ground_truth_goal_reached_any_time": "True",
            "ground_truth_final_within_goal_tolerance": "False",
            "navigation_pose_goal_reached_any_time": "True",
            "state_estimate_goal_reached_any_time": "True",
            "termination_reason": "experiment_timeout",
            "ground_truth_final_error_m": "0.136",
            "navigation_pose_final_error_m": "0.097",
            "state_estimate_final_error_m": "0.100",
        }

        report = diagnose(evaluation, [])

        self.assertFalse(report["controller_success"])
        self.assertTrue(report["physical_success"])
        self.assertFalse(report["physical_final_within_tolerance"])
        self.assertEqual(
            report["classification"],
            "physical_goal_reached_but_not_completed",
        )


if __name__ == "__main__":
    unittest.main()
