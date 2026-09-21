import unittest

from tools.summarize_robustness import summarize


class RobustnessSummaryTests(unittest.TestCase):
    def test_groups_runs_and_ignores_incomplete_goal_time(self) -> None:
        common = {
            "configured_minimum_clearance_m": "0.50",
            "configured_sensor_latency_s": "0.10",
            "configured_scan_delay_s": "0.20",
            "configured_scan_noise_std_m": "0.03",
            "configured_safety_margin_m": "0.15",
            "configured_planning_radius_m": "0.41",
            "collision": "false",
            "final_error_m": "0.04",
            "travelled_distance_m": "3.98",
            "minimum_clearance_m": "0.63",
            "safety_override_count": "1",
            "safety_override_time_s": "0.10",
            "safety_override_ratio": "0.01",
            "path_efficiency": "1.02",
            "configured_scan_noise_seed": "1",
            "termination_reason": "goal_reached",
        }
        successful = {
            **common,
            "success": "True",
            "time_to_goal_s": "70.0",
        }
        incomplete = {
            **common,
            "success": "False",
            "time_to_goal_s": "",
            "collision": "false",
            "configured_scan_noise_seed": "2",
            "termination_reason": "experiment_timeout",
        }

        [summary] = summarize([successful, incomplete])

        self.assertEqual(summary["runs"], 2)
        self.assertEqual(summary["seed_count"], 2)
        self.assertEqual(summary["successes"], 1)
        self.assertEqual(summary["goal_reached_count"], 1)
        self.assertEqual(summary["experiment_timeout_count"], 1)
        self.assertEqual(summary["manual_interrupt_count"], 0)
        self.assertEqual(summary["unknown_termination_count"], 0)
        self.assertEqual(summary["success_rate"], 0.5)
        self.assertEqual(summary["mean_time_to_goal_s"], 70.0)
        self.assertEqual(summary["collision_count"], 0)
        self.assertEqual(summary["seeds"], "1,2")


if __name__ == "__main__":
    unittest.main()
