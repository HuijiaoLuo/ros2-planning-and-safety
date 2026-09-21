import sys
from pathlib import Path
import unittest


TOOLS_ROOT = Path(__file__).parents[1] / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from summarize_estimation import summary_for_rows, summarize


def row(
    seed: int,
    wheel_heading: float,
    estimate_heading: float,
    slip_ratio: float = 0.0,
    wheel_weight: float = 0.02,
) -> dict[str, str]:
    return {
        "configured_imu_gyro_bias_rad_s": "0.0",
        "configured_imu_gyro_noise_std_rad_s": "0.03",
        "configured_imu_gyro_noise_seed": str(seed),
        "configured_wheel_slip_ratio": str(slip_ratio),
        "configured_wheel_weight": str(wheel_weight),
        "wheel_position_rmse_m": "0.08",
        "estimate_position_rmse_m": "0.081",
        "wheel_heading_rmse_rad": str(wheel_heading),
        "imu_heading_rmse_rad": "0.005",
        "estimate_heading_rmse_rad": str(estimate_heading),
        "wheel_heading_bias_rad": "0.01",
        "estimate_heading_bias_rad": "0.008",
        "wheel_final_position_error_m": "0.18",
        "estimate_final_position_error_m": "0.18",
        "estimate_final_heading_error_rad": "0.02",
    }


class EstimationSummaryTests(unittest.TestCase):
    def test_groups_seeds_and_reports_heading_improvement(self) -> None:
        summaries = summarize([row(2, 0.20, 0.16), row(1, 0.10, 0.08)])

        self.assertEqual(len(summaries), 1)
        summary = summaries[0]
        self.assertEqual(summary["runs"], 2)
        self.assertEqual(summary["seed_count"], 2)
        self.assertEqual(summary["seeds"], "1,2")
        self.assertAlmostEqual(summary["mean_wheel_heading_rmse_rad"], 0.15)
        self.assertAlmostEqual(summary["mean_estimate_heading_rmse_rad"], 0.12)
        self.assertAlmostEqual(summary["heading_rmse_improvement_pct"], 20.0)

    def test_missing_metric_is_reported_as_none(self) -> None:
        current = row(1, 0.10, 0.08)
        current["estimate_position_rmse_m"] = ""
        summary = summary_for_rows([current])
        self.assertIsNone(summary["mean_estimate_position_rmse_m"])

    def test_slip_ratio_creates_a_separate_configuration_group(self) -> None:
        summaries = summarize([row(1, 0.10, 0.08), row(1, 0.10, 0.07, 0.10)])

        self.assertEqual(len(summaries), 2)
        self.assertEqual(summaries[0]["configured_wheel_slip_ratio"], 0.0)
        self.assertEqual(summaries[1]["configured_wheel_slip_ratio"], 0.1)

    def test_wheel_weight_creates_a_separate_configuration_group(self) -> None:
        summaries = summarize(
            [row(1, 0.10, 0.08, wheel_weight=0.0), row(1, 0.10, 0.07)]
        )

        self.assertEqual(len(summaries), 2)
        self.assertEqual(summaries[0]["configured_wheel_weight"], 0.0)
        self.assertEqual(summaries[1]["configured_wheel_weight"], 0.02)


if __name__ == "__main__":
    unittest.main()
