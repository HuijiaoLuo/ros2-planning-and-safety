import sys
import unittest
from pathlib import Path


TOOLS_ROOT = Path(__file__).parents[1] / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from summarize_covariance_calibration import summarize_trace  # noqa: E402


class CovarianceCalibrationTests(unittest.TestCase):
    def test_summary_reports_nees_and_coverage(self) -> None:
        path = Path(__file__).parent / "fixtures" / "covariance_trace_minimal.csv"
        summary = summarize_trace(path)

        self.assertEqual(summary["samples"], 2)
        self.assertAlmostEqual(summary["mean_position_nees"], 1.0)
        self.assertAlmostEqual(summary["position_95pct_coverage"], 1.0)
        self.assertAlmostEqual(summary["mean_heading_nees"], 0.5)


if __name__ == "__main__":
    unittest.main()
