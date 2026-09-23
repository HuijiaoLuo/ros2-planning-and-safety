import unittest
from pathlib import Path

from tools.summarize_mcl_diagnostics import summarize


class MCLDiagnosticsTests(unittest.TestCase):
    def test_summary_uses_mcl_update_records(self) -> None:
        path = Path(__file__).parent / "fixtures" / "mcl_diagnostics_minimal.jsonl"
        result = summarize(path)
        self.assertEqual(result["update_count"], 2)
        self.assertEqual(result["belief_event_count"], 2)
        self.assertEqual(result["belief_measurement_applied_count"], 1)
        self.assertAlmostEqual(result["mean_belief_update_latency_s"], 0.35)
        self.assertEqual(result["accepted_update_count"], 1)
        self.assertAlmostEqual(result["accepted_update_ratio"], 0.5)
        self.assertAlmostEqual(result["mean_update_latency_s"], 0.3)
        self.assertAlmostEqual(result["mean_log_likelihood_gain"], 0.15)
        self.assertAlmostEqual(result["accepted_mean_log_likelihood_gain"], 0.4)
        self.assertEqual(result["resample_count"], 1)


if __name__ == "__main__":
    unittest.main()
