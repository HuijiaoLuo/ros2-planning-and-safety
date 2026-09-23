import sys
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
import unittest


REPO_ROOT = Path(__file__).parents[1]
TOOLS_ROOT = REPO_ROOT / "tools"
sys.path.insert(0, str(TOOLS_ROOT))

from run_localization_matrix import (  # noqa: E402
    RunSpec,
    build_command,
    build_specs,
    validate_run_artifacts,
)


class FakeArtifact:
    def __init__(self, content: str):
        self.content = content

    def stat(self) -> SimpleNamespace:
        return SimpleNamespace(st_size=len(self.content), st_mtime=1.0)

    def open(self, *args, **kwargs) -> StringIO:
        return StringIO(self.content)


class LocalizationMatrixTests(unittest.TestCase):
    def test_build_specs_has_one_row_per_combination(self) -> None:
        specs = build_specs(
            ("baseline_obstacle", "l_corridor"),
            ("v4", "mcl", "icp"),
            (0, 1),
        )
        self.assertEqual(len(specs), 12)
        self.assertEqual(specs[0].stem, "baseline_obstacle__v4__seed_000")
        self.assertEqual(specs[-1].stem, "l_corridor__icp__seed_001")

    def test_command_keeps_control_on_motion_prior(self) -> None:
        spec = RunSpec("symmetric_corridor", "mcl", 2)
        command = build_command(
            spec,
            output_dir=Path("results/localization_matrix"),
            experiment_timeout_s=120.0,
            mcl_initialization_mode="local",
        )
        self.assertIn("scenario:=symmetric_corridor", command)
        self.assertIn("localization_backend:=mcl", command)
        self.assertIn("navigation_pose_topic:=/localized_estimate", command)
        self.assertIn("control_pose_topic:=/state_prediction", command)
        self.assertIn("motion_prior_topic:=/state_prediction", command)
        self.assertIn("mcl_random_seed:=2", command)
        self.assertIn("experiment_timeout_s:=120.000000", command)

    def test_incomplete_artifacts_are_not_a_successful_run(self) -> None:
        paths = {
            "evaluation": FakeArtifact(
                "termination_reason,samples\nexperiment_timeout,0\n"
            ),
            "trace": FakeArtifact("time_s\n"),
            "metrics": FakeArtifact("samples\n0\n"),
            "estimation_trace": FakeArtifact("time_s\n"),
            "diagnostic": FakeArtifact('{"event":"config"}\n'),
        }

        completed, reason = validate_run_artifacts(paths)

        self.assertFalse(completed)
        self.assertEqual(reason, "evaluation_samples_zero")

    def test_valid_artifacts_require_positive_samples(self) -> None:
        paths = {
            "evaluation": FakeArtifact(
                "termination_reason,samples\nexperiment_timeout,3\n"
            ),
            "trace": FakeArtifact("time_s\n0.0\n"),
            "metrics": FakeArtifact("samples\n3\n"),
            "estimation_trace": FakeArtifact("time_s\n0.0\n"),
            "diagnostic": FakeArtifact(
                '{"event":"config"}\n{"event":"update"}\n'
            ),
        }

        completed, reason = validate_run_artifacts(paths)

        self.assertTrue(completed)
        self.assertEqual(reason, "ok")


if __name__ == "__main__":
    unittest.main()
