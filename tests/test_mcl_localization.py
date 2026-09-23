import math
import unittest

from robotics_nav.mcl_localization import (
    MCLConfig,
    OccupancyGridMap,
    Particle,
    ParticleFilter2D,
    wrap_angle,
)


class MCLLocalizationTests(unittest.TestCase):
    def make_map(self) -> OccupancyGridMap:
        return OccupancyGridMap.from_ascii(
            [
                "########",
                "#......#",
                "#..##..#",
                "#......#",
                "########",
            ]
        )

    def make_filter(self, count: int = 20) -> ParticleFilter2D:
        return ParticleFilter2D(
            self.make_map(),
            MCLConfig(
                particle_count=count,
                motion_distance_noise_std_m=0.0,
                motion_yaw_noise_std_rad=0.0,
                random_seed=7,
            ),
        )

    def test_midpoint_motion_is_deterministic_without_noise(self) -> None:
        particle_filter = self.make_filter()
        particle_filter.set_particles(
            Particle(2.5, 1.5, 0.0, 1.0 / 20.0) for _ in range(20)
        )
        particle_filter.predict(1.0, math.pi / 2.0)
        estimate = particle_filter.estimate()
        self.assertAlmostEqual(2.5 + math.sqrt(0.5), estimate.pose[0], places=8)
        self.assertAlmostEqual(1.5 + math.sqrt(0.5), estimate.pose[1], places=8)
        self.assertAlmostEqual(math.pi / 2.0, estimate.pose[2], places=8)

    def test_likelihood_field_prefers_endpoint_on_wall(self) -> None:
        particle_filter = self.make_filter()
        scan = [1.0]
        correct_score, correct_count = particle_filter.pose_log_likelihood(
            (2.5, 2.5, 0.0),
            scan,
            angle_min_rad=0.0,
            angle_increment_rad=0.0,
            range_min_m=0.05,
            range_max_m=10.0,
        )
        wrong_score, wrong_count = particle_filter.pose_log_likelihood(
            (2.5, 1.5, 0.0),
            scan,
            angle_min_rad=0.0,
            angle_increment_rad=0.0,
            range_min_m=0.05,
            range_max_m=10.0,
        )
        self.assertEqual(correct_count, 1)
        self.assertEqual(wrong_count, 1)
        self.assertGreater(correct_score, wrong_score)

    def test_systematic_resampling_preserves_count_and_normalizes_weights(self) -> None:
        particle_filter = self.make_filter()
        particle_filter.set_particles(
            [
                Particle(1.5, 1.5, 0.0, 0.9),
                *[Particle(2.5, 1.5, 0.0, 0.1 / 19.0) for _ in range(19)],
            ]
        )
        particle_filter.resample_systematic()
        self.assertEqual(len(particle_filter.particles), 20)
        self.assertAlmostEqual(
            1.0,
            sum(particle.weight for particle in particle_filter.particles),
            places=12,
        )
        self.assertGreaterEqual(
            sum(particle.x_m == 1.5 for particle in particle_filter.particles),
            10,
        )

    def test_circular_heading_mean_does_not_jump_at_pi(self) -> None:
        particle_filter = self.make_filter(count=4)
        particle_filter.set_particles(
            [
                Particle(1.5, 1.5, math.pi - 0.02, 0.25),
                Particle(1.5, 1.5, -math.pi + 0.02, 0.25),
                Particle(1.5, 1.5, math.pi - 0.01, 0.25),
                Particle(1.5, 1.5, -math.pi + 0.01, 0.25),
            ]
        )
        estimate = particle_filter.estimate()
        self.assertLess(abs(abs(estimate.pose[2]) - math.pi), 0.02)
        self.assertAlmostEqual(0.0, wrap_angle(estimate.pose[2] - math.pi), places=2)

    def test_update_reports_multimodal_spread_instead_of_false_certainty(self) -> None:
        particle_filter = self.make_filter(count=20)
        particle_filter.set_particles(
            [
                Particle(1.5, 1.5, 0.0, 0.05) for _ in range(10)
            ]
            + [
                Particle(5.5, 1.5, math.pi, 0.05) for _ in range(10)
            ]
        )
        result = particle_filter.update(
            [],
            angle_min_rad=0.0,
            angle_increment_rad=0.0,
            range_min_m=0.05,
            range_max_m=10.0,
        )
        self.assertAlmostEqual(0.0, result.valid_beam_count)
        self.assertGreater(result.covariance[0][0], 1.0)
        self.assertAlmostEqual(1.0, result.normalized_entropy, places=8)


if __name__ == "__main__":
    unittest.main()
