import math
import sys
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.heading_fusion import (
    AdaptiveHeadingFusion,
    DifferentialDrivePoseModel,
    GyroMeasurementModel,
    HeadingFusion,
    WheelSlipMeasurementModel,
    blend_angles,
    wrap_angle,
)
from robotics_nav.pose_ekf import PoseEKF


class HeadingFusionTests(unittest.TestCase):
    def test_wrap_angle_uses_shortest_representation(self) -> None:
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), math.pi, places=7)
        self.assertAlmostEqual(abs(wrap_angle(-3.0 * math.pi)), math.pi, places=7)

    def test_blend_angles_crosses_pi_without_large_jump(self) -> None:
        result = blend_angles(math.pi - 0.1, -math.pi + 0.1, 0.5)
        self.assertAlmostEqual(abs(result), math.pi, delta=0.11)

    def test_gyro_integrates_and_wheel_odometry_anchors_heading(self) -> None:
        fusion = HeadingFusion(wheel_weight=0.02)
        fusion.update_wheel(0.0)
        self.assertAlmostEqual(fusion.update_gyro(0.5, 0.0), 0.0)
        first = fusion.update_gyro(0.5, 1.0)
        self.assertIsNotNone(first)
        self.assertAlmostEqual(first, 0.50, delta=0.01)

        anchored = fusion.update_wheel(0.40)
        for _ in range(100):
            anchored = fusion.update_wheel(0.40)
        self.assertLess(abs(anchored - 0.40), abs(first - 0.40))

    def test_gyro_measurement_bias_is_explicit(self) -> None:
        model = GyroMeasurementModel(bias_rad_s=0.02, seed=7)
        self.assertAlmostEqual(model.apply(0.5), 0.52)

    def test_gyro_noise_is_reproducible_from_seed(self) -> None:
        first = GyroMeasurementModel(noise_std_rad_s=0.01, seed=7)
        second = GyroMeasurementModel(noise_std_rad_s=0.01, seed=7)
        self.assertEqual(
            [first.apply(0.0) for _ in range(3)],
            [second.apply(0.0) for _ in range(3)],
        )

    def test_wheel_slip_reduces_translational_increments(self) -> None:
        model = WheelSlipMeasurementModel(slip_ratio=0.25)

        model.apply(0.0, 0.0, 0.0)
        estimate = model.apply(1.0, 0.4, 0.2)

        self.assertAlmostEqual(estimate[0], 0.75)
        self.assertAlmostEqual(estimate[1], 0.30)
        self.assertAlmostEqual(estimate[2], 0.2)

    def test_zero_wheel_slip_preserves_pose_increments(self) -> None:
        model = WheelSlipMeasurementModel(slip_ratio=0.0)

        model.apply(0.2, -0.1, 0.1)
        estimate = model.apply(0.5, 0.3, 0.4)

        self.assertAlmostEqual(estimate[0], 0.5)
        self.assertAlmostEqual(estimate[1], 0.3)
        self.assertAlmostEqual(estimate[2], 0.4)

    def test_propagated_pose_uses_fused_heading_and_body_speed(self) -> None:
        model = DifferentialDrivePoseModel(slip_ratio=0.0)

        first = model.apply(1.0, 2.0, 0.0, 0.0, 0.0)
        second = model.apply(1.0, 2.0, 1.0, 1.0, 0.0)

        self.assertEqual(first, (1.0, 2.0, 0.0))
        self.assertAlmostEqual(second[0], 2.0)
        self.assertAlmostEqual(second[1], 2.0)
        self.assertAlmostEqual(second[2], 0.0)

    def test_propagated_pose_scales_translation_for_slip(self) -> None:
        model = DifferentialDrivePoseModel(slip_ratio=0.25)
        model.apply(0.0, 0.0, 0.0, 0.0, 0.0)
        estimate = model.apply(0.0, 0.0, 1.0, 1.0, 0.0)

        self.assertAlmostEqual(estimate[0], 0.75)
        self.assertAlmostEqual(estimate[1], 0.0)

    def test_adaptive_fusion_computes_a_wheel_gain(self) -> None:
        fusion = AdaptiveHeadingFusion(
            gyro_rate_noise_std_rad_s=0.02,
            wheel_yaw_noise_std_rad=0.05,
        )
        fusion.update_wheel(0.0)
        fusion.update_gyro(0.0, 0.0)
        fusion.update_gyro(0.4, 1.0)
        corrected = fusion.update_wheel(0.0)

        self.assertGreater(fusion.last_gain, 0.0)
        self.assertLess(fusion.last_gain, 1.0)
        self.assertLess(abs(corrected), 0.4)

    def test_adaptive_gain_trusts_lower_variance_measurement(self) -> None:
        low_noise = AdaptiveHeadingFusion(wheel_yaw_noise_std_rad=0.01)
        high_noise = AdaptiveHeadingFusion(wheel_yaw_noise_std_rad=0.20)
        for fusion in (low_noise, high_noise):
            fusion.update_wheel(0.0)
            fusion.update_gyro(0.2, 0.0)
            fusion.update_gyro(0.2, 1.0)
            fusion.update_wheel(0.0)

        self.assertGreater(low_noise.last_gain, high_noise.last_gain)

    def test_adaptive_fusion_estimates_positive_gyro_bias(self) -> None:
        fusion = AdaptiveHeadingFusion(
            gyro_rate_noise_std_rad_s=0.001,
            wheel_yaw_noise_std_rad=0.01,
            gyro_bias_random_walk_std_rad_s2=0.01,
        )
        fusion.update_wheel(0.0)
        stamp = 0.0
        for _ in range(20):
            fusion.update_gyro(0.05, stamp)
            fusion.update_wheel(0.0)
            stamp += 0.1

        self.assertGreater(fusion.bias_estimate, 0.0)

    def test_innovation_adaptive_wheel_noise_increases_for_large_innovation(self) -> None:
        fusion = AdaptiveHeadingFusion(
            gyro_rate_noise_std_rad_s=0.01,
            wheel_yaw_noise_std_rad=0.05,
            initial_heading_variance_rad2=0.01,
            adaptive_wheel_noise=True,
            wheel_yaw_noise_min_std_rad=0.02,
            wheel_yaw_noise_max_std_rad=0.20,
            wheel_noise_adaptation_rate=0.5,
        )
        fusion.update_wheel(0.0)
        initial_noise = fusion.wheel_yaw_noise_std_estimate
        fusion.update_gyro(0.4, 0.0)
        fusion.update_gyro(0.4, 1.0)
        fusion.update_wheel(0.0)

        self.assertGreater(fusion.wheel_yaw_noise_std_estimate, initial_noise)
        self.assertLessEqual(fusion.wheel_yaw_noise_std_estimate, 0.20)

    def test_innovation_adaptive_wheel_noise_respects_lower_bound(self) -> None:
        fusion = AdaptiveHeadingFusion(
            wheel_yaw_noise_std_rad=0.05,
            adaptive_wheel_noise=True,
            wheel_yaw_noise_min_std_rad=0.03,
            wheel_yaw_noise_max_std_rad=0.10,
            wheel_noise_adaptation_rate=0.5,
        )
        fusion.update_wheel(0.0)
        fusion.update_gyro(0.0, 0.0)
        fusion.update_gyro(0.0, 1.0)
        fusion.update_wheel(0.0)

        self.assertGreaterEqual(fusion.wheel_yaw_noise_std_estimate, 0.03)
        self.assertLessEqual(fusion.wheel_yaw_noise_std_estimate, 0.10)

    def test_pose_ekf_propagates_position_from_wheel_speed(self) -> None:
        ekf = PoseEKF(
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
        )
        ekf.update_wheel(0.0, x=1.0, y=2.0, linear_velocity_x=1.0)
        ekf.update_gyro(0.0, 0.0)
        ekf.update_gyro(0.0, 1.0)

        x, y, yaw = ekf.pose
        self.assertAlmostEqual(x, 2.0, places=6)
        self.assertAlmostEqual(y, 2.0, places=6)
        self.assertAlmostEqual(yaw, 0.0, places=6)
        self.assertEqual(len(ekf.pose_covariance_6x6), 36)

    def test_pose_ekf_wheel_yaw_update_reduces_heading_uncertainty(self) -> None:
        ekf = PoseEKF(
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
            wheel_yaw_noise_std_rad=0.05,
        )
        ekf.update_wheel(0.0)
        initial_variance = ekf.covariance[2][2]
        ekf.update_gyro(0.4, 0.0)
        ekf.update_gyro(0.4, 1.0)
        ekf.update_wheel(0.0)

        self.assertTrue(ekf.last_measurement_accepted)
        self.assertGreater(ekf.last_gain, 0.0)
        self.assertLess(ekf.covariance[2][2], initial_variance)

    def test_pose_ekf_tracks_wheel_yaw_bias_as_a_state(self) -> None:
        ekf = PoseEKF(
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
            wheel_yaw_noise_std_rad=0.02,
            initial_wheel_yaw_bias_variance_rad2=0.04,
            wheel_yaw_bias_random_walk_std_rad_sqrt_s=0.02,
        )
        ekf.update_wheel(0.0)
        ekf.update_gyro(0.0, 0.0)
        ekf.update_gyro(0.0, 1.0)
        for _ in range(10):
            ekf.update_wheel(0.2)

        self.assertEqual(len(ekf.state), 5)
        self.assertNotEqual(ekf.wheel_yaw_bias_estimate, 0.0)
        self.assertGreaterEqual(ekf.covariance[4][4], 0.0)

    def test_pose_ekf_fixed_gyro_bias_is_not_changed_by_wheel_yaw(self) -> None:
        ekf = PoseEKF(
            gyro_bias_mode="fixed",
            initial_gyro_bias_rad_s=0.01,
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
            wheel_yaw_noise_std_rad=0.02,
        )
        ekf.update_wheel(0.0)
        ekf.update_gyro(0.4, 0.0)
        ekf.update_gyro(0.4, 1.0)
        ekf.update_wheel(0.0)

        self.assertAlmostEqual(ekf.bias_estimate, 0.01, places=12)
        self.assertTrue(all(abs(row[3]) <= 1.1e-12 for row in ekf.covariance))
        self.assertTrue(
            all(abs(ekf.covariance[3][column]) <= 1.1e-12 for column in range(5))
        )

    def test_pose_ekf_slip_uncertainty_increases_position_covariance(self) -> None:
        certain_slip = PoseEKF(
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
            wheel_slip_ratio=0.10,
            wheel_slip_noise_std=0.0,
        )
        uncertain_slip = PoseEKF(
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
            wheel_slip_ratio=0.10,
            wheel_slip_noise_std=0.10,
        )
        for ekf in (certain_slip, uncertain_slip):
            ekf.update_wheel(math.pi / 4.0, linear_velocity_x=1.0)
            ekf.update_gyro(0.0, 0.0)
            ekf.update_gyro(0.0, 1.0)

        self.assertGreater(
            uncertain_slip.covariance[0][0], certain_slip.covariance[0][0]
        )
        self.assertGreater(
            uncertain_slip.covariance[1][1], certain_slip.covariance[1][1]
        )

    def test_pose_ekf_bias_position_jacobian_has_correct_x_sign(self) -> None:
        ekf = PoseEKF(
            gyro_rate_noise_std_rad_s=0.0,
            wheel_speed_noise_std_m_s=0.0,
            gyro_bias_random_walk_std_rad_s2=0.0,
        )
        ekf.update_wheel(math.pi / 4.0, linear_velocity_x=1.0)
        ekf.update_gyro(0.0, 0.0)
        ekf.update_gyro(0.0, 1.0)

        # Increasing gyro bias decreases the midpoint heading.  At a positive
        # heading this makes x increase, so the x/bias covariance is positive.
        self.assertGreater(ekf.covariance[0][3], 0.0)

    def test_pose_ekf_rejects_large_yaw_innovation_with_nis_gate(self) -> None:
        ekf = PoseEKF(nis_gate_threshold=9.0)
        ekf.update_wheel(0.0)
        ekf.update_gyro(0.0, 0.0)
        ekf.update_gyro(0.0, 1.0)
        covariance_before = [row[:] for row in ekf.covariance]
        ekf.update_wheel(math.pi)

        self.assertFalse(ekf.last_measurement_accepted)
        self.assertEqual(ekf.last_gain, 0.0)
        self.assertEqual(ekf.covariance, covariance_before)

    def test_pose_ekf_fuses_external_position_with_reported_covariance(self) -> None:
        ekf = PoseEKF(
            initial_position_variance_m2=1.0,
            initial_heading_variance_rad2=0.25,
            initial_bias_variance_rad2_s2=1.0,
            nis_gate_threshold=9.0,
        )
        ekf.update_wheel(0.0, x=0.0, y=0.0)
        covariance_before = [row[:] for row in ekf.covariance]

        accepted = ekf.update_external_position(
            1.0,
            -2.0,
            [[0.04, 0.0], [0.0, 0.04]],
        )

        self.assertTrue(accepted)
        self.assertTrue(ekf.last_external_measurement_accepted)
        self.assertGreater(ekf.last_external_nis, 0.0)
        self.assertGreater(ekf.pose[0], 0.9)
        self.assertLess(ekf.pose[1], -1.8)
        self.assertLess(ekf.covariance[0][0], covariance_before[0][0])
        self.assertLess(ekf.covariance[1][1], covariance_before[1][1])

    def test_pose_ekf_external_position_gate_rejects_inconsistent_candidate(self) -> None:
        ekf = PoseEKF(
            initial_position_variance_m2=0.01,
            initial_heading_variance_rad2=0.25,
            nis_gate_threshold=9.0,
        )
        ekf.update_wheel(0.0, x=0.0, y=0.0)
        state_before = ekf.state[:]
        covariance_before = [row[:] for row in ekf.covariance]

        accepted = ekf.update_external_position(
            1.0,
            1.0,
            [[0.001, 0.0], [0.0, 0.001]],
        )

        self.assertFalse(accepted)
        self.assertFalse(ekf.last_external_measurement_accepted)
        self.assertGreater(ekf.last_external_nis, 9.0)
        self.assertEqual(ekf.state, state_before)
        self.assertEqual(ekf.covariance, covariance_before)

    def test_pose_ekf_external_position_uses_cross_covariance_to_update_bias(self) -> None:
        ekf = PoseEKF(
            initial_position_variance_m2=1.0,
            initial_bias_variance_rad2_s2=1.0,
            nis_gate_threshold=100.0,
        )
        ekf.update_wheel(0.0, x=0.0, y=0.0)
        ekf.covariance[0][3] = 0.2
        ekf.covariance[3][0] = 0.2

        accepted = ekf.update_external_position(
            0.5,
            0.0,
            [[0.04, 0.0], [0.0, 0.04]],
        )

        self.assertTrue(accepted)
        self.assertNotEqual(ekf.bias_estimate, 0.0)

    def test_pose_ekf_replays_delayed_external_position_at_measurement_time(self) -> None:
        def build_filter() -> PoseEKF:
            ekf = PoseEKF(
                initial_position_variance_m2=1.0,
                initial_heading_variance_rad2=0.25,
                initial_bias_variance_rad2_s2=1.0,
                nis_gate_threshold=9.0,
                gyro_rate_noise_std_rad_s=0.0,
                wheel_speed_noise_std_m_s=0.0,
            )
            ekf.update_wheel(0.0, x=0.0, y=0.0, linear_velocity_x=1.0, stamp=0.0)
            ekf.update_gyro(0.0, 0.0)
            ekf.update_gyro(0.0, 0.5)
            return ekf

        on_time = build_filter()
        self.assertTrue(
            on_time.update_external_position(
                0.55,
                0.0,
                [[0.01, 0.0], [0.0, 0.01]],
                measurement_stamp=0.5,
            )
        )
        on_time.update_gyro(0.0, 1.0)

        delayed = build_filter()
        delayed.update_gyro(0.0, 1.0)
        self.assertTrue(
            delayed.update_external_position(
                0.55,
                0.0,
                [[0.01, 0.0], [0.0, 0.01]],
                measurement_stamp=0.5,
            )
        )

        self.assertAlmostEqual(delayed.pose[0], on_time.pose[0], places=9)
        self.assertAlmostEqual(delayed.pose[1], on_time.pose[1], places=9)
        self.assertAlmostEqual(delayed.pose[2], on_time.pose[2], places=9)
        self.assertGreater(delayed.last_external_measurement_age_s, 0.49)
        self.assertTrue(delayed.last_external_measurement_replayed)

    def test_pose_ekf_external_position_rejects_nonfinite_covariance(self) -> None:
        ekf = PoseEKF()
        ekf.update_wheel(0.0)
        state_before = ekf.state[:]

        accepted = ekf.update_external_position(
            0.0,
            0.0,
            [[float("nan"), 0.0], [0.0, 1.0]],
        )

        self.assertFalse(accepted)
        self.assertEqual(ekf.state, state_before)


if __name__ == "__main__":
    unittest.main()
