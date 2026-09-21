import math
import sys
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.heading_fusion import HeadingFusion, blend_angles, wrap_angle


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


if __name__ == "__main__":
    unittest.main()
