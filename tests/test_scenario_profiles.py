import sys
from collections import deque
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1] / "ros2_ws" / "src" / "robotics_nav"
sys.path.insert(0, str(PACKAGE_ROOT))

from robotics_nav.scenario_profiles import (  # noqa: E402
    MAP_HEIGHT,
    MAP_ORIGIN_X_M,
    MAP_ORIGIN_Y_M,
    MAP_RESOLUTION_M,
    MAP_WIDTH,
    SCENARIOS,
    build_occupancy_data,
    render_world,
)


WORLD_TEMPLATE = (
    Path(__file__).parents[1]
    / "ros2_ws"
    / "src"
    / "robotics_sim"
    / "worlds"
    / "differential_drive.sdf"
)


def world_to_cell(x_m: float, y_m: float) -> tuple[int, int]:
    return (
        int((x_m - MAP_ORIGIN_X_M) / MAP_RESOLUTION_M),
        int((y_m - MAP_ORIGIN_Y_M) / MAP_RESOLUTION_M),
    )


def has_four_connected_path(data: list[int], start: tuple[int, int], goal: tuple[int, int]) -> bool:
    """Check raw-map reachability before planner obstacle inflation."""

    def is_free(cell: tuple[int, int]) -> bool:
        x, y = cell
        return (
            0 <= x < MAP_WIDTH
            and 0 <= y < MAP_HEIGHT
            and data[y * MAP_WIDTH + x] < 50
        )

    if not is_free(start) or not is_free(goal):
        return False

    queue = deque([start])
    visited = {start}
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True
        for neighbor in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if neighbor not in visited and is_free(neighbor):
                visited.add(neighbor)
                queue.append(neighbor)
    return False


class ScenarioProfileTests(unittest.TestCase):
    def test_all_profiles_keep_start_and_goal_reachable(self) -> None:
        for profile in SCENARIOS.values():
            with self.subTest(scenario=profile.name):
                data = build_occupancy_data(profile)
                start = world_to_cell(profile.start_x_m, profile.start_y_m)
                goal = world_to_cell(profile.goal_x_m, profile.goal_y_m)
                self.assertTrue(has_four_connected_path(data, start, goal))

    def test_profiles_have_distinct_obstacle_layouts(self) -> None:
        layouts = {
            tuple(
                (
                    obstacle.center_x_m,
                    obstacle.center_y_m,
                    obstacle.size_x_m,
                    obstacle.size_y_m,
                )
                for obstacle in profile.obstacles
            )
            for profile in SCENARIOS.values()
        }
        self.assertEqual(len(layouts), len(SCENARIOS))

    def test_rendered_world_is_valid_sdf_for_each_profile(self) -> None:
        template = WORLD_TEMPLATE.read_text(encoding="utf-8")
        for profile in SCENARIOS.values():
            with self.subTest(scenario=profile.name):
                rendered = render_world(template, profile)
                ET.fromstring(rendered)
                self.assertEqual(
                    rendered.count("<model name=\"scenario_obstacle_"),
                    len(profile.obstacles),
                )


if __name__ == "__main__":
    unittest.main()
