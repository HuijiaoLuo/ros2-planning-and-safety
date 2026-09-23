"""Shared map and Gazebo-scene profiles for controlled experiments.

The profile is deliberately dependency-free.  The static-map publisher uses
it to rasterize the occupancy grid, while the simulation launch file uses the
same rectangles to render the Gazebo obstacle models.  Keeping both views in
one module prevents a localization experiment from silently comparing one
map against a different physical world.
"""

from __future__ import annotations

from dataclasses import dataclass
import re


MAP_WIDTH = 60
MAP_HEIGHT = 60
MAP_RESOLUTION_M = 0.1
MAP_ORIGIN_X_M = -1.0
MAP_ORIGIN_Y_M = -3.0
START_X_M = 0.0
START_Y_M = 0.0
GOAL_X_M = 2.0
GOAL_Y_M = 0.0


@dataclass(frozen=True)
class RectangleObstacle:
    """One axis-aligned static box shared by the map and Gazebo world."""

    name: str
    center_x_m: float
    center_y_m: float
    size_x_m: float
    size_y_m: float
    height_m: float = 0.5

    @property
    def x_min_m(self) -> float:
        return self.center_x_m - 0.5 * self.size_x_m

    @property
    def x_max_m(self) -> float:
        return self.center_x_m + 0.5 * self.size_x_m

    @property
    def y_min_m(self) -> float:
        return self.center_y_m - 0.5 * self.size_y_m

    @property
    def y_max_m(self) -> float:
        return self.center_y_m + 0.5 * self.size_y_m


@dataclass(frozen=True)
class ScenarioProfile:
    """Static geometry and metadata for one reproducible navigation scene."""

    name: str
    description: str
    obstacles: tuple[RectangleObstacle, ...]
    start_x_m: float = START_X_M
    start_y_m: float = START_Y_M
    goal_x_m: float = GOAL_X_M
    goal_y_m: float = GOAL_Y_M


SCENARIOS: dict[str, ScenarioProfile] = {
    "baseline_obstacle": ScenarioProfile(
        name="baseline_obstacle",
        description="Single obstacle from the frozen planning and safety baseline.",
        obstacles=(
            RectangleObstacle(
                name="obstacle_box",
                center_x_m=1.0,
                center_y_m=0.0,
                size_x_m=0.30,
                size_y_m=1.00,
            ),
        ),
    ),
    "l_corridor": ScenarioProfile(
        name="l_corridor",
        description="Perpendicular walls that force an upper detour around the obstacle.",
        obstacles=(
            RectangleObstacle(
                name="l_vertical_wall",
                center_x_m=1.0,
                center_y_m=0.15,
                size_x_m=0.30,
                size_y_m=1.30,
            ),
            RectangleObstacle(
                name="l_horizontal_wall",
                center_x_m=1.65,
                center_y_m=0.95,
                size_x_m=1.60,
                size_y_m=0.30,
            ),
        ),
    ),
    "symmetric_corridor": ScenarioProfile(
        name="symmetric_corridor",
        description="Parallel walls with repeated range structure along the route.",
        obstacles=(
            RectangleObstacle(
                name="corridor_lower_wall",
                center_x_m=1.50,
                center_y_m=-0.85,
                size_x_m=2.20,
                size_y_m=0.25,
            ),
            RectangleObstacle(
                name="corridor_upper_wall",
                center_x_m=1.50,
                center_y_m=0.85,
                size_x_m=2.20,
                size_y_m=0.25,
            ),
        ),
    ),
}


def get_scenario_profile(name: str) -> ScenarioProfile:
    """Return a named profile or raise a useful configuration error."""

    try:
        return SCENARIOS[str(name)]
    except KeyError as error:
        supported = ", ".join(sorted(SCENARIOS))
        raise ValueError(
            f"unknown scenario {name!r}; expected one of: {supported}"
        ) from error


def build_occupancy_data(
    profile: ScenarioProfile,
    *,
    width: int = MAP_WIDTH,
    height: int = MAP_HEIGHT,
    resolution_m: float = MAP_RESOLUTION_M,
    origin_x_m: float = MAP_ORIGIN_X_M,
    origin_y_m: float = MAP_ORIGIN_Y_M,
) -> list[int]:
    """Rasterize a profile into ROS occupancy-grid data."""

    if width <= 0 or height <= 0 or resolution_m <= 0.0:
        raise ValueError("map width, height, and resolution must be positive")

    data = [0] * (width * height)

    for row in range(height):
        y = origin_y_m + (row + 0.5) * resolution_m
        for column in range(width):
            x = origin_x_m + (column + 0.5) * resolution_m
            if any(
                obstacle.x_min_m <= x <= obstacle.x_max_m
                and obstacle.y_min_m <= y <= obstacle.y_max_m
                for obstacle in profile.obstacles
            ):
                data[row * width + column] = 100

    # Keep the outer boundary occupied so the planner cannot leave the map.
    for column in range(width):
        data[column] = 100
        data[(height - 1) * width + column] = 100
    for row in range(height):
        data[row * width] = 100
        data[row * width + width - 1] = 100

    return data


def render_obstacle_models(profile: ScenarioProfile) -> str:
    """Render profile obstacles as static SDF box models."""

    models: list[str] = []
    for index, obstacle in enumerate(profile.obstacles):
        model_name = f"scenario_obstacle_{index}_{obstacle.name}"
        center_z_m = 0.5 * obstacle.height_m
        models.append(
            f'''    <model name="{model_name}">
      <static>true</static>
      <pose>{obstacle.center_x_m:g} {obstacle.center_y_m:g} {center_z_m:g} 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{obstacle.size_x_m:g} {obstacle.size_y_m:g} {obstacle.height_m:g}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{obstacle.size_x_m:g} {obstacle.size_y_m:g} {obstacle.height_m:g}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.75 0.20 0.10 1</ambient>
            <diffuse>0.75 0.20 0.10 1</diffuse>
          </material>
        </visual>
      </link>
    </model>'''
        )
    return "\n\n".join(models)


_BASELINE_OBSTACLE_MODEL = re.compile(
    r"\n    <model name=\"obstacle_box\">.*?\n    </model>\n",
    flags=re.DOTALL,
)


def render_world(template: str, profile: ScenarioProfile) -> str:
    """Replace the baseline obstacle block with a selected profile."""

    replacement = "\n" + render_obstacle_models(profile) + "\n"
    rendered, replacements = _BASELINE_OBSTACLE_MODEL.subn(
        replacement,
        template,
        count=1,
    )
    if replacements != 1:
        raise ValueError("Gazebo world template does not contain the baseline obstacle block")
    return rendered
