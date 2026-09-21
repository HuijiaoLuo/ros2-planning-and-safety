"""Pure-Python pose-error statistics for the V3 estimator diagnostics."""

from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """Return an angle in the shortest [-pi, pi] representation."""
    return math.atan2(math.sin(angle), math.cos(angle))


class PoseErrorStats:
    """Accumulate position and heading errors without storing every sample."""

    def __init__(self) -> None:
        self.count = 0
        self._sum_x = 0.0
        self._sum_y = 0.0
        self._sum_heading = 0.0
        self._sum_x_sq = 0.0
        self._sum_y_sq = 0.0
        self._sum_position_sq = 0.0
        self._sum_heading_sq = 0.0
        self.last_position_error = math.nan
        self.last_heading_error = math.nan

    def add(
        self,
        truth_x: float,
        truth_y: float,
        truth_heading: float,
        estimate_x: float,
        estimate_y: float,
        estimate_heading: float,
    ) -> None:
        error_x = estimate_x - truth_x
        error_y = estimate_y - truth_y
        error_heading = wrap_angle(estimate_heading - truth_heading)
        position_squared = error_x * error_x + error_y * error_y

        self.count += 1
        self._sum_x += error_x
        self._sum_y += error_y
        self._sum_heading += error_heading
        self._sum_x_sq += error_x * error_x
        self._sum_y_sq += error_y * error_y
        self._sum_position_sq += position_squared
        self._sum_heading_sq += error_heading * error_heading
        self.last_position_error = math.sqrt(position_squared)
        self.last_heading_error = error_heading

    def summary(self, prefix: str) -> dict[str, float | int | None]:
        """Return CSV-friendly aggregate metrics with a consistent prefix."""
        if self.count == 0:
            return {
                f"{prefix}_samples": 0,
                f"{prefix}_x_rmse_m": None,
                f"{prefix}_y_rmse_m": None,
                f"{prefix}_position_rmse_m": None,
                f"{prefix}_heading_rmse_rad": None,
                f"{prefix}_x_bias_m": None,
                f"{prefix}_y_bias_m": None,
                f"{prefix}_heading_bias_rad": None,
                f"{prefix}_final_position_error_m": None,
                f"{prefix}_final_heading_error_rad": None,
            }

        count = float(self.count)
        return {
            f"{prefix}_samples": self.count,
            f"{prefix}_x_rmse_m": math.sqrt(self._sum_x_sq / count),
            f"{prefix}_y_rmse_m": math.sqrt(self._sum_y_sq / count),
            f"{prefix}_position_rmse_m": math.sqrt(self._sum_position_sq / count),
            f"{prefix}_heading_rmse_rad": math.sqrt(self._sum_heading_sq / count),
            f"{prefix}_x_bias_m": self._sum_x / count,
            f"{prefix}_y_bias_m": self._sum_y / count,
            f"{prefix}_heading_bias_rad": self._sum_heading / count,
            f"{prefix}_final_position_error_m": self.last_position_error,
            f"{prefix}_final_heading_error_rad": self.last_heading_error,
        }
