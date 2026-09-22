"""Pure-Python pose-error statistics for the V3 estimator diagnostics."""

from __future__ import annotations

import math


def wrap_angle(angle: float) -> float:
    """Return an angle in the shortest [-pi, pi] representation."""
    return math.atan2(math.sin(angle), math.cos(angle))


def mahalanobis_squared_2d(
    error_x: float,
    error_y: float,
    covariance_xx: float,
    covariance_xy: float,
    covariance_yy: float,
) -> float | None:
    """Return the squared 2-D normalized error, or ``None`` if invalid.

    The covariance entries describe the same world-frame x/y coordinates as
    the error.  This is used for offline calibration only: a value near 2 is
    the expected mean for a well-calibrated two-dimensional Gaussian, and the
    95-percent chi-square threshold is approximately 5.991.
    """
    determinant = covariance_xx * covariance_yy - covariance_xy * covariance_xy
    if not all(
        math.isfinite(value)
        for value in (
            error_x,
            error_y,
            covariance_xx,
            covariance_xy,
            covariance_yy,
        )
    ) or determinant <= 0.0:
        return None
    numerator = (
        covariance_yy * error_x * error_x
        - 2.0 * covariance_xy * error_x * error_y
        + covariance_xx * error_y * error_y
    )
    value = numerator / determinant
    return value if value >= 0.0 and math.isfinite(value) else None


def normalized_squared_error(error: float, variance: float) -> float | None:
    """Return ``error**2 / variance`` when the variance is physically valid."""
    if not math.isfinite(error) or not math.isfinite(variance) or variance <= 0.0:
        return None
    value = error * error / variance
    return value if math.isfinite(value) else None


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
        # Heading errors must be circular: +pi and -pi describe the same
        # orientation, so a direct subtraction would create a false 2*pi error.
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
        # Store sums of squares rather than every sample.  This keeps the
        # logger bounded in memory while preserving exact batch RMSE values.
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
        # Bias is the signed mean error; RMSE is the square root of the mean
        # squared error.  They answer different questions and are both useful
        # for diagnosing systematic drift.
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
