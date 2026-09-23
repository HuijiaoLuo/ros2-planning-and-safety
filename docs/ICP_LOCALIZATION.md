# Point-to-point Iterative Closest Point (ICP) localization baseline

This document describes the independent point-to-point Iterative Closest Point
(ICP) registration baseline implemented in `icp_localization.py` and exposed
by the `icp_localizer` ROS 2 node. It is a model comparison for the known-map
localization layer. It is not a SLAM system: the map is fixed, the initial
pose comes from wheel/IMU estimation, and the registration is local.

## Measurement model

For a valid LiDAR return with range `r_i` and bearing `alpha_i`, the point in
the LiDAR frame is

$$
\mathbf{p}_i^{\mathrm{lidar}} =
r_i
\begin{bmatrix}
\cos(\alpha_i)\\
\sin(\alpha_i)
\end{bmatrix}.
$$

For a candidate planar pose, write the state explicitly as

$$
\mathbf{x} =
\begin{bmatrix}
x\\
y\\
\theta
\end{bmatrix}.
$$

The corresponding map-frame endpoint is

$$
\mathbf{p}_i^{\mathrm{map}}(\mathbf{x}) =
\begin{bmatrix}
x\\
y
\end{bmatrix}
+ R(\theta)\mathbf{p}_i^{\mathrm{lidar}},
\qquad
R(\theta) =
\begin{bmatrix}
\cos(\theta) & -\sin(\theta)\\
\sin(\theta) & \cos(\theta)
\end{bmatrix}.
$$

The sign is positive because this is the forward transform from the LiDAR
frame to the map frame: first rotate the measured ray by the robot heading,
then translate it by the robot position `(x, y)`. A subtraction would describe
an inverse-frame transform and is not the endpoint model used by this node.

The current map representation is the set `Q` of centres of occupied grid
cells. For each transformed scan point, ICP selects the nearest map point
within the fixed correspondence radius:

$$
c(i) = \underset{\mathbf{q}\in Q}{\arg\min}
\left\|\mathbf{p}_i^{\mathrm{map}}(\mathbf{x})-\mathbf{q}\right\|_2.
$$

The registration objective for the current correspondence set is

$$
J(\mathbf{x}) =
\frac{1}{N}\sum_{i=1}^{N}
\left\|\mathbf{p}_i^{\mathrm{map}}(\mathbf{x})-c(i)\right\|_2^2.
$$

## Iterative update

The implementation alternates between two deterministic operations:

1. transform the scan endpoints using the current pose;
2. associate each endpoint with its nearest occupied-cell centre, then solve
   the two-dimensional rigid least-squares update from the source and target
   centroids.

The rotation increment is obtained from the centred point pairs using the
two-dimensional cross and dot sums:

$$
\delta\theta =
\mathrm{atan2}\left(
\sum_i(s_{ix}t_{iy}-s_{iy}t_{ix}),
\sum_i(s_{ix}t_{ix}+s_{iy}t_{iy})
\right).
$$

The translation increment aligns the rotated source centroid with the target
centroid. The pose is updated and the association/update cycle repeats until
the increment is small or the fixed iteration limit is reached.

The current baseline uses a brute-force nearest-neighbour search. This keeps
the implementation transparent and makes compute time measurable; it is not
intended to be the final performance implementation.

## ROS integration and timing

The node consumes:

```text
/state_prediction + /scan + /map
                  |
                  v
        point-to-point ICP worker
                  |
                  v
          /localized_estimate
```

The wheel/IMU-only `/state_prediction` is relayed at the control-rate
publication frequency. ICP runs at a separate lower rate in a worker process,
so registration does not block the wheel/IMU estimator or the controller. A
converged result produces one timestamped `/localization_candidate` event and
updates the persistent map-to-odom transform used for later relays. An
unfinished or non-converged registration leaves the previous transform
unchanged. The corrected localization stream is diagnostic in the current
architecture; it is not fed back into the high-rate control loop.

The JSONL diagnostic stream records the quantities needed to distinguish
registration failure from execution failure:

- scan and pose timestamps;
- worker compute time and match age;
- correspondence count and iteration count;
- mean and RMS point residual;
- candidate and applied corrections;
- convergence status and whether the candidate was published.

`tools/summarize_icp_diagnostics.py` aggregates these records without
discarding rejected attempts.

## Why this is not another parameter-tuning loop

The ICP run is intended as a controlled model comparison. The map, scan
subsampling, estimator, controller, safety policy, and goal logic remain the
same as the preceding localization experiment. The registration model changes
from a particle likelihood / ray-casting hypothesis to deterministic local
point registration. A result with zero accepted registrations is therefore
evidence that this measurement model is not observable or not computationally
adequate for the current map and scan geometry; it is not a reason to keep
searching over acceptance thresholds.

## Known limitations

- Occupied-cell centres are only a raster approximation of physical surfaces.
- Point-to-point residuals do not use wall normals, so long straight walls can
  be weakly constrained in translation along the wall.
- Nearest-neighbour association is local and can converge to a wrong map alias.
- The brute-force search scales with both scan points and occupied cells.
- The current baseline reports registration residuals but does not derive a
  statistically calibrated covariance from ICP curvature.
- Global relocalization is outside this node. A global method would need a
  particle/global hypothesis layer, scan context, or a full SLAM system.

These limitations are intentional: they make the failure mode of the
registration objective visible before introducing a larger SLAM stack.
