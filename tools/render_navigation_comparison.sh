#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-docs/assets}"
original_dir="${2:-results/localization_matrix}"
validated_dir="${3:-results/estimator_validation_ekf_fixedgyro}"

python3 tools/render_baseline_gif.py \
  --trace "${original_dir}/baseline_obstacle__v4__seed_000_trace.csv" \
  --plan "${original_dir}/baseline_obstacle__v4__seed_000_plan.csv" \
  --map "${original_dir}/baseline_obstacle__v4__seed_000_map.csv" \
  --output "${output_dir}/baseline_obstacle_navigation.gif" \
  --title "Original fixed fusion | baseline obstacle" \
  --map-context-m 1.2

python3 tools/render_baseline_gif.py \
  --trace "${validated_dir}/baseline_obstacle__v4__seed_000_trace.csv" \
  --plan "${validated_dir}/baseline_obstacle__v4__seed_000_plan.csv" \
  --map "${validated_dir}/baseline_obstacle__v4__seed_000_map.csv" \
  --output "${output_dir}/baseline_obstacle_ekf_navigation.gif" \
  --title "Validated EKF | baseline obstacle" \
  --map-context-m 1.2

python3 tools/render_baseline_gif.py \
  --trace "${original_dir}/l_corridor__v4__seed_000_trace.csv" \
  --plan "${original_dir}/l_corridor__v4__seed_000_plan.csv" \
  --map "${original_dir}/l_corridor__v4__seed_000_map.csv" \
  --output "${output_dir}/l_corridor_navigation.gif" \
  --title "Original fixed fusion | L corridor" \
  --map-context-m 1.2

python3 tools/render_baseline_gif.py \
  --trace "${validated_dir}/l_corridor__v4__seed_000_trace.csv" \
  --plan "${validated_dir}/l_corridor__v4__seed_000_plan.csv" \
  --map "${validated_dir}/l_corridor__v4__seed_000_map.csv" \
  --output "${output_dir}/l_corridor_ekf_navigation.gif" \
  --title "Validated EKF | L corridor" \
  --map-context-m 1.2

echo "Rendered four navigation comparison GIFs into ${output_dir}"
