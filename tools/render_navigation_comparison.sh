#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-docs/assets}"
original_dir="${2:-results/localization_matrix}"
validated_dir="${3:-results/estimator_validation_ekf_fixedgyro}"

python_bin="${PYTHON_BIN:-}"
if [[ -z "$python_bin" ]]; then
  for candidate in python python3; do
    if command -v "$candidate" >/dev/null 2>&1 && \
       "$candidate" -c 'import matplotlib, PIL' >/dev/null 2>&1; then
      python_bin="$candidate"
      break
    fi
  done
fi

render_mode="python"
if [[ -z "$python_bin" ]]; then
  render_mode="powershell"
  if command -v powershell.exe >/dev/null 2>&1; then
    powershell_bin="powershell.exe"
  elif command -v pwsh >/dev/null 2>&1; then
    powershell_bin="pwsh"
  else
    echo "No usable renderer found." >&2
    echo "Activate the conda environment or set PYTHON_BIN to a Python with matplotlib and Pillow." >&2
    exit 2
  fi
  echo "Python plotting dependencies were not found; using ${powershell_bin} fallback renderer."
else
  echo "Using ${python_bin} for GIF rendering."
fi

render_one() {
  local source_dir="$1"
  local scenario="$2"
  local output_name="$3"
  local title="$4"
  local trace="${source_dir}/${scenario}__v4__seed_000_trace.csv"
  local plan="${source_dir}/${scenario}__v4__seed_000_plan.csv"
  local map="${source_dir}/${scenario}__v4__seed_000_map.csv"

  for input_file in "$trace" "$plan" "$map"; do
    if [[ ! -f "$input_file" ]]; then
      echo "Missing input: $input_file" >&2
      exit 3
    fi
  done

  if [[ "$render_mode" == "python" ]]; then
    "$python_bin" tools/render_baseline_gif.py \
      --trace "$trace" \
      --plan "$plan" \
      --map "$map" \
      --output "${output_dir}/${output_name}" \
      --title "$title" \
      --map-context-m 1.2
  else
    "$powershell_bin" -NoProfile -ExecutionPolicy Bypass -File tools/render_navigation_gif.ps1 \
      -Trace "$trace" \
      -Plan "$plan" \
      -MapPath "$map" \
      -Output "${output_dir}/${output_name}" \
      -Title "$title" \
      -MapContextM 1.2
  fi
}

render_one "${original_dir}" baseline_obstacle baseline_obstacle_navigation.gif \
  "Original fixed fusion | baseline obstacle"

render_one "${validated_dir}" baseline_obstacle baseline_obstacle_ekf_navigation.gif \
  "Validated EKF | baseline obstacle"

render_one "${original_dir}" l_corridor l_corridor_navigation.gif \
  "Original fixed fusion | L corridor"

render_one "${validated_dir}" l_corridor l_corridor_ekf_navigation.gif \
  "Validated EKF | L corridor"

echo "Rendered four navigation comparison GIFs into ${output_dir}"
