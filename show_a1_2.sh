#!/usr/bin/env bash
# Opens the plots (top view, overview) and the animated 3D view of one A1.2 run.
#
# Usage: ./show_a1_2.sh [RUN_FOLDER]
#   no argument -> the recording in results/A1_2/ flown with the current safety_radius
#   RUN_FOLDER  -> e.g. results/A1_2/safety_0.45_20261018 or log/a1_2_runs/<run>
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RADIUS="$(awk '/safety_radius:/ { printf "%.2f", $2 }' "$ROOT_DIR/src/lrs_psenak_valko/config/planner.yaml")"
RUN_DIR="${1:-$(ls -d "$ROOT_DIR/results/A1_2/safety_${RADIUS}_"*/ 2>/dev/null | sort | tail -n 1)}"
# A relative folder may be given from anywhere: try it next to this script too.
if [[ ! -d "$RUN_DIR" && -d "$ROOT_DIR/$RUN_DIR" ]]; then RUN_DIR="$ROOT_DIR/$RUN_DIR"; fi
if [[ -z "$RUN_DIR" || ! -f "$RUN_DIR/flight_log.csv" ]]; then
  printf 'No run folder with flight_log.csv: %s\n' "${RUN_DIR:-<none>}" >&2
  exit 1
fi

# The 3D view runs in the background so both windows are open at the same time.
python3 "$ROOT_DIR/view_3d_a1_2.py" "$RUN_DIR" > /dev/null 2>&1 &
python3 "$ROOT_DIR/plot_a1_2.py" "$RUN_DIR" --no-save
wait
