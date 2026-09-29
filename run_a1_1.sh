#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RADIUS=""
RADIUS_SET=0
OPEN_RVIZ=1
START=(12.0 4.0 2.0)
GOAL=(16.0 4.0 2.0)
START_SET=0
GOAL_SET=0

usage() {
  printf 'Usage: %s [--safety-radius METERS] [--inflation-radius METERS] [--start X Y Z] [--goal X Y Z] [--no-rviz]\n' "$0"
  printf '       --radius is retained as an alias; radius options set the same 3D inflation parameter.\n'
}

while (($#)); do
  case "$1" in
    --radius|--safety-radius|--inflation-radius)
      if (($# < 2)); then usage >&2; exit 2; fi
      RADIUS="$2"
      RADIUS_SET=1
      shift 2
      ;;
    --start)
      if (($# < 4)); then usage >&2; exit 2; fi
      START=("$2" "$3" "$4")
      START_SET=1
      shift 4
      ;;
    --goal)
      if (($# < 4)); then usage >&2; exit 2; fi
      GOAL=("$2" "$3" "$4")
      GOAL_SET=1
      shift 4
      ;;
    --no-rviz)
      OPEN_RVIZ=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ((RADIUS_SET)) && ! [[ "$RADIUS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'Invalid radius: %s\n' "$RADIUS" >&2
  exit 2
fi
if ((START_SET != GOAL_SET)); then
  printf 'Specify both --start and --goal, or neither.\n' >&2
  exit 2
fi
COORD_RE='^-?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$'
for coordinate in "${START[@]}" "${GOAL[@]}"; do
  if ! [[ "$coordinate" =~ $COORD_RE ]]; then
    printf 'Invalid waypoint coordinate: %s\n' "$coordinate" >&2
    exit 2
  fi
done

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'ROS setup not found: %s\n' "$ROS_SETUP" >&2
  exit 1
fi
set +u
source "$ROS_SETUP"
set -u

cd "$ROOT_DIR"
colcon build --packages-select lrs_psenak_valko
set +u
source "$ROOT_DIR/install/setup.bash"
set -u

PCD_OUTPUT="$ROOT_DIR/map_voxels_with_racks_inflated.pcd"
RVIZ_PID=""
NODE_PID=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [[ -n "$NODE_PID" ]] && kill -0 "$NODE_PID" 2>/dev/null; then
    kill "$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ROS_ARGS=(--ros-args --params-file "$ROOT_DIR/src/lrs_psenak_valko/config/planner.yaml"
  -p "occupancy_pcd_path:=$PCD_OUTPUT")
if ((RADIUS_SET)); then
  ROS_ARGS+=(-p "safety_radius:=$RADIUS")
fi
ros2 run lrs_psenak_valko map_planner_node "${ROS_ARGS[@]}" &
NODE_PID=$!

if ((OPEN_RVIZ)); then
  rviz2 -d "$ROOT_DIR/src/lrs_psenak_valko/config/planner.rviz" &
  RVIZ_PID=$!
fi

READY=0
for _ in {1..100}; do
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    wait "$NODE_PID"
    exit $?
  fi
  if ros2 node list 2>/dev/null | grep -qx '/map_planner_node'; then
    READY=1
    break
  fi
  sleep 0.2
done
if ((!READY)); then
  printf 'Planner node did not become ready within 20 seconds.\n' >&2
  exit 1
fi

ros2 topic pub --once /plan_request geometry_msgs/msg/PoseArray \
  "{header: {frame_id: map}, poses: [{position: {x: ${START[0]}, y: ${START[1]}, z: ${START[2]}}, orientation: {w: 1.0}}, {position: {x: ${GOAL[0]}, y: ${GOAL[1]}, z: ${GOAL[2]}}, orientation: {w: 1.0}}]}"

printf '\nFinal inflated voxel PCD will be saved to:\n  %s\n' "$PCD_OUTPUT"
printf 'Planner topics: /plan_request, /planned_path, /planning_status\n'
printf 'Press Ctrl+C to stop the planner and RViz.\n\n'

wait "$NODE_PID"