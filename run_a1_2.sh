#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MISSION_FILE="${1:-$ROOT_DIR/src/lrs_psenak_valko/missions/hangar_example.csv}"
READY_TIMEOUT="${A1_2_READY_TIMEOUT:-120}"
READY_DEADLINE=0
PLANNER_PID=""
EXECUTOR_PID=""

usage() {
  printf 'Usage: %s [MISSION.csv]\n' "$0"
  printf '       Default mission: src/lrs_psenak_valko/missions/hangar_example.csv\n'
  printf '       Start Gazebo, SITL, and MAVROS manually before running this helper.\n'
}

if (($# > 1)); then
  usage >&2
  exit 2
fi
if [[ "${MISSION_FILE}" == "-h" || "${MISSION_FILE}" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "${MISSION_FILE}" != /* ]]; then
  MISSION_FILE="$PWD/$MISSION_FILE"
fi
if [[ ! -f "$MISSION_FILE" ]]; then
  printf 'Mission file not found: %s\n' "$MISSION_FILE" >&2
  exit 2
fi
if ! [[ "$READY_TIMEOUT" =~ ^[0-9]+$ ]] || ((READY_TIMEOUT < 1)); then
  printf 'A1_2_READY_TIMEOUT must be a positive integer number of seconds.\n' >&2
  exit 2
fi

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

cleanup() {
  trap - EXIT INT TERM
  for pid in "$EXECUTOR_PID" "$PLANNER_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

PCD_OUTPUT="$ROOT_DIR/map_voxels_with_racks_inflated.pcd"
setsid ros2 run lrs_psenak_valko map_planner_node --ros-args \
  --params-file "$ROOT_DIR/src/lrs_psenak_valko/config/planner.yaml" \
  -p "occupancy_pcd_path:=$PCD_OUTPUT" &
PLANNER_PID=$!

printf 'Waiting for map planner and MAVROS (timeout: %s s)...\n' "$READY_TIMEOUT"
READY=0
READY_DEADLINE=$((SECONDS + READY_TIMEOUT))
LAST_WAIT_REPORT=$SECONDS
while ((SECONDS < READY_DEADLINE)); do
  if ! kill -0 "$PLANNER_PID" 2>/dev/null; then
    wait "$PLANNER_PID"
    printf 'Map planner exited before becoming ready.\n' >&2
    exit 1
  fi

  PLANNER_READY=0
  if ros2 node list 2>/dev/null | grep -qx '/map_planner_node'; then
    PLANNER_READY=1
  fi

  MAVROS_READY=1
  SERVICES_READY=1
  MISSING_SERVICES=()
  SERVICE_LIST="$(ros2 service list 2>/dev/null || true)"
  for service in /mavros/set_mode /mavros/cmd/arming /mavros/cmd/takeoff /mavros/cmd/land; do
    if ! grep -qx "$service" <<< "$SERVICE_LIST"; then
      MAVROS_READY=0
      SERVICES_READY=0
      MISSING_SERVICES+=("$service")
    fi
  done

  FCU_STATE="$(timeout 5s ros2 topic echo --once /mavros/state 2>/dev/null || true)"
  FCU_CONNECTED=0
  POSE_READY=0
  if [[ "$FCU_STATE" != *"connected: true"* ]]; then
    MAVROS_READY=0
  else
    FCU_CONNECTED=1
    POSE_READY=0
    POSE_SAMPLE="$(timeout 5s ros2 topic echo --once /mavros/local_position/pose 2>/dev/null || true)"
    if [[ "$POSE_SAMPLE" != *"position:"* ]]; then
      MAVROS_READY=0
    else
      POSE_READY=1
    fi
  fi

  if ((PLANNER_READY && MAVROS_READY)); then
    READY=1
    break
  fi
  if ((SECONDS - LAST_WAIT_REPORT >= 10)); then
    printf 'Still waiting: planner=%s, mavros_services=%s, fcu_connected=%s, local_pose=%s' \
      "$PLANNER_READY" "$SERVICES_READY" "$FCU_CONNECTED" "$POSE_READY"
    if ((${#MISSING_SERVICES[@]})); then
      printf ', missing_services=%s' "${MISSING_SERVICES[*]}"
    fi
    printf '\n'
    LAST_WAIT_REPORT=$SECONDS
  fi
  sleep 1
done

if ((!READY)); then
  printf 'Planner/MAVROS did not become ready within %s s. Check Gazebo, SITL, MAVROS and the planner terminals.\n' \
    "$READY_TIMEOUT" >&2
  exit 1
fi

printf 'Planner ready; MAVROS connected with services and local pose available.\n'
printf 'Starting mission: %s\n' "$MISSION_FILE"
setsid ros2 run lrs_psenak_valko mission_executor_node --ros-args \
  -p "mission_file:=$MISSION_FILE" &
EXECUTOR_PID=$!

printf 'Mission active. Leave Gazebo and SITL visible; press Ctrl+C here to stop the executor and planner.\n'
wait "$EXECUTOR_PID"