#!/usr/bin/env bash
# A1.2 helper: builds the package, (optionally) starts Gazebo -> SITL -> MAVROS in their own
# terminals with readiness gates between them, then starts the map planner and the mission executor.
#
# Usage: ./run_a1_2.sh [MISSION.csv]
# Environment (all optional):
#   A1_2_URK_DIR           repo with scripts/run_gazebo.sh, run_sitl.sh, run_mavros.sh   [$HOME/LRS-URK]
#   A1_2_START_SIM         1 = start missing Gazebo/SITL/MAVROS here, 0 = you started them [1]
#   A1_2_STOP_SIM          1 = kill Gazebo/SITL/MAVROS when this script exits             [0]
#   A1_2_TERMINAL          auto|gnome-terminal|xterm|x-terminal-emulator                  [auto]
#   A1_2_MIN_RTF           Gazebo real-time factor required before SITL starts            [0.8]
#   A1_2_RTF_TIMEOUT       seconds to wait for that real-time factor, then continue        [90]
#   A1_2_SITL_BOOT_TIMEOUT seconds for SITL to build/boot and show its first heartbeat     [300]
#   A1_2_READY_TIMEOUT     seconds for planner + MAVROS connection                         [180]
#   A1_2_POLL_INTERVAL     seconds between readiness polls                                 [5]
#   A1_2_SKIP_BUILD        1 = skip colcon build                                           [0]
#   A1_2_TIME_SCALE        multiply the executor's wall-clock timeouts (default: 1/RTF when RTF < MIN_RTF)
#   A1_2_LOG_DIR           where simulator logs go                                         [<ws>/log/a1_2_sim]
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MISSION_FILE="${1:-$ROOT_DIR/src/lrs_psenak_valko/missions/hangar_example.csv}"
URK_DIR="${A1_2_URK_DIR:-$HOME/LRS-URK}"
# Tolerate being pointed at the scripts folder itself.
if [[ ! -d "$URK_DIR/scripts" && -x "$URK_DIR/run_gazebo.sh" ]]; then URK_DIR="$(dirname "$URK_DIR")"; fi
START_SIM="${A1_2_START_SIM:-1}"
STOP_SIM="${A1_2_STOP_SIM:-0}"
TERMINAL="${A1_2_TERMINAL:-auto}"
MIN_RTF="${A1_2_MIN_RTF:-0.8}"
RTF_TIMEOUT="${A1_2_RTF_TIMEOUT:-90}"
SITL_BOOT_TIMEOUT="${A1_2_SITL_BOOT_TIMEOUT:-300}"
READY_TIMEOUT="${A1_2_READY_TIMEOUT:-180}"
POLL_INTERVAL="${A1_2_POLL_INTERVAL:-5}"
SKIP_BUILD="${A1_2_SKIP_BUILD:-0}"
LOG_DIR="${A1_2_LOG_DIR:-$ROOT_DIR/log/a1_2_sim}"
PLANNER_PID=""
EXECUTOR_PID=""

# BEGIN_FUNCS
log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { log "WARNING: $*" >&2; }
is_uint() { [[ "$1" =~ ^[0-9]+$ ]]; }
is_number() { [[ "$1" =~ ^[0-9]+([.][0-9]+)?$ ]]; }
float_ge() { awk -v a="$1" -v b="$2" 'BEGIN { exit !(a + 0 >= b + 0) }'; }

# First /world/<name>/stats topic published by a running Gazebo (empty if none).
gz_stats_topic() {
  timeout 6s gz topic -l 2>/dev/null | grep -m1 -E '^/world/[^/]+/stats$' || true
}
# One field (e.g. real_time_factor, paused) from the world statistics message.
gz_stat() {
  timeout 8s gz topic -e -t "$1" -n 1 2>/dev/null | awk -v key="$2:" '$1 == key { print $2; exit }' || true
}
sitl_running()   { pgrep -f 'bin/arducopter' >/dev/null 2>&1; }
mavros_running() { ros2 service list 2>/dev/null | grep -qx '/mavros/set_mode'; }

wait_for_log() { # file pattern timeout_s
  local deadline=$((SECONDS + $3))
  while ((SECONDS < deadline)); do
    if grep -aqE -- "$2" "$1" 2>/dev/null; then return 0; fi
    sleep 2
  done
  return 1
}

# Factor by which wall-clock timeouts must grow when Gazebo runs slower than real time.
time_scale() { # rtf
  awk -v r="$1" -v m="$MIN_RTF" 'BEGIN { if (r + 0 <= 0 || r + 0 >= m + 0) { print 1; exit }
    s = int(1 / r); if (s < 1 / r) s++; if (s > 10) s = 10; print s }'
}
scaled() { awk -v v="$1" -v s="$2" 'BEGIN { printf "%.1f", v * s }'; }
# END_FUNCS

usage() {
  printf 'Usage: %s [MISSION.csv]\n' "$0"
  printf '       Default mission: src/lrs_psenak_valko/missions/hangar_example.csv\n'
  printf '       Starts Gazebo, SITL and MAVROS (from A1_2_URK_DIR=%s) unless A1_2_START_SIM=0.\n' "$URK_DIR"
  printf '       See the header of this file for all A1_2_* options.\n'
}

if (($# > 1)); then usage >&2; exit 2; fi
if [[ "${MISSION_FILE}" == "-h" || "${MISSION_FILE}" == "--help" ]]; then usage; exit 0; fi
if [[ "${MISSION_FILE}" != /* ]]; then MISSION_FILE="$PWD/$MISSION_FILE"; fi
if [[ ! -f "$MISSION_FILE" ]]; then
  printf 'Mission file not found: %s\n' "$MISSION_FILE" >&2
  exit 2
fi
for name in RTF_TIMEOUT SITL_BOOT_TIMEOUT READY_TIMEOUT POLL_INTERVAL; do
  value="${!name}"
  if ! is_uint "$value" || ((value < 1)); then
    printf 'A1_2_%s must be a positive integer number of seconds.\n' "$name" >&2
    exit 2
  fi
done
if ! is_number "$MIN_RTF"; then printf 'A1_2_MIN_RTF must be a number, e.g. 0.8\n' >&2; exit 2; fi
for name in START_SIM STOP_SIM SKIP_BUILD; do
  if [[ "${!name}" != 0 && "${!name}" != 1 ]]; then
    printf 'A1_2_%s must be 0 or 1.\n' "$name" >&2
    exit 2
  fi
done
if [[ "$START_SIM" == 1 ]]; then
  for s in run_gazebo.sh run_sitl.sh run_mavros.sh; do
    if [[ ! -x "$URK_DIR/scripts/$s" ]]; then
      printf 'Missing %s/scripts/%s - set A1_2_URK_DIR or use A1_2_START_SIM=0.\n' "$URK_DIR" "$s" >&2
      exit 2
    fi
  done
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'ROS setup not found: %s\n' "$ROS_SETUP" >&2
  exit 1
fi
set +u
source "$ROS_SETUP"
set -u

# Build first, before the simulator exists: compiling during sim start-up slows Gazebo down, and a
# slow Gazebo is what makes ArduPilot report "Accels/Gyros inconsistent".
cd "$ROOT_DIR"
if [[ "$SKIP_BUILD" == 0 ]]; then
  colcon build --packages-select lrs_psenak_valko
elif [[ ! -x "$ROOT_DIR/install/lrs_psenak_valko/lib/lrs_psenak_valko/mission_executor_node" ]]; then
  printf 'Built mission_executor_node not found; run colcon build or unset A1_2_SKIP_BUILD.\n' >&2
  exit 1
else
  printf 'Skipping colcon build (A1_2_SKIP_BUILD=1).\n'
fi
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
  if [[ "$STOP_SIM" == 1 ]]; then
    log "Stopping MAVROS, SITL and Gazebo (A1_2_STOP_SIM=1)"
    pkill -f 'mavros_node' 2>/dev/null || true
    pkill -f 'sim_vehicle.py' 2>/dev/null || true
    pkill -f 'mavproxy' 2>/dev/null || true
    pkill -f 'bin/arducopter' 2>/dev/null || true
    pkill -f 'gz sim' 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- simulator start-up
pick_terminal() {
  local t="$TERMINAL"
  if [[ "$t" == auto ]]; then
    if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then t=none
    elif command -v gnome-terminal >/dev/null 2>&1; then t=gnome-terminal
    elif command -v xterm >/dev/null 2>&1; then t=xterm
    elif command -v x-terminal-emulator >/dev/null 2>&1; then t=x-terminal-emulator
    else t=none; fi
  fi
  printf '%s' "$t"
}

# launch_component NAME SCRIPT: runs $URK_DIR/scripts/SCRIPT in its own terminal window and copies
# its output to $LOG_DIR/NAME.log (through `script`, so the program still sees a real terminal).
launch_component() {
  local name="$1" script_name="$2" launcher="$LOG_DIR/launch_$1.sh" logfile="$LOG_DIR/$1.log" term run
  term="$(pick_terminal)"
  if [[ "$term" == none ]]; then
    printf 'No terminal emulator/display found to open %s in.\n' "$name" >&2
    printf 'Start Gazebo, SITL and MAVROS yourself and run with A1_2_START_SIM=0.\n' >&2
    exit 1
  fi
  mkdir -p "$LOG_DIR"
  : > "$logfile"
  run="scripts/$script_name"
  if command -v script >/dev/null 2>&1; then run="script -qefc 'scripts/$script_name' '$logfile'"; fi
  cat > "$launcher" <<LAUNCHER
#!/usr/bin/env bash
cd "$URK_DIR" || exit 1
$run
rc=\$?
echo
echo "[$name exited with status \$rc] - press Enter to close this window"
read -r _
LAUNCHER
  chmod +x "$launcher"
  log "Starting $name in a new terminal (log: $logfile)"
  # bash -i so ~/.bashrc runs (ROS, ArduPilot venv, ...), exactly like a terminal you open yourself.
  case "$term" in
    gnome-terminal)      gnome-terminal --title="$name" -- bash -ic "$launcher" >/dev/null 2>&1 & ;;
    xterm)               xterm -T "$name" -e bash -ic "$launcher" >/dev/null 2>&1 & ;;
    x-terminal-emulator) x-terminal-emulator -T "$name" -e bash -ic "$launcher" >/dev/null 2>&1 & ;;
    *) printf 'Unknown A1_2_TERMINAL=%s\n' "$term" >&2; exit 2 ;;
  esac
  disown || true
}

wait_gazebo_realtime() {
  local topic="$1" deadline=$((SECONDS + RTF_TIMEOUT)) good=0 rtf paused warned=0
  log "Waiting for Gazebo to run at >= $MIN_RTF x real time (timeout ${RTF_TIMEOUT}s)"
  while ((SECONDS < deadline)); do
    paused="$(gz_stat "$topic" paused)"
    rtf="$(gz_stat "$topic" real_time_factor)"
    if [[ "$paused" == true ]]; then
      if ((!warned)); then warn "Gazebo is PAUSED - press Play in the Gazebo window"; warned=1; fi
      good=0
    elif [[ -n "$rtf" ]] && float_ge "$rtf" "$MIN_RTF"; then
      good=$((good + 1))
      log "Gazebo real-time factor $rtf (ok $good/3)"
      if ((good >= 3)); then return 0; fi
    else
      good=0
      log "Gazebo real-time factor ${rtf:-unknown}; waiting"
    fi
    sleep 2
  done
  warn "Gazebo did not reach real-time factor $MIN_RTF in ${RTF_TIMEOUT}s (last: ${rtf:-unknown}). Continuing, but expect 'Accels/Gyros inconsistent'. Close other heavy programs, or run Gazebo headless."
}

if [[ "$START_SIM" == 1 ]]; then
  mkdir -p "$LOG_DIR"

  # 1) Gazebo
  GZ_TOPIC="$(gz_stats_topic)"
  if [[ -n "$GZ_TOPIC" ]]; then
    log "Gazebo already running ($GZ_TOPIC)"
  else
    launch_component gazebo run_gazebo.sh
    log "Waiting for Gazebo world (timeout 120s)"
    deadline=$((SECONDS + 120))
    while ((SECONDS < deadline)) && [[ -z "$GZ_TOPIC" ]]; do
      sleep 3
      GZ_TOPIC="$(gz_stats_topic)"
    done
    if [[ -z "$GZ_TOPIC" ]]; then
      printf 'Gazebo did not publish a /world/*/stats topic within 120 s; see %s/gazebo.log\n' "$LOG_DIR" >&2
      exit 1
    fi
  fi
  wait_gazebo_realtime "$GZ_TOPIC"

  # 2) ArduPilot SITL (only after the world is running, so its IMUs start in step with the sim)
  if sitl_running; then
    log "SITL already running - reusing it"
  else
    launch_component sitl run_sitl.sh
    SITL_LOG="$LOG_DIR/sitl.log"
    log "Waiting for SITL to boot (first build can take minutes; timeout ${SITL_BOOT_TIMEOUT}s)"
    # MAVProxy's --console sends FCU messages (PreArm, EKF) to its own window, not to stdout, so
    # only MAVProxy's own lines can be matched here.  GPS/EKF readiness is checked by the executor.
    SITL_BOOT_RE='Detected vehicle|Received [0-9]+ parameters|online system|ArduPilot Ready'
    deadline=$((SECONDS + SITL_BOOT_TIMEOUT))
    proc_seen_at=0
    BOOT_OK=0
    while ((SECONDS < deadline)); do
      if grep -aqE -- "$SITL_BOOT_RE" "$SITL_LOG" 2>/dev/null; then BOOT_OK=1; break; fi
      if sitl_running; then
        if ((proc_seen_at == 0)); then proc_seen_at=$SECONDS; fi
        if ((SECONDS - proc_seen_at >= 60)); then break; fi   # process is up but the log shows nothing
      fi
      sleep 2
    done
    if ((BOOT_OK)); then
      log "SITL is up"
    elif sitl_running; then
      warn "SITL is running but $SITL_LOG has no boot message yet; continuing."
    else
      printf 'SITL did not start within %s s; see %s\n' "$SITL_BOOT_TIMEOUT" "$SITL_LOG" >&2
      exit 1
    fi
  fi

  # 3) MAVROS
  if mavros_running; then
    log "MAVROS already running"
  else
    launch_component mavros run_mavros.sh
  fi
fi

# ---------------------------------------------------------------- planner + MAVROS link
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

  FCU_CONNECTED=0
  POSE_READY=0
  if ((SERVICES_READY)); then
    FCU_STATE="$(timeout 7s ros2 topic echo --once /mavros/state 2>/dev/null || true)"
    if [[ "$FCU_STATE" != *"connected: true"* ]]; then
      MAVROS_READY=0
    else
      FCU_CONNECTED=1
      POSE_SAMPLE="$(timeout 7s ros2 topic echo --once /mavros/local_position/pose 2>/dev/null || true)"
      if [[ "$POSE_SAMPLE" != *"position:"* ]]; then
        MAVROS_READY=0
      else
        POSE_READY=1
      fi
    fi
  else
    MAVROS_READY=0
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
  sleep "$POLL_INTERVAL"
done

if ((!READY)); then
  printf 'Planner/MAVROS did not become ready within %s s. Check Gazebo, SITL, MAVROS and the planner terminals.\n' \
    "$READY_TIMEOUT" >&2
  exit 1
fi
printf 'Planner ready; MAVROS connected with services and local pose available.\n'

# GPS fix / EKF readiness is NOT waited for here: the executor waits in WAIT_FCU until the EKF
# reports "is using GPS" (or the GPS fix is stable), and logs every FCU PreArm message.

# The executor's timeouts are wall-clock. If Gazebo runs at e.g. 0.18x real time, a 60 s takeoff
# timeout is only ~11 s of simulated flight, so scale them.
RTF_NOW=""
GZ_TOPIC="$(gz_stats_topic)"
if [[ -n "$GZ_TOPIC" ]]; then RTF_NOW="$(gz_stat "$GZ_TOPIC" real_time_factor)"; fi
TIME_SCALE="${A1_2_TIME_SCALE:-$(time_scale "$RTF_NOW")}"
if ! is_number "$TIME_SCALE"; then printf 'A1_2_TIME_SCALE must be a number.\n' >&2; exit 2; fi
EXEC_ARGS=(-p "mission_file:=$MISSION_FILE")
EXEC_ARGS+=(-p "handshake_timeout_sec:=$(scaled 180 "$TIME_SCALE")")
if float_ge "$TIME_SCALE" 1.5; then
  warn "Gazebo real-time factor is ${RTF_NOW:-unknown}: scaling executor timeouts x$TIME_SCALE. Fix the simulation speed if you can."
  EXEC_ARGS+=(
    -p "takeoff_timeout_sec:=$(scaled 60 "$TIME_SCALE")"
    -p "takeoff_climb_timeout_sec:=$(scaled 12 "$TIME_SCALE")"
    -p "landing_timeout_sec:=$(scaled 90 "$TIME_SCALE")"
    -p "navigation_timeout_sec:=$(scaled 120 "$TIME_SCALE")"
    -p "yaw_timeout_sec:=$(scaled 15 "$TIME_SCALE")"
    -p "yaw_hold_time_sec:=$(scaled 0.5 "$TIME_SCALE")"
    -p "settle_time:=$(scaled 1.0 "$TIME_SCALE")"
    -p "disarm_grace_sec:=$(scaled 5.0 "$TIME_SCALE")"
    -p "ground_wait_sec:=$(scaled 3.0 "$TIME_SCALE")"
  )
fi

printf 'Starting mission: %s\n' "$MISSION_FILE"
setsid ros2 run lrs_psenak_valko mission_executor_node --ros-args "${EXEC_ARGS[@]}" &
EXECUTOR_PID=$!

printf 'Mission active. Leave Gazebo and SITL visible; press Ctrl+C here to stop the executor and planner.\n'
wait "$EXECUTOR_PID"
