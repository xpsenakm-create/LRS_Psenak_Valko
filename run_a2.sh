#!/usr/bin/env bash
# Pomocný skript pre A2 (lokalizácia, 8-stavový EKF). Spustí Gazebo (svet s lokalizačnými
# senzormi, so seedom), SITL, MAVROS, simulátor senzorov (s tým istým seedom), náš estimátor
# a misiu letenú na odhade. Ručné príkazy (to isté, čo skript robí) sú v README.md.
#
#   ./run_a2.sh                         # okná terminálov + Gazebo GUI, seed 42
#   ./run_a2.sh --seed 73 --record      # bag + grafy do results/A2/<NAZOV>/
#   ./run_a2.sh --record --scenario     # + automatický "učiteľ": GPS 15 s, outlier, VO 3 s
#   ./run_a2.sh --headless --record --scenario   # bez okien, počká na koniec misie a všetko zavrie
#   ./run_a2.sh --characterize 40 --seed 73      # len Gazebo + senzory + estimátor, 40 s na zemi (A2 §1)
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Nájde koreň repozitára LRS-URK (adresár so scripts/run_gazebo.sh), rovnako ako run_a1_3.sh.
resolve_lrs_urk() {
  local c d
  local candidates=()
  if [[ -n "${LRS_URK_DIR:-}" ]]; then
    c="${LRS_URK_DIR%/}"
    candidates+=("$c" "${c%/scripts}")
  fi
  for d in "$ROOT_DIR" "$ROOT_DIR/.." "$ROOT_DIR/../.." "$ROOT_DIR/../../.."; do
    candidates+=("$d/LRS-URK")
  done
  candidates+=("$HOME/Desktop/LRS/LRS-URK" "$HOME/LRS-URK" "$HOME/Desktop/LRS-URK")
  for c in "${candidates[@]}"; do
    if [[ -f "$c/scripts/run_gazebo.sh" ]]; then
      (cd "$c" && pwd)
      return 0
    fi
  done
  return 1
}

SEED=42
RECORD_BAG=0
BAG_NAME=""
SCENARIO=0
HEADLESS=0
CHARACTERIZE=0
MISSION=1
TIMEOUT_S=900
SCENARIO_ARGS=()

usage() {
  printf 'Použitie: %s [--seed N] [--record [NAZOV]] [--scenario] [--headless] [--no-mission]\n' "$0"
  printf '          [--characterize SEKUNDY] [--timeout SEKUNDY] [-- ARGUMENTY_PRE_a2_scenario.py]\n'
  printf '  --seed N            seed pre gz sim aj pre simulátor senzorov (predvolené 42)\n'
  printf '  --record [NAZOV]    bag + grafy do results/A2/NAZOV/ (predvolene seedN_DATUM_CAS)\n'
  printf '  --scenario          tools/a2_scenario.py: GPS vypnúť 15 s v strednom úseku, outlier, VO 3 s\n'
  printf '  --headless          bez okien (gz server, SITL bez konzoly), čaká na koniec misie\n'
  printf '  --no-mission        iba estimátor (bez letu)\n'
  printf '  --characterize S    A2 §1: bez SITL/MAVROS/misie, dron stojí, nahrá S sekúnd (sim) a skončí\n'
  printf '  Env: LRS_URK_DIR, ARDUPILOT_DIR (predvolene ~/ardupilot), ROS_DISTRO\n'
}

while (($#)); do
  case "$1" in
    --seed) (($# >= 2)) || { usage >&2; exit 2; }; SEED="$2"; shift 2 ;;
    --record)
      RECORD_BAG=1
      if (($# >= 2)) && [[ "$2" != -* ]]; then BAG_NAME="$2"; shift 2; else shift 1; fi
      ;;
    --scenario) SCENARIO=1; shift ;;
    --headless) HEADLESS=1; shift ;;
    --no-mission) MISSION=0; shift ;;
    --characterize) (($# >= 2)) || { usage >&2; exit 2; }; CHARACTERIZE="$2"; HEADLESS=1; MISSION=0; RECORD_BAG=1; shift 2 ;;
    --timeout) (($# >= 2)) || { usage >&2; exit 2; }; TIMEOUT_S="$2"; shift 2 ;;
    --) shift; SCENARIO_ARGS=("$@"); break ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Neznámy argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$SEED" =~ ^[0-9]+$ ]] || { printf 'Chyba: --seed musí byť celé číslo\n' >&2; exit 2; }
((MISSION)) || SCENARIO=0
if ((RECORD_BAG)) && [[ -z "$BAG_NAME" ]]; then
  if ((CHARACTERIZE)); then BAG_NAME="char_seed${SEED}_$(date +%Y%m%d_%H%M%S)"; else BAG_NAME="seed${SEED}_$(date +%Y%m%d_%H%M%S)"; fi
fi

if ! LRS_URK_DIR="$(resolve_lrs_urk)"; then
  printf 'Chyba: nenašiel som repozitár LRS-URK (adresár so scripts/run_gazebo.sh).\n' >&2
  printf '  Spusti napr.: LRS_URK_DIR=/cesta/k/LRS-URK %s\n' "$0" >&2
  exit 1
fi
SENSOR_PKG="$LRS_URK_DIR/localization_sensor_sim"
WORLD="$LRS_URK_DIR/worlds/fei_lrs_gazebo_localization.world"
[[ -f "$WORLD" && -d "$SENSOR_PKG" ]] || { printf 'Chyba: v %s chýba lokalizačný svet alebo localization_sensor_sim\n' "$LRS_URK_DIR" >&2; exit 1; }
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
printf 'LRS-URK: %s | seed: %s\n' "$LRS_URK_DIR" "$SEED"

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
[[ -f "$ROS_SETUP" ]] || { printf 'ROS setup nebol nájdený: %s\n' "$ROS_SETUP" >&2; exit 1; }
set +u
source "$ROS_SETUP"
set -u
# Náš balík + simulátor senzorov z LRS-URK sa zostavia do TOHTO workspace-u (LRS-URK ostane čisté).
printf 'Build: lrs_psenak_valko + localization_sensor_sim\n'
(cd "$ROOT_DIR" && colcon build --base-paths src "$SENSOR_PKG")
INSTALL_SETUP="$ROOT_DIR/install/setup.bash"
set +u
source "$INSTALL_SETUP"
set -u

RUN_DIR=""
if ((RECORD_BAG)); then
  RUN_DIR="$ROOT_DIR/results/A2/$BAG_NAME"
  [[ -e "$RUN_DIR" ]] && { printf 'Chyba: %s už existuje -- zvoľ iný názov.\n' "$RUN_DIR" >&2; exit 2; }
  mkdir -p "$RUN_DIR"
fi
LOG_DIR="${RUN_DIR:-$ROOT_DIR/log/a2_$(date +%Y%m%d_%H%M%S)}/logs"
mkdir -p "$LOG_DIR"

BAG_TOPICS=(/localization/gps /localization/vo /localization/imu /evaluation/ground_truth
  /estimator/odometry /estimator/debug /estimator/innovation /estimator/events
  /mission/state /mission/leg /mission/carrot /mavros/setpoint_raw/local /mavros/state /rosout)
ENV_CMD="source '$ROS_SETUP' && source '$INSTALL_SETUP'"
GZ_ENV="GZ_SIM_RESOURCE_PATH='$LRS_URK_DIR/models:${GZ_SIM_RESOURCE_PATH:-}'"

# ------------------------------------------------------------------ process handling
PGIDS=()
BAG_PGID=""
LAST_PGID=""
# Job control: every background job gets its own process group (killed as a whole) and keeps
# the default SIGINT handling -- without it a script's "&" jobs IGNORE SIGINT and ros2 bag would
# not close the bag on Ctrl+C.
set -m
start_bg() {            # start_bg NAME CMD -- own process group, log in $LOG_DIR/NAME.log
  local name="$1" cmd="$2"
  bash -c "$cmd" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
  local pid=$!
  PGIDS+=("$pid")
  printf '  [%s] pid %s, log %s\n' "$name" "$pid" "$LOG_DIR/$name.log"
  LAST_PGID="$pid"
}
open_terminal() {
  local title="$1" cmd="$2"
  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="$title" -- bash -c "$cmd; echo ''; echo 'Proces skončil. Stlač Enter pre zatvorenie okna...'; read" &
  elif command -v xterm &>/dev/null; then
    xterm -title "$title" -e bash -c "$cmd; echo ''; echo 'Proces skončil. Stlač Enter pre zatvorenie okna...'; read" &
  else
    start_bg "$title" "$cmd"
  fi
}
launch() {              # launch NAME CMD -- window (interactive) or background (headless)
  if ((HEADLESS)); then start_bg "$1" "$2"; else open_terminal "$1" "$2"; fi
}

make_plots() {
  [[ -n "$RUN_DIR" && -f "$RUN_DIR/bag/metadata.yaml" ]] || return 0
  if ((CHARACTERIZE)); then
    python3 "$ROOT_DIR/tools/a2_characterize.py" "$RUN_DIR/bag" --labels "seed$SEED" --out "$RUN_DIR" --no-show \
      2>/dev/null | tee "$RUN_DIR/characterization.txt" || true
  else
    python3 "$ROOT_DIR/tools/a2_plot.py" "$RUN_DIR/bag" --out "$RUN_DIR" --no-show 2>/dev/null \
      | tee "$RUN_DIR/summary.txt" || true
    python3 "$ROOT_DIR/tools/a2_characterize.py" "$RUN_DIR/bag" --labels "seed$SEED" --out "$RUN_DIR" --no-show \
      2>/dev/null > "$RUN_DIR/characterization.txt" || true
  fi
}

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "==> Ukončujem procesy..."
  if [[ -n "$BAG_PGID" ]]; then
    kill -INT -- "-$BAG_PGID" 2>/dev/null || true
    for _ in $(seq 20); do [[ -f "$RUN_DIR/bag/metadata.yaml" ]] && break; sleep 0.5; done
    [[ -f "$RUN_DIR/bag/metadata.yaml" ]] || kill -TERM -- "-$BAG_PGID" 2>/dev/null || true
    for _ in $(seq 20); do [[ -f "$RUN_DIR/bag/metadata.yaml" ]] && break; sleep 0.5; done
  fi
  if ((HEADLESS)); then
    for ((i = ${#PGIDS[@]} - 1; i >= 0; i--)); do kill -INT -- "-${PGIDS[i]}" 2>/dev/null || true; done
    sleep 3
    for pg in "${PGIDS[@]}"; do kill -TERM -- "-$pg" 2>/dev/null || true; done
  else
    pkill -INT -u "$USER" -f "^/.*lrs_psenak_valko/lib/lrs_psenak_valko/(localization_estimator|localization_mission)_node" 2>/dev/null || true
    echo "    Gazebo/SITL/MAVROS/senzory nechávam bežať v ich oknách (zavri ich ručne)."
    for _ in $(seq 20); do [[ -z "$RUN_DIR" || -f "$RUN_DIR/bag/metadata.yaml" ]] && break; sleep 0.5; done
  fi
  if [[ -n "$RUN_DIR" ]]; then
    [[ -f "$RUN_DIR/bag/metadata.yaml" ]] || echo "    POZOR: bag sa neuzavrel (chýba metadata.yaml)."
    echo "==> Grafy a súhrn do $RUN_DIR"
    make_plots
  fi
  echo "==> Hotovo. Logy: $LOG_DIR"
}
trap cleanup EXIT INT TERM

sim_seconds() {         # current /clock in seconds (sim time), empty if not available
  timeout 5 ros2 topic echo --once --field clock /clock 2>/dev/null |
    awk '/^sec:/{s=$2} /^nanosec:/{n=$2} END{if (s != "") printf "%.1f\n", s + n / 1e9}'
}

# ------------------------------------------------------------------ start everything
echo "=========================================================="
echo " A2 lokalizácia: seed $SEED, $( ((HEADLESS)) && echo 'headless' || echo 'okná' )$( ((CHARACTERIZE)) && echo ", charakterizácia ${CHARACTERIZE} s" )"
echo "=========================================================="
echo "[1] Gazebo (fei_lrs_gazebo_localization.world, --seed $SEED)"
if ((HEADLESS)); then
  launch "gazebo" "cd '$LRS_URK_DIR' && $GZ_ENV exec gz sim -s -r -v 3 --headless-rendering --seed $SEED '$WORLD'"
else
  launch "1 - Gazebo (seed $SEED)" "cd '$LRS_URK_DIR' && $GZ_ENV gz sim -r -v 4 --seed $SEED '$WORLD'"
fi
sleep 6

if ((!CHARACTERIZE)); then
  echo "[2] ArduPilot SITL"
  if ((HEADLESS)); then
    # bez DISPLAY nespustí sim_vehicle.py ani konzolu, ani xterm okno
    launch "sitl" "cd '$ARDUPILOT_DIR/ArduCopter' && exec env -u DISPLAY python3 '$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py' \
      -v ArduCopter -f gazebo-iris --model JSON --add-param-file='$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm' \
      --no-rebuild -l 48.15084570555732,17.072729745416016,150,0 --mavproxy-args=--non-interactive"
  else
    launch "2 - ArduPilot SITL" "cd '$LRS_URK_DIR' && ./scripts/run_sitl.sh"
  fi
  sleep 8
  echo "[3] MAVROS"
  launch "mavros" "source '$ROS_SETUP' && cd '$LRS_URK_DIR' && exec ./scripts/run_mavros.sh"
  sleep 4
fi

if ((RECORD_BAG)); then
  # Bag beží ešte PRED simulátorom senzorov: prvé GPS/VO vzorky sú v bagu (porovnanie seedov).
  echo "[REC] ROS 2 bag -> $RUN_DIR/bag"
  start_bg "bag" "$ENV_CMD && exec ros2 bag record -s mcap -o '$RUN_DIR/bag' ${BAG_TOPICS[*]}"
  BAG_PGID="$LAST_PGID"
  sleep 2
fi

echo "[4] Simulátor senzorov (seed $SEED)"
launch "sensors" "$ENV_CMD && exec ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=$SEED"
sleep 4

echo "[5] Estimátor$( ((MISSION)) && echo ' + misia' )"
launch "estimator" "$ENV_CMD && exec ros2 launch lrs_psenak_valko a2_localization.launch.py mission:=$( ((MISSION)) && echo true || echo false )"

if ((SCENARIO)); then
  echo "[6] Scenár učiteľa (tools/a2_scenario.py ${SCENARIO_ARGS[*]:-})"
  launch "scenario" "$ENV_CMD && exec python3 '$ROOT_DIR/tools/a2_scenario.py' ${SCENARIO_ARGS[*]:-}"
fi

echo ""
echo " Ručné testy učiteľa (v ďalšom termináli):"
echo "   ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool \"{data: false}\""
echo "   ros2 service call /localization/inject_gps_outlier std_srvs/srv/Trigger \"{}\""
echo " Abort: ros2 service call /localization_mission/abort std_srvs/srv/Trigger"
echo ""

if ((CHARACTERIZE)); then
  start=""
  for _ in $(seq 30); do start="$(sim_seconds)"; [[ -n "$start" ]] && break; sleep 1; done
  [[ -n "$start" ]] || { echo "Chyba: /clock nebeží"; exit 1; }
  echo "==> Nahrávam ${CHARACTERIZE} s simulačného času od t=${start} s..."
  while true; do
    now_s="$(sim_seconds)"
    [[ -n "$now_s" ]] && python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) - float(sys.argv[2]) >= float(sys.argv[3]) else 1)" \
      "$now_s" "$start" "$CHARACTERIZE" && break
    sleep 2
  done
  exit 0
fi

if ((HEADLESS)); then
  echo "==> Čakám na koniec misie (max ${TIMEOUT_S} s)... log: $LOG_DIR/estimator.log"
  for ((s = 0; s < TIMEOUT_S; s += 2)); do
    sleep 2
    if ((MISSION)) && grep -qE 'State: [A-Z_]+ -> (DONE|ABORTED|STOPPED|FAULT)' "$LOG_DIR/estimator.log" 2>/dev/null; then
      grep -E 'State: [A-Z_]+ -> (DONE|ABORTED|STOPPED|FAULT)' "$LOG_DIR/estimator.log" | tail -1
      sleep 4
      exit 0
    fi
  done
  echo "==> Timeout ${TIMEOUT_S} s"
  exit 1
fi

echo "Ctrl+C v tomto okne ukončí misiu a nahrávanie$( ((RECORD_BAG)) && echo ' (potom sa vygenerujú grafy)' )."
while true; do sleep 1; done
