#!/usr/bin/env bash
# Zadanie 2 (lokalizácia, 8-stavový EKF) -- jeden skript spustí všetko naraz: Gazebo (svet
# s lokalizačnými senzormi, so seedom), SITL, MAVROS, simulátor senzorov (s tým istým seedom),
# náš estimátor a misiu letenú iba na odhade. Ručné príkazy (to isté, čo skript robí) sú v README.md.
#
#   ./run_assignment2.sh                          # okná terminálov + Gazebo GUI + živý dashboard, seed 42
#                                                 # (testy robí učiteľ ručne; --no-live bez dashboardu)
#   ./run_assignment2.sh --record --video         # + bag, grafy, súhrn a video do results/A2/<NAZOV>/
#   ./run_assignment2.sh --record --scenario      # + automatický "učiteľ": GPS 15 s, outlier, VO 3 s
#   ./run_assignment2.sh --record --scenario-random 7    # udalosti v náhodných úsekoch a časoch (seed 7)
#   ./run_assignment2.sh --headless --record --scenario --seed 73   # bez okien, počká na pristátie a všetko zavrie
#   ./run_assignment2.sh --characterize 30 --seed 73      # A2 §1: dron stojí, 30 s senzorov
#   ./run_assignment2.sh --offline                # bez simulátora: unit test + offline test filtra na 20 seedoch
#   ./run_assignment2.sh --all                    # CELÉ A2 od začiatku do konca (bez okien, ~40 min):
#                                                 # build, test, offline, 3x charakterizácia, lety, grafy,
#                                                 # video a tabuľka výsledkov do docs/A2/A2_lokalizacia.md
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Nájde koreň repozitára LRS-URK (adresár so scripts/run_gazebo.sh). Poradie: LRS_URK_DIR (aj keď
# ukazuje omylom na .../scripts alebo neexistuje), okolie tohto adresára, bežné miesta v $HOME.
# Repo stiahnuté ako ZIP z GitHubu sa volá LRS-URK-main, preto obe mená.
resolve_lrs_urk() {
  local c d n
  local candidates=()
  if [[ -n "${LRS_URK_DIR:-}" ]]; then
    c="${LRS_URK_DIR%/}"
    candidates+=("$c" "${c%/scripts}")
  fi
  for d in "$ROOT_DIR" "$ROOT_DIR/.." "$ROOT_DIR/../.." "$ROOT_DIR/../../.." "$HOME/Desktop/LRS" "$HOME" "$HOME/Desktop"; do
    for n in LRS-URK LRS-URK-main; do candidates+=("$d/$n"); done
  done
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
SCENARIO_RANDOM=""
HEADLESS=0
CHARACTERIZE=0
MISSION=1
VIDEO=0
LIVE=1
OVERWRITE=0
BUILD=1
MODE="run"
ALL_SEEDS="42 73 7 123"
RANDOM_SCENARIO_SEED=11
TIMEOUT_S=900
SCENARIO_ARGS=()

usage() {
  printf 'Použitie: %s [--seed N] [--record [NAZOV]] [--video] [--scenario | --scenario-random S]\n' "$0"
  printf '          [--headless] [--no-mission] [--characterize SEKUNDY] [--offline] [--all [--seeds "42 73 ..."]]\n'
  printf '          [-- ARGUMENTY_PRE_a2_scenario.py]\n'
  printf '  --seed N             seed pre gz sim aj pre simulátor senzorov (predvolené 42)\n'
  printf '  --record [NAZOV]     bag + grafy + súhrn do results/A2/NAZOV/ (predvolene seedN_DATUM_CAS)\n'
  printf '  --video              po lete vyrobí a2_flight.mp4 z bagu (vyžaduje --record)\n'
  printf '  --no-live            bez živého dashboardu (tools/a2_live.py; v okennom režime je zapnutý)\n'
  printf '  --scenario           tools/a2_scenario.py: GPS 15 s v strednom úseku, outlier, VO 3 s\n'
  printf '  --scenario-random S  to isté, ale úseky a časy udalostí sa vyžrebujú zo seedu S\n'
  printf '  --headless           bez okien (gz server, SITL bez konzoly), čaká na koniec misie\n'
  printf '  --no-mission         iba estimátor (bez letu)\n'
  printf '  --characterize S     A2 §1: bez SITL/MAVROS/misie, dron stojí, nahrá S sekúnd (sim) a skončí\n'
  printf '  --offline            bez simulátora: unit test ekf8_test + offline test na 20 seedoch\n'
  printf '  --all                celé A2 (headless): build, test, offline, charakterizácia, lety pre --seeds,\n'
  printf '                       jeden let s náhodným scenárom, grafy, video, tabuľka do dokumentácie\n'
  printf '  --seeds "A B ..."    seedy letov pre --all (predvolené: %s)\n' "$ALL_SEEDS"
  printf '  --overwrite          prepíše existujúci results/A2/NAZOV/\n'
  printf '  Env: LRS_URK_DIR, ARDUPILOT_DIR (predvolene ~/ardupilot), ROS_DISTRO\n'
}

while (($#)); do
  case "$1" in
    --seed) (($# >= 2)) || { usage >&2; exit 2; }; SEED="$2"; shift 2 ;;
    --record)
      RECORD_BAG=1
      if (($# >= 2)) && [[ "$2" != -* ]]; then BAG_NAME="$2"; shift 2; else shift 1; fi
      ;;
    --video) VIDEO=1; shift ;;
    --no-live) LIVE=0; shift ;;
    --scenario) SCENARIO=1; shift ;;
    --scenario-random) (($# >= 2)) || { usage >&2; exit 2; }; SCENARIO=1; SCENARIO_RANDOM="$2"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --no-mission) MISSION=0; shift ;;
    --characterize) (($# >= 2)) || { usage >&2; exit 2; }; CHARACTERIZE="$2"; HEADLESS=1; MISSION=0; RECORD_BAG=1; shift 2 ;;
    --offline) MODE="offline"; shift ;;
    --all) MODE="all"; shift ;;
    --seeds) (($# >= 2)) || { usage >&2; exit 2; }; ALL_SEEDS="$2"; shift 2 ;;
    --overwrite) OVERWRITE=1; shift ;;
    --no-build) BUILD=0; shift ;;
    --timeout) (($# >= 2)) || { usage >&2; exit 2; }; TIMEOUT_S="$2"; shift 2 ;;
    --) shift; SCENARIO_ARGS=("$@"); break ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Neznámy argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done
[[ "$SEED" =~ ^[0-9]+$ ]] || { printf 'Chyba: --seed musí byť celé číslo\n' >&2; exit 2; }
((MISSION)) || SCENARIO=0
[[ -n "$SCENARIO_RANDOM" ]] && SCENARIO_ARGS=(--random "$SCENARIO_RANDOM" "${SCENARIO_ARGS[@]}")
if ((RECORD_BAG)) && [[ -z "$BAG_NAME" ]]; then
  if ((CHARACTERIZE)); then BAG_NAME="char_seed${SEED}_$(date +%Y%m%d_%H%M%S)"; else BAG_NAME="seed${SEED}_$(date +%Y%m%d_%H%M%S)"; fi
fi
((VIDEO && !RECORD_BAG)) && { printf 'Chyba: --video potrebuje --record\n' >&2; exit 2; }

if ! LRS_URK_DIR="$(resolve_lrs_urk)"; then
  printf 'Chyba: nenašiel som repozitár LRS-URK (adresár so scripts/run_gazebo.sh).\n' >&2
  printf '  Spusti napr.: LRS_URK_DIR=/cesta/k/LRS-URK %s\n' "$0" >&2
  exit 1
fi
export LRS_URK_DIR
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
if ((BUILD)); then
  printf 'Build: lrs_psenak_valko + localization_sensor_sim\n'
  (cd "$ROOT_DIR" && colcon build --base-paths src "$SENSOR_PKG")
fi
INSTALL_SETUP="$ROOT_DIR/install/setup.bash"
set +u
source "$INSTALL_SETUP"
set -u
EKF_TEST="$ROOT_DIR/install/lrs_psenak_valko/lib/lrs_psenak_valko/ekf8_test"
EKF_REPLAY="$ROOT_DIR/install/lrs_psenak_valko/lib/lrs_psenak_valko/ekf8_replay"

# ================================================================== offline: test + 20 seedov
run_offline() {
  local out="$ROOT_DIR/results/A2/offline" sim=(python3 "$ROOT_DIR/tools/a2_offline_sim.py" --replay "$EKF_REPLAY"
    --sensor-sim-dir "$SENSOR_PKG" --seeds 1-20 --summary)
  mkdir -p "$out"
  echo "==> Unit test (ekf8_test)"
  "$EKF_TEST"
  echo "==> Offline test filtra: učiteľov model senzorov -> náš C++ filter, 20 seedov (-> $out)"
  python3 "$ROOT_DIR/tools/a2_offline_sim.py" --replay "$EKF_REPLAY" --sensor-sim-dir "$SENSOR_PKG" --seeds 42 \
    --plot "$out/offline_seed42.png" --summary > /dev/null
  {
    echo "### final config (config/localization.yaml), GPS 15 s + outlier + VO 3 s"; "${sim[@]}"
    echo; echo "### vo.velocity_model: rotate (linear-KF approximation)"; "${sim[@]}" vo_velocity_ekf=0
    echo; echo "### static alignment OFF (bias only through VO yaw)"; "${sim[@]}" zaru_enabled=0
    echo; echo "### previous config (no static alignment, VO yaw std 0.1, yaw_rate 0.002, bias rw 1e-4, accel 0.3)"
    "${sim[@]}" zaru_enabled=0 vo_yaw_std=0.1 yaw_rate=0.002 gyro_bias_rw=1e-4 accel_xy=0.3
    echo; echo "### VO yaw not used"; "${sim[@]}" use_vo_yaw=0
    echo; echo "### VO velocity std 0.03 (= message covariance)"; "${sim[@]}" vo_velocity_std=0.03
    echo; echo "### outlier injected into the FIRST GPS sample after the 15 s outage"; "${sim[@]}" --outlier-after-outage
    echo; echo "### clean flight (no outages)"; "${sim[@]}" --no-outage
  } | tee "$out/offline_20_seeds.txt"
}

# ================================================================== --all: celé zadanie
run_all() {
  if pgrep -u "$USER" -f "gz sim|arducopter|mavros_node" > /dev/null; then
    printf 'Chyba: už beží Gazebo / ArduPilot / MAVROS (pgrep). Zavri ich a spusti --all znova.\n' >&2
    exit 1
  fi
  local self="$ROOT_DIR/run_assignment2.sh" seeds=($ALL_SEEDS) runs=() s a b
  local started; started=$(date +%s)
  wait_clean() {          # nič z predošlého behu nesmie bežať ďalej (port SITL, gz server)
    for _ in $(seq 30); do pgrep -u "$USER" -f "gz sim|arducopter|mavros_node" > /dev/null || return 0; sleep 1; done
    pkill -INT -u "$USER" -f "gz sim|arducopter|mavros_node|sim_vehicle" || true; sleep 5
    pkill -KILL -u "$USER" -f "gz sim|arducopter|mavros_node|sim_vehicle" || true; sleep 2
  }
  echo "############ [1/5] unit test + offline"
  run_offline

  a="${seeds[0]}"; b="${seeds[1]:-73}"
  echo "############ [2/5] A2 §1 charakterizácia: seed $a, seed $b, znova seed $a (30 s, dron stojí)"
  for name in "seed$a:$a" "seed$b:$b" "seed${a}_repeat:$a"; do
    "$self" --no-build --overwrite --characterize 30 --seed "${name#*:}" --record "characterization/${name%%:*}" || true
    wait_clean
  done
  local tmp; tmp="$(mktemp -d)"
  python3 "$ROOT_DIR/tools/a2_characterize.py" "$ROOT_DIR/results/A2/characterization/seed$a/bag" \
    "$ROOT_DIR/results/A2/characterization/seed$b/bag" "$ROOT_DIR/results/A2/characterization/seed${a}_repeat/bag" \
    --labels "seed$a" "seed$b" "seed${a}b" --out "$tmp" --no-show \
    | tee "$ROOT_DIR/results/A2/characterization/comparison.txt"
  rm -rf -- "$tmp"        # the per-run plots are already in each run directory

  echo "############ [3/5] lety so scenárom (GPS 15 s, outlier, VO 3 s): seedy ${seeds[*]}"
  for s in "${seeds[@]}"; do
    "$self" --no-build --overwrite --headless --scenario --seed "$s" --record "flight_seed$s" || true
    runs+=("$ROOT_DIR/results/A2/flight_seed$s")
    wait_clean
  done
  echo "############ [4/5] let s náhodným scenárom (--scenario-random $RANDOM_SCENARIO_SEED, seed ${seeds[1]:-73})"
  "$self" --no-build --overwrite --headless --scenario-random "$RANDOM_SCENARIO_SEED" --seed "${seeds[1]:-73}" \
    --record "flight_random${RANDOM_SCENARIO_SEED}_seed${seeds[1]:-73}" || true
  runs+=("$ROOT_DIR/results/A2/flight_random${RANDOM_SCENARIO_SEED}_seed${seeds[1]:-73}")
  wait_clean

  echo "############ [5/5] video, tabuľka výsledkov do dokumentácie"
  python3 "$ROOT_DIR/tools/a2_video.py" "$ROOT_DIR/results/A2/flight_seed${seeds[0]}/bag" || true
  python3 "$ROOT_DIR/tools/a2_report.py" "${runs[@]}" --doc "$ROOT_DIR/docs/A2/A2_lokalizacia.md" || true
  echo "==> --all hotové za $(( ($(date +%s) - started) / 60 )) min. Výsledky: $ROOT_DIR/results/A2/"
}

case "$MODE" in
  offline) run_offline; exit 0 ;;
  all) run_all; exit 0 ;;
esac

RUN_DIR=""
if ((RECORD_BAG)); then
  RUN_DIR="$ROOT_DIR/results/A2/$BAG_NAME"
  if [[ -e "$RUN_DIR" ]]; then
    ((OVERWRITE)) || { printf 'Chyba: %s už existuje -- zvoľ iný názov alebo --overwrite.\n' "$RUN_DIR" >&2; exit 2; }
    rm -rf -- "$RUN_DIR"
  fi
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
    if ((VIDEO)); then python3 "$ROOT_DIR/tools/a2_video.py" "$RUN_DIR/bag" || true; fi
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
echo " Zadanie 2 lokalizácia: seed $SEED, $( ((HEADLESS)) && echo 'headless' || echo 'okná' )$( ((CHARACTERIZE)) && echo ", charakterizácia ${CHARACTERIZE} s" )"
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

if ((LIVE && !HEADLESS)); then
  # Živý dashboard: samostatný proces iba na pozeranie (nič nepublikuje, truth iba na zobrazenie chyby).
  echo "[LIVE] Dashboard (tools/a2_live.py)"
  launch "Live dashboard" "$ENV_CMD && exec python3 '$ROOT_DIR/tools/a2_live.py'"
fi

if ((RECORD_BAG && MISSION)); then
  # Dôkaz pre obhajobu: odbery estimátora a misie (žiadny truth, žiadna poloha z MAVROS).
  start_bg "node_info" "$ENV_CMD && sleep 20 && { echo '# ros2 node info počas letu (no truth / MAVROS pose subscription)'; \
    for n in /localization_estimator /localization_mission; do echo \"===== ros2 node info \$n\"; ros2 node info \$n; done; } \
    > '$RUN_DIR/node_info.txt' 2>&1"
fi

if ((SCENARIO)); then
  echo "[6] Scenár učiteľa (tools/a2_scenario.py ${SCENARIO_ARGS[*]:-})"
  launch "scenario" "$ENV_CMD && exec python3 '$ROOT_DIR/tools/a2_scenario.py' ${SCENARIO_ARGS[*]:-}"
fi

echo ""
echo " Ručné testy učiteľa (v ďalšom termináli):"
echo "   ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool \"{data: false}\""
echo "   ros2 service call /localization/set_vo_enabled std_srvs/srv/SetBool \"{data: false}\""
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
