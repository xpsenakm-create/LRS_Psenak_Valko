#!/usr/bin/env bash
# Pomocný skript pre A1.3 (outdoor). Otvorí 4 terminály (Gazebo, SITL, MAVROS, misia).
# Ručné príkazy (to isté, čo skript robí) sú v README.md -- na obhajobe ich vedz vysvetliť.
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Nájde koreň repozitára LRS-URK (adresár, v ktorom je scripts/run_gazebo.sh).
# Poradie: premenná LRS_URK_DIR (aj keď ukazuje omylom na .../scripts), potom okolie tohto
# adresára, potom bežné miesta v $HOME.
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

# Nájde install/setup.bash workspace-u (v tomto adresári alebo o 1-2 úrovne vyššie).
resolve_install_setup() {
  local d
  for d in "$ROOT_DIR" "$ROOT_DIR/.." "$ROOT_DIR/../.."; do
    if [[ -f "$d/install/setup.bash" ]]; then
      (cd "$d" && printf '%s/install/setup.bash\n' "$(pwd)")
      return 0
    fi
  done
  return 1
}

WORLD="fei_lrs_outdoor.world"
# Home pre SITL musí zodpovedať georeferencii sveta (viď hlavičku fei_lrs_outdoor.world).
SITL_HOME="${HOME_LOCATION:-48.15144444,17.07375000,150,0}"
RECORD_BAG=0
BAG_NAME=""
SQUARE_SIDE=""
AMPLITUDE=""
ALTITUDE=""
POI_LAT=""
POI_LON=""
POI_EAST=""
POI_NORTH=""
EXTRA_NODE_ARGS=()

usage() {
  printf 'Použitie: %s [--world NAME] [--square M] [--amplitude M] [--altitude M]\n' "$0"
  printf '          [--poi-lat DEG --poi-lon DEG | --poi-east M --poi-north M] [--record [BAG]] [ROS_ARGS...]\n'
  printf '  --world NAZOV       Svet v Gazebo (predvolené: %s)\n' "$WORLD"
  printf '  --square METRE      Strana štvorca S (predvolené: 5.0 m)\n'
  printf '  --amplitude METRE   Amplitúda osmičky A (predvolené: 3.0 m)\n'
  printf '  --altitude METRE    Letová výška (predvolené: 5.0 m)\n'
  printf '  --poi-lat/--poi-lon POI ako GPS súradnica (prepíše offset)\n'
  printf '  --poi-east/--poi-north  POI ako offset od štartu v metroch\n'
  printf '  --record [NAZOV]    Nahrávať ROS 2 bag + CSV do results/A1_3/NAZOV/ (predvolene run_DATUM_CAS);\n'
  printf '                      po pristátí stlač Ctrl+C v tomto okne -- bag sa uzavrie a vygenerujú sa grafy\n'
  printf '  Env: LRS_URK_DIR (cesta k LRS-URK), HOME_LOCATION (home pre SITL), KILL_ALL=1 (pri ukončení zabiť aj cudzie gz/sitl procesy)\n'
}

while (($#)); do
  case "$1" in
    --world) (($# >= 2)) || { usage >&2; exit 2; }; WORLD="$2"; shift 2 ;;
    --square|--square-side|--side) (($# >= 2)) || { usage >&2; exit 2; }; SQUARE_SIDE="$2"; shift 2 ;;
    --amplitude|--amp) (($# >= 2)) || { usage >&2; exit 2; }; AMPLITUDE="$2"; shift 2 ;;
    --altitude|--alt) (($# >= 2)) || { usage >&2; exit 2; }; ALTITUDE="$2"; shift 2 ;;
    --poi-lat) (($# >= 2)) || { usage >&2; exit 2; }; POI_LAT="$2"; shift 2 ;;
    --poi-lon) (($# >= 2)) || { usage >&2; exit 2; }; POI_LON="$2"; shift 2 ;;
    --poi-east) (($# >= 2)) || { usage >&2; exit 2; }; POI_EAST="$2"; shift 2 ;;
    --poi-north) (($# >= 2)) || { usage >&2; exit 2; }; POI_NORTH="$2"; shift 2 ;;
    --record)
      RECORD_BAG=1
      if (($# >= 2)) && [[ "$2" != --* ]]; then
        BAG_NAME="$2"; shift 2
      else
        BAG_NAME="run_$(date +%Y%m%d_%H%M%S)"; shift 1
      fi
      ;;
    -h|--help) usage; exit 0 ;;
    *) EXTRA_NODE_ARGS+=("$1"); shift ;;
  esac
done

# Pri --record ide všetko z jedného letu (bag, CSV log, grafy) do jedného priečinka v results/.
RUN_DIR=""
if ((RECORD_BAG)); then
  RUN_DIR="$ROOT_DIR/results/A1_3/$BAG_NAME"
  if [[ -e "$RUN_DIR" ]]; then
    printf 'Chyba: %s už existuje -- zvoľ iný názov (--record NAZOV).\n' "$RUN_DIR" >&2
    exit 2
  fi
fi

if [[ -n "$POI_LAT" || -n "$POI_LON" ]] && [[ -z "$POI_LAT" || -z "$POI_LON" ]]; then
  printf 'Chyba: --poi-lat a --poi-lon sa zadávajú spolu.\n' >&2
  exit 2
fi

if ! LRS_URK_DIR="$(resolve_lrs_urk)"; then
  printf 'Chyba: nenašiel som repozitár LRS-URK (adresár so scripts/run_gazebo.sh).\n' >&2
  printf '  LRS_URK_DIR v prostredí: "%s"\n' "${LRS_URK_DIR:-}" >&2
  printf '  Spusti napr.: LRS_URK_DIR=/cesta/k/LRS-URK %s\n' "$0" >&2
  exit 1
fi
printf 'LRS-URK: %s\n' "$LRS_URK_DIR"

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'ROS setup nebol nájdený: %s\n' "$ROS_SETUP" >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
set -u
printf 'Build workspace-u: %s\n' "$ROOT_DIR"
(cd "$ROOT_DIR" && colcon build)

INSTALL_SETUP="$ROOT_DIR/install/setup.bash"
set +u
source "$INSTALL_SETUP"
set -u
printf 'Workspace: %s\n' "$INSTALL_SETUP"

open_terminal() {
  local title="$1"
  local cmd="$2"
  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="$title" -- bash -c "$cmd; echo ''; echo 'Proces skončil. Stlač Enter pre zatvorenie okna...'; read" &
  elif command -v xterm &>/dev/null; then
    xterm -title "$title" -e bash -c "$cmd; echo ''; echo 'Proces skončil. Stlač Enter pre zatvorenie okna...'; read" &
  else
    echo "Upozornenie: gnome-terminal ani xterm nenájdený, spúšťam na pozadí..."
    bash -c "$cmd" &
  fi
}

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "==> Ukončujem procesy simulácie a misie (len tvoje, user=$USER)..."
  # Zabíjame len procesy tohto používateľa; -f hľadá v celej príkazovej riadke.
  # SIGINT (ako Ctrl+C): uzol pri ňom zapíše zvyšok CSV logu a ros2 bag uzavrie bag (metadata.yaml).
  pkill -INT -u "$USER" -f "outdoor_mission_node" 2>/dev/null || true
  pkill -INT -u "$USER" -f "ros2 bag record" 2>/dev/null || true
  if [[ -n "$RUN_DIR" && -d "$RUN_DIR" ]]; then
    echo "==> Čakám na uzavretie bagu..."
    for _ in $(seq 20); do
      [[ -f "$RUN_DIR/bag/metadata.yaml" ]] && break
      sleep 0.5
    done
    sleep 1
    if [[ -f "$RUN_DIR/outdoor_mission_log.csv" ]] && (($(wc -l < "$RUN_DIR/outdoor_mission_log.csv") > 1)); then
      echo "==> Generujem grafy do $RUN_DIR"
      (cd "$RUN_DIR" && python3 "$ROOT_DIR/plot_rosbag.py" --csv outdoor_mission_log.csv --no-show) || true
    else
      echo "    CSV log je prázdny (misia neodštartovala?) -- grafy negenerujem."
    fi
    [[ -f "$RUN_DIR/bag/metadata.yaml" ]] || echo "    POZOR: bag sa neuzavrel (chýba metadata.yaml)."
    echo "==> Výsledky letu: $RUN_DIR"
  fi
  if [[ "${KILL_ALL:-0}" == "1" ]]; then
    pkill -u "$USER" -f "arducopter" 2>/dev/null || true
    pkill -u "$USER" -f "sim_vehicle.py" 2>/dev/null || true
    pkill -u "$USER" -f "mavros_node" 2>/dev/null || true
    pkill -u "$USER" -f "gz sim" 2>/dev/null || true
  else
    echo "    Gazebo/SITL/MAVROS nechávam bežať (zavri ich okná, alebo spusti s KILL_ALL=1)."
  fi
  echo "==> Hotovo."
}
trap cleanup EXIT INT TERM

echo "=========================================================="
echo " Spúšťam A1.3 Outdoor misiu v samostatných termináloch"
echo " SITL home: $SITL_HOME"
echo "=========================================================="

# 1. Gazebo
echo "[1/4] Gazebo ($WORLD)..."
open_terminal "1 - Gazebo ($WORLD)" "cd '$LRS_URK_DIR' && ./scripts/run_gazebo.sh '$WORLD'"
sleep 4

# 2. SITL -- home musí byť rovnaký ako georeferencia sveta
echo "[2/4] ArduPilot SITL..."
open_terminal "2 - ArduPilot SITL" "cd '$LRS_URK_DIR' && HOME_LOCATION='$SITL_HOME' ./scripts/run_sitl.sh"
sleep 5

# 3. MAVROS
echo "[3/4] MAVROS..."
open_terminal "3 - MAVROS" "source '$ROS_SETUP' && cd '$LRS_URK_DIR' && ./scripts/run_mavros.sh"
sleep 4

# 4. outdoor_mission_node
echo "[4/4] outdoor_mission_node..."

to_double() {
  python3 -c "import sys; print(f'{float(sys.argv[1]):.7f}')" "$1"
}

NODE_PARAMS=()
[[ -n "$SQUARE_SIDE" ]] && NODE_PARAMS+=("-p" "square_side_m:=$(to_double "$SQUARE_SIDE")")
[[ -n "$AMPLITUDE" ]] && NODE_PARAMS+=("-p" "figure8_amplitude_m:=$(to_double "$AMPLITUDE")")
[[ -n "$ALTITUDE" ]] && NODE_PARAMS+=("-p" "flight_altitude_m:=$(to_double "$ALTITUDE")")
[[ -n "$POI_LAT" ]] && NODE_PARAMS+=("-p" "poi_lat_deg:=$(to_double "$POI_LAT")" "-p" "poi_lon_deg:=$(to_double "$POI_LON")")
[[ -n "$POI_EAST" ]] && NODE_PARAMS+=("-p" "poi_east_m:=$(to_double "$POI_EAST")")
[[ -n "$POI_NORTH" ]] && NODE_PARAMS+=("-p" "poi_north_m:=$(to_double "$POI_NORTH")")

# Geofence automaticky prispôsobíme, aby misia neabortla sama. Pri POI zadanom ako lat/lon
# sa vzdialenosť od štartu zistí až v uzle, preto sem pridáme rezervu podľa --poi-east/north
# (pri lat/lon nechaj limit explicitne cez -p max_horizontal_from_launch_m:=...).
if [[ -n "$SQUARE_SIDE$AMPLITUDE$ALTITUDE$POI_EAST$POI_NORTH" ]]; then
  S_VAL="${SQUARE_SIDE:-5.0}"
  A_VAL="${AMPLITUDE:-3.0}"
  Z_VAL="${ALTITUDE:-5.0}"
  E_VAL="${POI_EAST:-0.0}"
  N_VAL="${POI_NORTH:-0.0}"
  LIMITS=$(python3 -c "
import math, sys
s, a, z, e, n = (float(v) for v in sys.argv[1:6])
req_r = math.hypot(e, n) + max(s * math.sqrt(2.0) / 2.0, a)
max_h = float(max(4.0, math.ceil(req_r + 1.0)))
max_z = float(max(5.5, math.ceil(z + 1.0)))
print(f'{max_h:.2f} {max_z:.2f}')
" "$S_VAL" "$A_VAL" "$Z_VAL" "$E_VAL" "$N_VAL")
  MAX_H=$(echo "$LIMITS" | cut -d' ' -f1)
  MAX_Z=$(echo "$LIMITS" | cut -d' ' -f2)
  NODE_PARAMS+=("-p" "max_horizontal_from_launch_m:=$MAX_H" "-p" "max_altitude_m:=$MAX_Z")
fi

if ((RECORD_BAG)); then
  mkdir -p "$RUN_DIR"
  NODE_PARAMS+=("-p" "log_csv:=$RUN_DIR/outdoor_mission_log.csv")
fi

ROS_ARGS=()
if ((${#NODE_PARAMS[@]} > 0)); then
  ROS_ARGS=(--ros-args "${NODE_PARAMS[@]}")
fi

NODE_CMD="source '$ROS_SETUP' && source '$INSTALL_SETUP' && ros2 run lrs_psenak_valko outdoor_mission_node ${EXTRA_NODE_ARGS[*]:-} ${ROS_ARGS[*]:-}"
open_terminal "4 - Outdoor Mission Node" "$NODE_CMD"

# Voliteľný bag. Zadanie odporúča 'ros2 bag record -a'; tu je explicitný zoznam vrátane
# rýchlosti, rel_alt a home (potrebné na overenie výšky a origin).
if ((RECORD_BAG)); then
  sleep 1
  echo "[RECORD] ROS 2 bag + CSV: $RUN_DIR"
  BAG_CMD="source '$ROS_SETUP' && ros2 bag record -o '$RUN_DIR/bag' \
/mavros/local_position/pose /mavros/local_position/velocity_local /mavros/setpoint_raw/local \
/mission/intended_path /mission/flown_path /mission/state /mavros/global_position/global \
/mavros/global_position/rel_alt /mavros/global_position/compass_hdg \
/mavros/home_position/home /mavros/state"
  open_terminal "5 - ROS 2 Bag Recorder" "$BAG_CMD"
fi

echo ""
echo "=========================================================="
echo " Terminály otvorené: Gazebo, SITL, MAVROS, outdoor_mission_node"
((RECORD_BAG)) && echo " + bag recorder -> $RUN_DIR"
echo " Abort počas letu:"
echo "   ros2 service call /outdoor_mission_node/abort std_srvs/srv/Trigger"
echo " Ctrl+C v tomto okne ukončí misiu (a pri KILL_ALL=1 aj simuláciu)."
echo "=========================================================="
if ((RECORD_BAG)); then
  echo "Po pristátí stlač Ctrl+C: bag sa uzavrie a grafy sa uložia do $RUN_DIR"
else
  echo "Graf: python3 plot_rosbag.py --csv outdoor_mission_log.csv"
fi

while true; do
  sleep 1
done