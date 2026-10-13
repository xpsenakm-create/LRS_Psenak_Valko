#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LRS_URK_DIR="${LRS_URK_DIR:-$(cd "$ROOT_DIR/../.." && pwd)/LRS-URK}"
if [[ ! -d "$LRS_URK_DIR" ]]; then
  LRS_URK_DIR="$HOME/Desktop/LRS/LRS-URK"
fi

WORLD="fei_lrs_outdoor.world"
RECORD_BAG=0
BAG_NAME=""
EXTRA_NODE_ARGS=()

usage() {
  printf 'Použitie: %s [--world WORLD_NAME] [--record [BAG_NAME]] [ROS_ARGS...]\n' "$0"
  printf '  --world NAZOV       Svet v Gazebo (predvolené: %s)\n' "$WORLD"
  printf '  --record [BAG_NAME] Automaticky nahrávať ROS 2 bag letu\n'
  printf '  -h, --help          Zobraziť túto nápovedu\n'
}

while (($#)); do
  case "$1" in
    --world)
      if (($# < 2)); then usage >&2; exit 2; fi
      WORLD="$2"
      shift 2
      ;;
    --record)
      RECORD_BAG=1
      if (($# >= 2)) && [[ "$2" != --* ]]; then
        BAG_NAME="$2"
        shift 2
      else
        BAG_NAME="outdoor_flight_bag_$(date +%Y%m%d_%H%M%S)"
        shift 1
      fi
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_NODE_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -d "$LRS_URK_DIR" ]]; then
  printf 'Chyba: Adresár LRS-URK nebol nájdený: %s\n' "$LRS_URK_DIR" >&2
  exit 1
fi

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  printf 'ROS setup nebol nájdený: %s\n' "$ROS_SETUP" >&2
  exit 1
fi

INSTALL_SETUP="$ROOT_DIR/install/setup.bash"
if [[ ! -f "$INSTALL_SETUP" ]]; then
  printf 'Upozornenie: %s nebol nájdený. Najprv zostavte balíček (colcon build).\n' "$INSTALL_SETUP" >&2
fi

# Funkcia na otvorenie nového terminálového okna
open_terminal() {
  local title="$1"
  local cmd="$2"

  if command -v gnome-terminal &>/dev/null; then
    gnome-terminal --title="$title" -- bash -c "$cmd; echo ''; echo 'Proces bol ukončený. Stlačte Enter pre zatvorenie okna...'; read" &
  elif command -v xterm &>/dev/null; then
    xterm -title "$title" -e bash -c "$cmd; echo ''; echo 'Proces bol ukončený. Stlačte Enter pre zatvorenie okna...'; read" &
  else
    echo "Upozornenie: Grafický terminál (gnome-terminal / xterm) nebol nájdený. Spúšťam na pozadí..."
    bash -c "$cmd" &
  fi
}

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "==> Ukončujem bežiace procesy simulácie a misie..."
  pkill -f "outdoor_mission_node" 2>/dev/null || true
  pkill -f "arducopter" 2>/dev/null || true
  pkill -f "sim_vehicle.py" 2>/dev/null || true
  pkill -f "mavros_node" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f "ros2 bag record" 2>/dev/null || true
  echo "==> Všetko bolo ukončené."
}
trap cleanup EXIT INT TERM

echo "=========================================================="
echo " Spúšťam A1.3 Outdoor misiu v 4 samostatných termináloch"
echo "=========================================================="

# 1. Terminál: Gazebo
echo "[1/4] Otváram terminál pre Gazebo ($WORLD)..."
open_terminal "1 - Gazebo ($WORLD)" "cd '$LRS_URK_DIR' && ./scripts/run_gazebo.sh '$WORLD'"
sleep 4

# 2. Terminál: SITL
echo "[2/4] Otváram terminál pre ArduPilot SITL..."
open_terminal "2 - ArduPilot SITL" "cd '$LRS_URK_DIR' && ./scripts/run_sitl.sh"
sleep 5

# 3. Terminál: MAVROS
echo "[3/4] Otváram terminál pre MAVROS..."
open_terminal "3 - MAVROS" "source '$ROS_SETUP' && cd '$LRS_URK_DIR' && ./scripts/run_mavros.sh"
sleep 4

# 4. Terminál: outdoor_mission_node
echo "[4/4] Otváram terminál pre outdoor_mission_node..."
NODE_CMD="source '$ROS_SETUP' && source '$INSTALL_SETUP' && ros2 run lrs_psenak_valko outdoor_mission_node ${EXTRA_NODE_ARGS[*]:-}"
open_terminal "4 - Outdoor Mission Node" "$NODE_CMD"

# 5. Voliteľný terminál: ros2 bag record
if ((RECORD_BAG)); then
  sleep 1
  echo "[RECORD] Otváram nahrávanie ROS 2 bagu: $BAG_NAME"
  BAG_CMD="source '$ROS_SETUP' && ros2 bag record -o '$ROOT_DIR/$BAG_NAME' /mavros/local_position/pose /mavros/setpoint_raw/local /mission/intended_path /mission/flown_path /mavros/global_position/global /mavros/global_position/compass_hdg /mavros/state"
  open_terminal "5 - ROS 2 Bag Recorder" "$BAG_CMD"
fi

echo ""
echo "=========================================================="
echo " Všetky terminály boli úspešne otvorené!"
echo "   1. Gazebo ($WORLD)"
echo "   2. ArduPilot SITL"
echo "   3. MAVROS"
echo "   4. outdoor_mission_node"
if ((RECORD_BAG)); then
  echo "   5. ROS 2 Bag Recorder ($BAG_NAME)"
fi
echo "=========================================================="
echo ""
echo "Stlačením Ctrl+C v tomto okne ukončíte všetky bežiace procesy naraz."
if ((RECORD_BAG)); then
  echo "Po ukončení letu si môžete vykresliť dráhu príkazom:"
  echo "  python3 plot_rosbag.py $BAG_NAME"
else
  echo "Dráhu letu si môžete vykresliť z logu príkazom:"
  echo "  python3 plot_rosbag.py --csv outdoor_mission_log.csv"
fi
echo ""

# Čakáme, kým používateľ stlačí Ctrl+C
while true; do
  sleep 1
done
