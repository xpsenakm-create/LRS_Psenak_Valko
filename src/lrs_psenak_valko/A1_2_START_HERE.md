# A1.2 Mission Execution Starter

> Full documentation: [docs/A1_2/A1_2_indoor_misia.md](../../docs/A1_2/A1_2_indoor_misia.md) (SK, team format); detailed EN version: [docs/A1_2/A1_2_detailed_EN.md](../../docs/A1_2/A1_2_detailed_EN.md).
> Line-by-line study guide: open `docs/A1_study_guide.html` in a browser
> (rebuild with `python3 docs/study_guide/build_study_guide.py`).

## Package layout

```text
lrs_psenak_valko/
  missions/
    hangar_example.csv
  src/
    a_star_planner.h/.cpp          # Existing A1.1 path planner
    voxel_occupancy_grid.h         # Existing inflated map, including rack boxes
    map_planner_node.cpp           # Existing plan_request -> planned_path service-by-topic
    mission_model.h/.cpp           # Waypoint/task types and CSV file parser
    mission_executor_node.cpp      # A1.2 state machine and MAVROS controller
  config/
    planner.yaml                   # Hangar map and solid shelving-rack geometry
  CMakeLists.txt
  package.xml
```

`MissionExecutorNode` owns the mission state and MAVROS interfaces. `mission::Waypoint`
holds a goal, a `soft`/`hard` precision, and one task. `mission::loadCsv()` parses a
mission at runtime. The map planner remains a separate ROS node: for each mission goal,
the executor publishes a two-pose request and follows the returned collision-checked,
simplified path. Intermediate path poses are pass-through targets, not mission stops.

## Mission file format

CSV columns are `x,y,z,precision,task`, with coordinates in metres in the map's ENU
frame. `task` is `-`, `takeoff`, `land`, `landtakeoff`, or `yaw` followed by degrees.
Blank lines and `#` comments are accepted. The example at `missions/hangar_example.csv`
reproduces the assignment mission; replace its contents or pass another file without
recompiling.

The hangar map uses 0.15 m voxels. Controller defaults are `hard_radius=0.15 m`
(one voxel), `soft_radius=0.55 m` (about 3.7 voxels, allowing a visible fly-through
window), and `pass_radius=0.45 m` (three voxels, so path samples are pass-through
targets). Hard arrival also requires speed below `0.20 m/s` across at least two distinct
velocity samples spanning `1.0 s`;
soft arrival does not require stopping. Setpoint rate defaults to 20 Hz and accepts
values from 10 to 20 Hz. Handshake, service-response, planner-response, takeoff, and
landing waits are bounded by configurable timeouts. The executor plans the following
mission leg while flying the current leg, so it can hand off to the next setpoint
without waiting at a soft waypoint for A* to run. It rejects planner-snapped endpoints
outside their mission waypoint's acceptance radius. Hard arrival logs measured error
and speed after the stability interval; soft arrival logs its error and proceeds
without settling. These are starting values, not measured flight results; verify and
tune them against the actual SITL response.

Yaw task angles such as `yaw90` are **absolute world-frame ENU headings**, not relative
turns: 0 degrees faces +X/East, and positive angles rotate counter-clockwise when viewed
from above. MAVROS local pose and position setpoints use ENU; MAVROS performs the
ENU-to-NED conversion for MAVLink/ArduPilot, so the executor does not apply a second
manual NED conversion. This Gazebo world/SITL combination has an additional 90-degree
rotation between its Gazebo map axes and MAVROS local axes: the observed Gazebo drone
position `(13, 7)` corresponds to MAVROS local `(-7, 13)`. The executor therefore uses
`map_to_local_yaw_offset_deg=90` by default for both position and heading conversion;
its startup log prints both positions so the mapping is visible. Do not start a mission
if that displayed map position is outside the planner's bounds. `yaw_tolerance_deg=5`
must be held for `yaw_hold_time_sec=0.5` before
a yaw task completes, with a `yaw_timeout_sec=15` limit. Each route aligns to its first
travel segment before translating; at bends of at least
`turn_alignment_threshold_deg=20`, it holds near the corner and aligns before the next
segment. Smaller changes are followed continuously so the drone does not stop at every
planner point. After a yaw task, the next route's travel heading is reacquired before
movement begins.

## Build and run

The current environment has the upstream simulation checkout at `~/LRS-URK`,
ArduPilot at `~/ardupilot`, and this A1.2 workspace at
`~/Zadania/LRS_Psenak_Valko`. Start each simulation component in its own terminal.
Source ROS Jazzy in each shell first.

Once the three simulation terminals below are running, the workspace helper can
build the package, start the planner, wait for MAVROS readiness, and launch the mission:

```bash
cd ~/Zadania/LRS_Psenak_Valko
./run_a1_2.sh
```

Pass another mission file as the first argument, for example
`./run_a1_2.sh /absolute/path/to/defence_mission.csv`. For a relative mission path,
the helper resolves it from the directory where you invoke it. Set
`A1_2_READY_TIMEOUT=180` to allow more startup time. By default the helper
opens Gazebo, SITL and MAVROS in their own terminals (from `A1_2_URK_DIR`, default
`~/LRS-URK`). If you start those three yourself, run it with `A1_2_START_SIM=0`.
To avoid rebuilding while Gazebo/SITL are under load, build once with
`colcon build --packages-select lrs_psenak_valko`, then launch with
`A1_2_SKIP_BUILD=1 ./run_a1_2.sh`. Readiness polling defaults to every five seconds;
override it with `A1_2_POLL_INTERVAL` if needed.
While waiting, it reports planner, MAVROS service, FCU connection, and local-pose
readiness every ten seconds. If it times out, it will not start the executor; check
the three simulation terminals, then run the helper again after fixing the missing
prerequisite. Starting the helper after readiness succeeds automatically starts the
mission and can arm/fly the simulated vehicle.

The helper expects the ROS names `/mavros/state`, `/mavros/local_position/pose`,
`/mavros/set_mode`, `/mavros/cmd/arming`, `/mavros/cmd/takeoff`, and `/mavros/cmd/land`.
Confirm the actual names with `ros2 node list`, `ros2 topic list`, and
`ros2 service list` if the readiness report says they are missing. A SITL message such
as `Mission is stale` does not mean the A1.2 executor started: A1.2 sends Guided mode
setpoints, and movement only begins once `mission_executor_node` is running. Check
`ros2 node list` for that node after starting the helper. MAVROS local pose and velocity
use best-effort QoS; the executor subscribes using `SensorDataQoS` so it receives those
telemetry messages. If the executor remains in `WAIT_FCU`, inspect both its FCU state
and `/mavros/local_position/pose` endpoint QoS/subscriber count.

Terminal 1, Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/LRS-URK
scripts/run_gazebo.sh
```

Terminal 2, ArduPilot SITL (start after Gazebo is running):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/LRS-URK
scripts/run_sitl.sh
```

The SITL script selects `gazebo-iris`, explicitly enables `--model JSON`, loads
`gazebo-iris.parm`, and uses the FEI hangar home location. Keep the SITL console
visible because it reports pre-arm failures.

Terminal 3, MAVROS (start after SITL):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/LRS-URK
scripts/run_mavros.sh
```

Build and source the A1.2 workspace in Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Zadania/LRS_Psenak_Valko
colcon build --packages-select lrs_psenak_valko
source install/setup.bash
```

Terminal 5, planner (run from the workspace root so the occupancy PCD is written
there):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Zadania/LRS_Psenak_Valko
source install/setup.bash
ros2 run lrs_psenak_valko map_planner_node --ros-args \
  --params-file src/lrs_psenak_valko/config/planner.yaml \
  -p occupancy_pcd_path:=$PWD/map_voxels_with_racks_inflated.pcd
```

Terminal 6, first verify MAVROS and planner are ready:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Zadania/LRS_Psenak_Valko
source install/setup.bash
ros2 topic info /mavros/local_position/pose
ros2 service list | grep -E '^/mavros/(set_mode|cmd/arming|cmd/takeoff|cmd/land)$'
ros2 node list
```

First, wait for a connected FCU instead of trusting the first state sample:

```bash
ros2 topic echo /mavros/state
```

Once a message shows `connected: true`, press Ctrl+C. Then check that local pose has a
publisher, the four MAVROS services are listed, and `/map_planner_node` is in the node
list. Start the mission only after these checks pass:

```bash
ros2 run lrs_psenak_valko mission_executor_node --ros-args \
  -p mission_file:=$PWD/src/lrs_psenak_valko/missions/hangar_example.csv
```

The executor will arm and fly automatically; have the Gazebo and SITL windows visible.
Watch for `Mission state:` and `Completed task` logs, and wait for final `DONE` and
disarm confirmation. Do not use a manual `/plan_request` test while the executor is
running, because it shares the planner's request/result topics. To test the planner
manually, stop the executor first and use the examples in the package README.

The node logs `IDLE`, then waits for both `/mavros/state.connected` and local pose
telemetry. Watch its timestamped `Mission state:` log and the SITL console for
arming/pre-arm failures. Service calls are made only when their MAVROS service is
available, their responses are checked, and a missing response or state transition
ends in a logged `ERROR` rather than an indefinite wait. Each planner leg has a
configurable `navigation_timeout_sec` (default 120 s); during waypoint execution the
node also aborts on FCU disconnect, unexpected disarm, or leaving `GUIDED`. On failure
it holds the most recently measured local pose and logs the state and reason.
`/mavros/setpoint_position/local` is published at 20 Hz; the planner and executor use
the package's relative `plan_request`, `planned_path`, and `planning_status` topics.

Start-up sequence: `WAIT_FCU` waits for `/mavros/state.connected`, fresh local pose, and
GPS/EKF readiness: the FCU message `EKF3 IMUx is using GPS` (from
`/mavros/statustext/recv`) or, if the executor was started after that message, a GPS fix
stable for `gps_stable_sec=5`. On a slow machine this can take a couple of minutes after
SITL boots; the executor logs the reason every 5 s (`fcu_ready_timeout_sec=600`).
All FCU text messages (PreArm reasons, mode change failures) are re-logged by the
executor, so you do not have to read the MAVProxy console to see why arming fails.
`SET_MODE` requests `GUIDED` every `mode_retry_interval_sec` until `/mavros/state`
reports it, then `ARM` retries arming every `arm_retry_interval_sec` until ArduPilot
accepts.

Takeoff uses only MAVROS `/mavros/cmd/takeoff` (`MAV_CMD_NAV_TAKEOFF`). **No position
setpoints are published while arming, taking off or landing**: ArduCopter's GUIDED mode
abandons a running takeoff as soon as it receives a position target, the position
controller does nothing while the vehicle is still landed, and after `DISARM_DELAY` the
copter disarms on the ground. Takeoff is confirmed when local z reaches the waypoint Z
minus the hard tolerance. If the vehicle does not climb within
`takeoff_climb_timeout_sec`, the takeoff is re-sent. If it disarms on the ground, the
mode/arm/takeoff sequence is retried (`max_takeoff_attempts=3`).

Landing uses `/mavros/cmd/land` (LAND mode), waits for low-speed telemetry at ground
level, requests disarm if still armed, and waits until `/mavros/state.armed` is false.
For `landtakeoff` the executor then stays in `GROUND_WAIT` for `ground_wait_sec=3`,
re-enters `GUIDED`, re-arms, and takes off again to the waypoint Z before planning the
next leg. For `land` it declares `DONE`.

MAVROS may publish local pose/velocity more slowly than the setpoint loop. Before mode
and arming, the executor requests MAVLink `LOCAL_POSITION_NED` (message 32) at
`local_position_rate_hz=10`. The defaults tolerate pose gaps up to
`pose_stale_timeout_sec=5` and velocity gaps up to
`velocity_stale_timeout_sec=3`; once armed, longer pose loss still fails the mission.
Hard settling counts distinct low-speed velocity messages, so repeating one cached
sample at the control-loop rate cannot falsely satisfy the one-second settle interval.

## Flight logs, rosbag and plots

Every `./run_a1_2.sh` run gets its own folder `log/a1_2_runs/<date>_<time>_<mission>/`:

| File | Content |
|---|---|
| `mission.csv` | copy of the mission file that was flown |
| `flight_log.csv` | 20 Hz: state, waypoint, armed/mode, commanded and measured pose and yaw (map frame), speed |
| `planned_paths.csv` | every A* route that was flown (one row per route point) |
| `bag/` | rosbag (`A1_2_RECORD=1`, default): MAVROS state/pose/velocity/setpoints/FCU text, plan requests and results, markers |
| `topview.png` | floor plan: map, racks with safety radius, planned routes, flown track, heading arrows, waypoints and tasks |
| `overview.png` | altitude, heading, distance to target and speed, clearance to obstacles, arrival error per waypoint, deviation from the planned route |
| `view_3d_a1_2.py` | (window, not a file) 3D voxel view with the planned route |

The plots are made automatically when the mission ends (also after Ctrl+C or an error;
`A1_2_PLOT=0` disables it), and two interactive windows (top view, overview) open so you can
zoom in; they stay open after the script exits (`A1_2_SHOW_PLOTS=0` saves the PNGs only).
A third window, `view_3d_a1_2.py`, shows the run in 3D: the hangar as voxels, the planned A*
route as a glowing green tube inside a translucent safety-radius corridor, a green light that
travels along the route, the flown track (blue) and the waypoints. Mouse: rotate/zoom/pan.
Keys: `space` pause the light, `+`/`-` hide walls above a height, `c` roof, `i` the planner's
inflated occupancy, `f` flown track, `r` reset view, `q` quit. `A1_2_SHOW_3D=0` disables it;
open it later with `python3 view_3d_a1_2.py [RUN_FOLDER]`.
Gaps in the time plots are expected: during takeoff and landing ArduCopter flies on its own
and no setpoints are sent, and periods on the ground are labelled. To redo them: `python3 plot_a1_2.py [RUN_FOLDER]` (newest run by
default). The terminal summary lists the arrival error of each waypoint against its
tolerance, the minimum clearance to any wall, pillar or rack (floor excluded) compared with
the drone radius (0.33 m to the prop tips), and the largest deviation from the planned route.
`A1_2_RVIZ=1 ./run_a1_2.sh` also opens RViz with the map, racks, planned route and the flown
track (`/mission/flown_path`). The executor exits by itself after `DONE` (`exit_on_done`).

Navigation follows the A* route with a "carrot": the setpoint slides along each segment at
`cruise_speed` (1 m/s, scaled for a slow simulation) and stays at most `max_lead=1.0` m ahead
of the drone. Sending only the segment end lets ArduCopter mix its horizontal and vertical
speed limits and leave the straight line, for example descending before it has cleared a rack.

`safety_radius` in `config/planner.yaml` is 0.45 m: 0.33 m drone radius plus about 0.12 m for
tracking error. With 0.30 m the test flight came within 0.37 m of a rack (4 cm margin).

## Test missions

Besides `hangar_example.csv`, two missions cover other parts of the hangar:

* `missions/test_east_aisle.csv`: the open east half, the aisle between the racks, `yaw-90`,
  `yaw0`, `landtakeoff` south of the racks, and a final `land` away from the start.
* `missions/test_over_racks.csv`: a climb to 4.8 m over the lower rack, a hard waypoint 0.5 m
  from the north wall, `landtakeoff` in the east, and `yaw-45`.

Both completed end to end in SITL (all waypoints within tolerance, minimum clearance 0.61 m
and 0.38 m).

## Important first-flight checks

Before an unattended run, verify the local origin and takeoff altitude, the MAVROS
service response and landed/disarmed behavior, and the planner's readiness before
starting the executor. A timeout/fault stops the mission and holds the latest measured
pose; it does not attempt autonomous recovery. The final acceptance check is a complete
SITL run including collision clearance at the shelving racks, all mission tasks,
direction-of-travel heading, the `landtakeoff` waypoint, and final disarm.