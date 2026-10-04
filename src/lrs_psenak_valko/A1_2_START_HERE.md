# A1.2 Mission Execution Starter

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
targets). Hard arrival also requires speed below `0.20 m/s` continuously for `1.0 s`;
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
manual conversion. `yaw_tolerance_deg=5` must be held for `yaw_hold_time_sec=0.5` before
a yaw task completes, with a `yaw_timeout_sec=15` limit. Each route aligns to its first
travel segment before translating; at bends of at least
`turn_alignment_threshold_deg=20`, it holds near the corner and aligns before the next
segment. Smaller changes are followed continuously so the drone does not stop at every
planner point. After a yaw task, the next route's travel heading is reacquired before
movement begins.

## Build and run

The supplied checkout contains the ROS package but not the upstream Gazebo world,
models, or `scripts/run_*.sh`. In a checkout of `KocurMaros/LRS-URK`, start the
simulation manually in three terminals as described by its README:

1. `cd ~/LRS-URK && scripts/run_gazebo.sh`
2. `cd ~/LRS-URK && scripts/run_sitl.sh`
3. `cd ~/LRS-URK && scripts/run_mavros.sh`

Build this workspace, source it in each ROS terminal, then start the planner and
executor in two additional terminals:

```bash
cd ~/Zadania/LRS_Psenak_Valko
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lrs_psenak_valko
source install/setup.bash
```

Planner terminal (run from the workspace root so its generated occupancy file lands
here):

```bash
ros2 run lrs_psenak_valko map_planner_node --ros-args \
  --params-file src/lrs_psenak_valko/config/planner.yaml \
  -p occupancy_pcd_path:=$PWD/map_voxels_with_racks_inflated.pcd
```

Executor terminal:

```bash
ros2 run lrs_psenak_valko mission_executor_node --ros-args \
  -p mission_file:=$PWD/src/lrs_psenak_valko/missions/hangar_example.csv
```

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

Takeoff is complete only after MAVROS accepts the `CommandTOL` request, the FCU still
reports armed, and fresh local-pose telemetry confirms a climb of at least 0.5 m and
altitude within the 0.15 m hard tolerance of the requested waypoint Z. Landing waits
for fresh low-speed telemetry at ground level, requests disarm if still armed, and
waits until `/mavros/state.armed` is false before declaring `DONE`. For
`landtakeoff`, that confirmed ground/disarmed state is followed by a `GUIDED` mode
check, re-arm confirmation, and a new acknowledged takeoff before navigation resumes.

## Important first-flight checks

This is a compiling A1.2 implementation, not a recorded or validated complete flight.
Before an unattended run, verify the local origin and takeoff altitude, the MAVROS
service response and landed/disarmed behavior, and the planner's readiness before
starting the executor. A timeout/fault stops the mission and holds the latest measured
pose; it does not attempt autonomous recovery. The final acceptance check is a complete
SITL run including collision clearance at the shelving racks, all mission tasks,
direction-of-travel heading, the `landtakeoff` waypoint, and final disarm.