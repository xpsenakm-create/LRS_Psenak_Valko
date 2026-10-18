"""Explanatory chapters of the study guide (HTML fragments)."""

HOW_TO = """
<p>This guide explains the A1.2 solution from the ground up, then walks through <b>every line of
the code</b> in blocks. The code shown is read from the repository when the guide is built, so
the line numbers match the files.</p>
<ul>
<li><b>Read chapters 2–7 first</b>: the system, one flight from start to end, the concepts, the
state machine, the bugs we fixed. Most exam questions are about these.</li>
<li>Then the <b>code chapters</b>: code on the left, explanation on the right. The executor
chapter is the core of A1.2; read it fully. The planner chapters are A1.1 (also used here).</li>
<li><b>Search</b> (top bar) filters every block and flashcard, e.g. <code>carrot</code>,
<code>GUIDED</code>, <code>DISARM_DELAY</code>, <code>octile</code>.</li>
<li><b>Quiz mode</b> blurs all explanations: read the code, say aloud what it does, then click to
check yourself.</li>
<li>The <b>flashcards</b> near the end are likely examiner questions with model answers.</li>
</ul>
<p>Rebuild after changing code: <code>python3 docs/study_guide/build_study_guide.py</code>.</p>
"""

BIG_PICTURE = """
<p>Five programs cooperate. Think of a <b>real drone</b>: there is a flight controller board on
it (ArduPilot) that keeps it stable and flies to a target you give it, and a companion computer
(our ROS 2 nodes) that decides where to go. In simulation, Gazebo plays the role of the real
world and the real sensors.</p>
<pre class="diagram">
   ┌──────────── simulation (LRS-URK scripts) ────────────┐
   │  Gazebo  ◄──── lock-step ────►  ArduCopter SITL      │      physics ↔ flight controller
   │  (world, physics,               (EKF, controllers,    │
   │   sensors)                       modes, arming)       │
   └────────────────────────────────────┬─────────────────┘
                                        │ MAVLink (UDP 14550)
                                   ┌────▼────┐
                                   │ MAVROS  │  MAVLink  ⇄  ROS 2 topics / services
                                   └────┬────┘
          /mavros/state, pose, velocity │  ▲  /mavros/setpoint_position/local (20 Hz)
          GPS, FCU texts                ▼  │  set_mode, arming, takeoff, land
                         ┌──────────────────────────┐   plan_request    ┌──────────────────┐
                         │  mission_executor_node   │ ────────────────► │ map_planner_node │
                         │  (A1.2: state machine)   │ ◄──────────────── │ (A1.1: A* on     │
                         └────────────┬─────────────┘  planned_path,    │  voxel map)      │
                                      │                planning_status   └──────────────────┘
                             flight_log.csv, rosbag → plot_a1_2.py, view_3d_a1_2.py
</pre>
<table>
<tr><th>Program</th><th>In one sentence</th></tr>
<tr><td>Gazebo</td><td>Simulates the hangar, the physics of the drone and its sensors (IMU, GPS).</td></tr>
<tr><td>ArduCopter SITL</td><td>The real autopilot firmware compiled for the PC: estimates the drone's state (EKF), stabilises it, flies to targets, checks if arming is safe.</td></tr>
<tr><td>MAVROS</td><td>A bridge: converts ArduPilot's MAVLink messages to ROS 2 topics/services and back.</td></tr>
<tr><td>map_planner_node</td><td>Turns the hangar point cloud into a 3D grid of blocked/free voxels (obstacles grown by a safety radius) and finds routes with A*.</td></tr>
<tr><td>mission_executor_node</td><td>Reads the mission, decides what to do 20× per second (state machine), sends targets and commands, logs everything.</td></tr>
</table>
<div class="box"><b>Who flies the drone?</b> ArduPilot does, always. Our executor never controls motors
or attitude. It only says "be at this position with this heading" (a <b>setpoint</b>) or calls a
command ("take off to 1.5 m", "land"). ArduPilot's own controllers do the rest. That is what the
<b>GUIDED</b> mode means: an external computer guides it with targets.</div>
"""

FLIGHT = """
<p>Real log lines from the <code>test_east_aisle</code> run, with what happens behind them.
Times are seconds after the executor started; the simulation ran at 0.25–0.3× real time.</p>
<table>
<tr><th>t [s]</th><th>Log line</th><th>What is going on</th></tr>
<tr><td>0.0</td><td><code>Mission state: IDLE → WAIT_FCU</code></td><td>Executor started, mission loaded and printed.</td></tr>
<tr><td>2.4</td><td><code>Coordinate frames: MAVROS local=(-7.00, 12.99, 0.00), planner map=(12.99, 7.00, 0.00)</code></td><td>First pose. Shows the 90° rotation between MAVROS's local frame and the map frame.</td></tr>
<tr><td>5–7</td><td><code>Waiting for FCU readiness: GPS fix not stable yet</code> → <code>FCU ready (GPS fix stable)</code></td><td>GPS fix held for 5 s.</td></tr>
<tr><td>7.7</td><td><code>SET_MODE</code>, <code>GUIDED mode request sent</code>, <code>ARM</code></td><td>/mavros/state confirms GUIDED; arming begins.</td></tr>
<tr><td>8–34</td><td>(repeated) <code>Arm attempt N rejected</code>, <code>FCU: Arm: Gyros inconsistent</code></td><td>ArduPilot's pre-arm check: both simulated IMUs must agree for ~10 s of <i>simulated</i> time; at 0.3× that takes ~30 s. The executor retries every 2 s.</td></tr>
<tr><td>34.6</td><td><code>FCU: Arming motors</code> → <code>TAKEOFF</code>, <code>MAVROS accepted NAV_TAKEOFF to 1.50 m</code></td><td>Armed. Takeoff command; <b>no setpoints</b> are sent now.</td></tr>
<tr><td>48.1</td><td><code>Takeoff confirmed: z=1.43 m (threshold 1.35 m)</code></td><td>Within 0.15 m of the target altitude.</td></tr>
<tr><td>48.1</td><td><code>PLAN → WAIT_PLAN → NAVIGATE</code>, <code>Reached soft waypoint 1: error=0.071 m</code></td><td>Waypoint 1 is directly above the start: a one-point route, already within the soft radius.</td></tr>
<tr><td>48.4</td><td><code>WAIT_PREFETCH → ALIGN_PATH</code></td><td>The route to waypoint 2 was requested in advance. First turn towards the route.</td></tr>
<tr><td>53.8</td><td><code>Aligned with path heading 53.1 deg</code> → <code>NAVIGATE</code></td><td>Facing the direction of travel; the carrot starts sliding along the route.</td></tr>
<tr><td>74.0</td><td><code>Settled at hard waypoint 2: error=0.041 m, speed=0.037 m/s for 4.00 s</code></td><td>Hard waypoint: within 0.15 m and slow for the settle time (1 s, ×4 for the slow simulation).</td></tr>
<tr><td>83.3</td><td><code>Completed task 'yaw' at waypoint 2</code></td><td><code>yaw-90</code> reached and held.</td></tr>
<tr><td>97.2</td><td><code>Reached soft waypoint 3: error=0.336 m; continuing without settling</code></td><td>Soft: passed 0.34 m away, no stop.</td></tr>
<tr><td>143.9</td><td><code>Aligned with path heading -540.0 deg</code></td><td>Yaw is <b>unwrapped</b>: −540° is the same direction as 180°. The executor keeps the commanded yaw continuous so the drone never spins the long way round.</td></tr>
<tr><td>~191</td><td>waypoint 6 <code>landtakeoff</code>: <code>LAND → WAIT_LANDED → Touchdown → WAIT_DISARMED → FCU: Disarming motors → GROUND_WAIT → SET_MODE → ARM → RETAKEOFF</code></td><td>Land; ArduCopter disarms itself; wait on the ground; GUIDED again; arm; take off; continue.</td></tr>
<tr><td>~271</td><td>waypoint 8 <code>land</code>: <code>... Disarm confirmed on ground</code>, <code>Completed task 'land'</code>, <code>DONE</code></td><td>Final landing and disarm.</td></tr>
<tr><td>+1 s</td><td><code>Mission DONE; executor exits</code></td><td>The node shuts down; the run script stops the rosbag, plots, opens the windows.</td></tr>
</table>
"""

CONCEPTS = """
<h3>ROS 2</h3>
<table>
<tr><th>Term</th><th>Meaning here</th></tr>
<tr><td>Node</td><td>A program that talks ROS. We have two: <code>mission_executor_node</code>, <code>map_planner_node</code> (plus MAVROS).</td></tr>
<tr><td>Topic</td><td>A named stream of messages, one-way, many-to-many (publish/subscribe). Example: MAVROS publishes <code>/mavros/local_position/pose</code>; we subscribe.</td></tr>
<tr><td>Service</td><td>A request → response call. Example: <code>/mavros/cmd/arming</code> with <code>value=true</code> answers <code>success</code>. We call them <b>asynchronously</b> (we do not block waiting).</td></tr>
<tr><td>Parameter</td><td>A named setting of a node, set at start (<code>-p name:=value</code> or a YAML file).</td></tr>
<tr><td>QoS</td><td>Delivery rules. <b>Reliable</b> (retransmit) vs <b>best effort</b> (drop is fine); <b>transient local</b> = late subscribers get the last message. Incompatible QoS = no messages at all.</td></tr>
<tr><td>Executor</td><td>Runs the callbacks. <code>MultiThreadedExecutor</code> uses several threads.</td></tr>
<tr><td>Callback group</td><td>Decides which callbacks may run in parallel. MutuallyExclusive = one at a time within the group.</td></tr>
<tr><td>Mutex</td><td>A lock: only one thread at a time may touch the shared data. <code>std::lock_guard</code> locks for one scope.</td></tr>
</table>

<h3>MAVROS and MAVLink</h3>
<p><b>MAVLink</b> is the binary message protocol of ArduPilot/PX4 (e.g. HEARTBEAT, LOCAL_POSITION_NED,
COMMAND_LONG, SET_POSITION_TARGET_LOCAL_NED, STATUSTEXT). <b>MAVROS</b> converts them to ROS and
back. It also converts frames: MAVLink uses <b>NED</b>, ROS uses <b>ENU</b>.</p>

<h3>ArduCopter</h3>
<table>
<tr><th>Term</th><th>Meaning</th></tr>
<tr><td>GUIDED mode</td><td>The copter flies to targets sent by an external computer. Position targets are <b>latched</b> (kept until a new one comes); velocity/attitude targets time out after <code>GUID_TIMEOUT</code>.</td></tr>
<tr><td>LAND mode</td><td>Descends, detects touchdown, disarms by itself.</td></tr>
<tr><td>Arming</td><td>Motors enabled. Refused if <b>pre-arm checks</b> fail (IMUs inconsistent, no position estimate, ...). The reason arrives as a STATUSTEXT (<code>PreArm: ...</code>/<code>Arm: ...</code>).</td></tr>
<tr><td>DISARM_DELAY</td><td>If armed but sitting on the ground with no takeoff, the copter disarms after 10 s.</td></tr>
<tr><td>EKF</td><td>Extended Kalman Filter: fuses IMU, GPS, compass, barometer into position/velocity/attitude. "EKF3 IMU0 is using GPS" = it has an absolute position.</td></tr>
<tr><td>NAV_TAKEOFF</td><td>Command: climb to an altitude above home. In GUIDED, a position setpoint during the climb cancels it.</td></tr>
<tr><td>SITL, lock-step, RTF</td><td>Software In The Loop: firmware runs on the PC. Lock-step: SITL and Gazebo advance together, step by step, so if Gazebo pauses, SITL freezes ("no link"). RTF (real-time factor) = simulated seconds per real second; this PC: 0.2–0.3.</td></tr>
</table>

<h3>Frames and angles</h3>
<ul>
<li><b>ENU</b>: x east, y north, z up (ROS). <b>NED</b>: x north, y east, z down (ArduPilot/MAVLink). MAVROS converts; we only see ENU.</li>
<li><b>map frame</b> (planner, mission file, Gazebo world): rotated 90° from MAVROS's local ENU: map x = north, map y = west. Our code converts at the boundary (<code>localToMapPose</code>, <code>mapToLocalPose</code>).</li>
<li><b>Yaw</b>: rotation about z. Ours: absolute in the map frame, 0° = +x, counter-clockwise positive.</li>
<li><b>Quaternion</b>: (x, y, z, w) representation of a rotation; yaw only: (0, 0, sin ψ/2, cos ψ/2).</li>
<li><b>Wrapping</b> keeps an angle in [−180°, 180°]; <b>unwrapping</b> picks the equivalent angle closest to the previous one, so commands never jump by 360°.</li>
</ul>

<h3>Planning</h3>
<ul>
<li><b>Point cloud</b>: the hangar scanned as 620 k 3D points (<code>map.pcd</code>).</li>
<li><b>Voxel</b>: a 0.15 m cube. Occupied if a map point falls into it.</li>
<li><b>Inflation</b>: grow every obstacle voxel by a sphere of the safety radius (0.45 m). Then the drone can be treated as a point: any free voxel is safe for its centre.</li>
<li><b>A*</b>: best-first graph search with f = g + h; with an admissible, consistent h the result is the shortest path on the grid.</li>
<li><b>26-connectivity</b>: each voxel has 26 neighbours (faces, edges, corners); steps cost 1, √2, √3 voxels.</li>
<li><b>Octile heuristic</b>: the exact path length in empty space on that grid.</li>
<li><b>Line-of-sight simplification</b>: replace the voxel staircase by the fewest straight segments that stay free.</li>
</ul>

<h3>Control</h3>
<ul>
<li><b>Setpoint</b>: the position + yaw target streamed at 20 Hz to <code>/mavros/setpoint_position/local</code>.</li>
<li><b>Carrot</b>: a setpoint that slides along the route ahead of the drone (max 1 m ahead), so the drone follows the straight line.</li>
<li><b>Hard</b> waypoint: stop within 0.15 m and below 0.2 m/s for 1 s. <b>Soft</b>: pass within 0.55 m.</li>
<li><b>Pass radius</b>: intermediate route points are taken within 0.45 m, without stopping.</li>
</ul>
"""

STATE_MACHINE = """
<pre class="diagram">
IDLE ─► WAIT_FCU ─► SET_MODE ─► ARM ─► TAKEOFF ─► PLAN ─► WAIT_PLAN ─► ALIGN_PATH ◄──┐
           (GPS/EKF ok)  (GUIDED)  (armed)  (altitude)            (route ok)    │        │
                                                                                 ▼        │ sharp corner
                                                                             NAVIGATE ────┘
                                                                                 │ waypoint reached
                                                                                 ▼ (hard: settled)
              ┌──────────────────────── WAIT_PREFETCH ◄──────────────────── TASK ──► DONE (last, no land)
              │ (next route ready)                                            │
              └──► ALIGN_PATH ...                                             │ land / landtakeoff
                                                                              ▼
                     DONE ◄── (land) ── WAIT_DISARMED ◄── WAIT_LANDED ◄──── LAND
                                              │ (landtakeoff)
                                              ▼
                                         GROUND_WAIT ─► SET_MODE ─► ARM ─► RETAKEOFF ─► next waypoint
any state ──(timeout, refused service, lost link, unexpected disarm, mode change)──► ERROR (hold position)
</pre>
<p>Rules that hold in every state (checked at the top of <code>tick()</code>):</p>
<ul>
<li>a service without an answer for 5 s → ERROR;</li>
<li>a state lasting longer than its timeout → ERROR;</li>
<li>armed but no pose for 5 s → ERROR ("telemetry stale");</li>
<li>in flight states: link lost → ERROR; disarmed → ERROR (or a takeoff retry on the ground);
mode not GUIDED → ERROR.</li>
</ul>
<p>Setpoints are streamed only in PLAN, WAIT_PLAN, WAIT_PREFETCH, ALIGN_PATH, NAVIGATE, TASK and
ERROR.</p>
"""

BUGS = """
<p>What did not work at the start, why, and how it was fixed. Examiners like "what problems did
you have?" questions.</p>
<h4>1. "Arms, then disarms, never takes off"</h4>
<p><b>Cause:</b> the executor streamed position setpoints during takeoff. In GUIDED mode a position
target cancels ArduCopter's takeoff; position control does not lift a landed copter; after
DISARM_DELAY it disarms. <b>Fix:</b> <code>setpointsAllowed()</code>, no setpoints in
SET_MODE/ARM/TAKEOFF/LAND.</p>
<h4>2. "Takes way too long to arm"</h4>
<p><b>Cause:</b> the run script waited for <code>pre-arm good</code> / EKF lines in the SITL log, but
MAVProxy's <code>--console</code> prints them only in its GUI window, so the wait always hit
its 240 s timeout (plus 60 s for the boot check). <b>Fix:</b> the script waits only for MAVProxy's
own lines; the executor checks readiness through MAVROS (GPS fix, FCU texts) and logs the PreArm
reasons. Startup went from ~5 min to ~1.5 min.</p>
<h4>3. Mission died at the landtakeoff waypoint</h4>
<p><b>Cause:</b> after touchdown the executor sent a disarm before ArduCopter's land detector agreed;
the FCU refused, and the refusal was treated as fatal. <b>Fix:</b> wait for LAND mode to disarm by
itself; send our own disarm only after a grace period and retry it.</p>
<h4>4. Too close to a rack</h4>
<p><b>Cause:</b> <code>safety_radius</code> was 0.30 m, smaller than the drone (0.33 m to the prop
tips); the clearance plot showed 0.37 m to a rack (4 cm margin). <b>Fix:</b> 0.45 m; the same route
then kept 0.61 m.</p>
<h4>5. Drone left the straight line</h4>
<p><b>Cause:</b> the setpoint jumped to the segment end; ArduCopter mixed different horizontal and
vertical speed limits (0.56 m off the route, descending early). <b>Fix:</b> carrot following.</p>
<h4>6. Not a bug: "no link" in MAVProxy</h4>
<p>Gazebo was paused with its ⏸ button; because of lock-step, SITL froze too. Do not click in
Gazebo during a flight.</p>
"""

GAPS = """
<p>In the overview plot some lines stop for a while (for example around 200–250 s in
<code>test_over_racks</code>). Nothing was lost. Here is the simple version:</p>
<div class="box"><b>Analogy</b>
A car's cruise control graph shows "set speed" and "actual speed". While the car is parked, there
is no set speed, because cruise control is off. The "set speed" line stops; the "actual speed"
line continues at zero.</div>
<ul>
<li>During <b>takeoff</b>, <b>landing</b> and the time <b>on the ground</b> in a
<code>landtakeoff</code>, the executor sends <b>no setpoint</b> on purpose (bug 1). ArduCopter
flies these phases by itself.</li>
<li>No setpoint means "commanded z", "commanded yaw" and "distance to target" do not exist, so
their lines stop. The log column <code>cmd_valid = 0</code> marks those rows.</li>
<li>"Deviation from the route" only exists while flying a route (NAVIGATE/ALIGN).</li>
<li>The <b>measured</b> lines (altitude, heading, speed) continue. Clearance on the ground is
drawn grey. Every such period is labelled "on ground (landtakeoff)" or "landed (end)".</li>
</ul>
"""

NUMBERS = """
<table>
<tr><th>Value</th><th>Number</th><th>Where / why</th></tr>
<tr><td>Control / setpoint rate</td><td>20 Hz</td><td>assignment: 10–20 Hz</td></tr>
<tr><td>Hard tolerance</td><td>0.15 m, &lt; 0.2 m/s for 1 s</td><td>assignment suggests 10–20 cm</td></tr>
<tr><td>Soft tolerance</td><td>0.55 m</td><td>pass through</td></tr>
<tr><td>Pass radius (route points)</td><td>0.45 m</td><td>no stop at intermediate points</td></tr>
<tr><td>Turn-in-place threshold</td><td>20°</td><td>ALIGN_PATH at sharper corners</td></tr>
<tr><td>Yaw reached</td><td>5° for 0.5 s</td><td>yaw tasks and alignment</td></tr>
<tr><td>Carrot</td><td>1 m/s, ≤ 1 m ahead</td><td>straight-line tracking</td></tr>
<tr><td>Voxel size</td><td>0.15 m</td><td>map and A* resolution</td></tr>
<tr><td>Safety radius</td><td>0.45 m</td><td>0.33 drone + 0.12 tracking</td></tr>
<tr><td>Snap radius</td><td>5 voxels = 0.75 m</td><td>blocked start/goal</td></tr>
<tr><td>Map ↔ local rotation</td><td>90°</td><td>map x = north</td></tr>
<tr><td>GPS stable</td><td>5 s</td><td>before GUIDED/arm</td></tr>
<tr><td>Ground wait / disarm grace</td><td>3 s / 5 s</td><td>landtakeoff</td></tr>
<tr><td>Racks</td><td>6.50 × 1.05 × 3.76 m at (4.32, 6.82) and (4.32, 11.36)</td><td>planner.yaml</td></tr>
<tr><td>Drone radius</td><td>0.33 m</td><td>centre to prop tip (body 0.35 m square, props at ±0.15 m)</td></tr>
<tr><td>Measured hard arrival error</td><td>2.9–7.6 cm</td><td>three missions with logging</td></tr>
<tr><td>Measured min clearance</td><td>0.37 / 0.61 / 0.38 m</td><td>hangar_example (final) / east aisle / over racks</td></tr>
<tr><td>Voxel quantization</td><td>≈ 0.075 m</td><td>real clearance can be 0.45 − 0.075 ≈ 0.37 m</td></tr>
</table>
"""

GLOSSARY = """
<dl class="gloss">
<dt>Admissible heuristic</dt><dd>Never overestimates the remaining cost; A* is then optimal.</dd>
<dt>Consistent heuristic</dt><dd>h(a) ≤ cost(a, b) + h(b); no voxel is expanded twice.</dd>
<dt>Asynchronous service call</dt><dd>Send the request, continue; a callback handles the answer later.</dd>
<dt>Callback</dt><dd>A function the executor calls when something happens (message, timer, service answer).</dd>
<dt>Carrot following</dt><dd>Steering towards a moving point on the path a little ahead of the vehicle.</dd>
<dt>Clearance</dt><dd>Distance from the drone centre to the nearest obstacle point.</dd>
<dt>EKF</dt><dd>Extended Kalman Filter, ArduPilot's state estimator.</dd>
<dt>FCU</dt><dd>Flight Control Unit, the autopilot (here ArduCopter SITL).</dd>
<dt>GUIDED</dt><dd>ArduCopter mode that follows external targets.</dd>
<dt>Inflation</dt><dd>Growing obstacles by the safety radius.</dd>
<dt>k-d tree</dt><dd>Spatial index for fast nearest-neighbour queries (used for clearance).</dd>
<dt>Lazy deletion</dt><dd>Leaving outdated entries in the priority queue and skipping them when popped.</dd>
<dt>Lock-step</dt><dd>Simulator and firmware advance one step at a time together.</dd>
<dt>MAVLink</dt><dd>Message protocol between autopilot and ground/companion software.</dd>
<dt>MAVROS</dt><dd>ROS package bridging MAVLink and ROS.</dd>
<dt>Prefetch</dt><dd>Requesting the next leg's route while still flying the current one.</dd>
<dt>QoS</dt><dd>Quality of Service, delivery rules of a ROS topic.</dd>
<dt>RAII</dt><dd>Resource Acquisition Is Initialisation: resources (locks, files) are released by destructors, e.g. <code>lock_guard</code>.</dd>
<dt>RTF</dt><dd>Real-time factor of the simulation.</dd>
<dt>Setpoint</dt><dd>A target value sent to a controller.</dd>
<dt>SITL</dt><dd>Software In The Loop.</dd>
<dt>Snapping</dt><dd>Moving a blocked start/goal to the nearest free voxel.</dd>
<dt>STATUSTEXT</dt><dd>MAVLink text message from the autopilot (PreArm reasons etc.).</dd>
<dt>Transient local</dt><dd>QoS durability: late subscribers receive the last message.</dd>
<dt>Unwrap</dt><dd>Choosing the equivalent angle nearest to the previous one.</dd>
<dt>Voxel</dt><dd>A cube cell of a 3D grid.</dd>
</dl>
"""

BEFORE_CODE = [
    ("howto", "How to use this guide", HOW_TO),
    ("bigpicture", "The big picture", BIG_PICTURE),
    ("flight", "One flight, step by step (real log)", FLIGHT),
    ("concepts", "Concepts you must know", CONCEPTS),
    ("statemachine", "The state machine at a glance", STATE_MACHINE),
    ("bugs", "Problems we hit and how they were fixed", BUGS),
    ("gaps", "Why some plot lines have gaps", GAPS),
]
AFTER_CODE = [
    ("numbers", "Numbers to remember", NUMBERS),
    ("glossary", "Glossary", GLOSSARY),
]
