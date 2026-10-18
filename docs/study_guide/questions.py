"""Likely examiner questions with model answers (rendered as flashcards)."""

import html

QA = [
# ---------------------------------------------------------------- system
("System", "What does each of the three simulation terminals run, and what are their arguments?",
 """<ul><li><code>run_gazebo.sh</code>: <code>gz sim -r -v 4 fei_lrs_gazebo.world</code>. <code>-r</code>
 = start running (not paused), <code>-v 4</code> = verbose log; the world is the hangar with racks
 and the drone.</li>
 <li><code>run_sitl.sh</code>: <code>sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON
 --add-param-file=gazebo-iris.parm --console -l &lt;lat,lon,alt,heading&gt;</code>. ArduCopter
 firmware; <code>--model JSON</code> = talk to Gazebo's ArduPilot plugin; the param file sets the
 frame (otherwise it will not arm); <code>--console</code> = MAVProxy console window;
 <code>-l</code> = home location (FEI hangar).</li>
 <li><code>run_mavros.sh</code>: <code>mavros_node</code> with
 <code>fcu_url:=udp://0.0.0.0:14550@</code>: listen on UDP 14550, where MAVProxy forwards
 MAVLink.</li></ul>"""),
("System", "Who actually controls the motors?",
 """<p>ArduPilot. Our node only sends position/yaw targets and commands (GUIDED mode, arm, takeoff,
 land). ArduPilot's EKF estimates the state and its controllers compute motor outputs.</p>"""),
("System", "Why ROS 2 + MAVROS instead of talking MAVLink directly?",
 """<p>MAVROS handles the MAVLink connection, heartbeats, command acknowledgements and the
 NED↔ENU conversion, and exposes everything as ordinary ROS topics/services. Our node is then a
 normal ROS node that can talk to the ROS planner and be recorded with rosbag.</p>"""),
# ---------------------------------------------------------------- state machine
("State machine", "List the states of your state machine.",
 """<p>IDLE, WAIT_FCU, SET_MODE, ARM, TAKEOFF, PLAN, WAIT_PLAN, WAIT_PREFETCH, ALIGN_PATH,
 NAVIGATE, TASK, LAND, WAIT_LANDED, WAIT_DISARMED, GROUND_WAIT, RETAKEOFF, DONE, ERROR. The
 assignment's IDLE → SET_MODE → ARM → TAKEOFF → NAVIGATE → TASK → … → LAND → DONE is the backbone;
 the others are waiting states and landtakeoff.</p>"""),
("State machine", "How do you check service responses?",
 """<p>Every call is <code>async_send_request</code> with a callback. The callback checks the
 response (<code>mode_sent</code>, <code>success</code>) and retries or fails. A request id ignores
 answers that belong to a previous state; a pending call older than 5 s fails the mission. For
 GUIDED and arming, the decision to move on is taken only from <code>/mavros/state</code>, the
 FCU's real state.</p>"""),
("State machine", "What happens if something goes wrong in the air?",
 """<p><code>fail()</code> logs the reason and enters ERROR, where the target becomes the current
 measured position, so the copter hovers in place (GUIDED position targets are latched). Triggers:
 state timeout, refused service, no answer, link lost, telemetry stale, unexpected disarm, mode
 changed away from GUIDED (pilot override: we do not fight it).</p>"""),
("State machine", "Why is all the logic in tick() and not in the callbacks?",
 """<p>One place makes decisions at a fixed rate with a consistent snapshot of the data (the mutex
 is held for the whole tick). Callbacks only store data. This avoids races and makes the behaviour
 deterministic and easy to log.</p>"""),
("State machine", "How are state transitions logged?",
 """<p><code>transition()</code> logs <code>Mission state: X</code> through RCLCPP_INFO, which
 carries a timestamp. The CSV log also has the state in every 20 Hz row.</p>"""),
# ---------------------------------------------------------------- threading
("Threads", "Explain the callback groups and the executor.",
 """<p>Two MutuallyExclusive groups: subscriptions in <code>state_group_</code>, the 20 Hz timer
 in <code>timer_group_</code>, run by a <code>MultiThreadedExecutor</code>. The timer never waits
 behind a burst of messages. Because callbacks can then run in parallel, all shared members are
 protected by one mutex. Same pattern as <code>mt_executor_demo</code>.</p>"""),
("Threads", "Could the mutex cause a deadlock?",
 """<p>No. There is only one mutex, and nothing waits for anything while holding it: service calls
 are asynchronous, so <code>tick()</code> sends and returns. The response callback takes the mutex
 later, in another thread.</p>"""),
("Threads", "Why steady_clock and not ROS time?",
 """<p>Timeouts and retries must use a clock that never jumps. The node does not use sim time; on
 a slow simulation the run script scales the timeouts by the measured real-time factor.</p>"""),
# ---------------------------------------------------------------- takeoff / land
("Takeoff & land", "Why are no setpoints sent during takeoff?",
 """<p>In GUIDED, a position target makes ArduCopter abandon NAV_TAKEOFF and switch to position
 control, which does not lift a landed copter. The motors idle, and after DISARM_DELAY (10 s) it
 disarms. That was our first bug. Setpoints are only streamed in flight states.</p>"""),
("Takeoff & land", "How is 'airborne' confirmed?",
 """<p>Local z ≥ max(start + 0.5 m, target − 0.15 m) while armed. If it does not climb within
 <code>takeoff_climb_timeout_sec</code>, NAV_TAKEOFF is re-sent; if it disarms on the ground,
 GUIDED → arm → takeoff is retried (max 3).</p>"""),
("Takeoff & land", "Describe landtakeoff.",
 """<p>LAND command → wait for touchdown (z ≤ 0.15 m, speed ≤ 0.15 m/s) → wait for LAND mode to
 disarm (fallback disarm after 5 s, retried) → stay on the ground 3 s → GUIDED again (the copter is
 in LAND) → arm → takeoff to the waypoint's z → next waypoint.</p>"""),
("Takeoff & land", "Why did the disarm get refused, and how did you fix it?",
 """<p>ArduCopter accepts a disarm only when its own land detector says "landed", about a second
 after our touchdown condition. We now wait for LAND mode to disarm by itself and only then send our
 own disarm, retrying instead of failing.</p>"""),
("Takeoff & land", "What does 'ArduPilot may disarm after landing' mean for you?",
 """<p>It always does in LAND mode. So after a landtakeoff we must re-enter GUIDED and arm again;
 the state machine goes GROUND_WAIT → SET_MODE → ARM → RETAKEOFF.</p>"""),
# ---------------------------------------------------------------- navigation
("Navigation", "How is the route between two mission waypoints obtained?",
 """<p>The executor publishes <code>plan_request</code> (PoseArray: start, goal). The planner runs
 A* and answers on <code>planned_path</code> + <code>planning_status</code>. The first leg starts at
 the current pose; later legs are prefetched from waypoint to waypoint while flying.</p>"""),
("Navigation", "What is prefetching and why?",
 """<p>While flying to waypoint i, the route i → i+1 is already requested. When the drone arrives,
 the route is ready (WAIT_PREFETCH is usually instant), so there is no hovering while A* runs.</p>"""),
("Navigation", "What is carrot following and why did you add it?",
 """<p>The setpoint is a point that moves along the planned segment at cruise speed, never more
 than 1 m ahead of the drone. Before, only the segment end was sent; ArduCopter then used
 different horizontal and vertical speed limits and left the line by up to 0.56 m. The A* route
 is only collision-free on the line, which matters most when climbing over a rack.</p>"""),
("Navigation", "Does the drone stop at every intermediate point?",
 """<p>No. Route points are taken within 0.45 m without stopping. It only stops to turn at corners
 sharper than 20° (ALIGN_PATH), and at hard waypoints. A* simplification leaves few points.</p>"""),
("Navigation", "What happens if the planner moves the goal?",
 """<p>If the goal is inside the inflated space, A* snaps it to the nearest free voxel (≤ 0.75 m).
 The executor compares the route end with the mission goal: further than the waypoint tolerance
 (0.15 hard / 0.55 soft) = fail, because we would not really reach the waypoint.</p>"""),
# ---------------------------------------------------------------- precision
("Precision", "How do you define 'arrived' for hard and soft waypoints? Justify the values.",
 """<p>Hard: within 0.15 m and speed ≤ 0.2 m/s continuously for 1 s, from at least 2 distinct
 velocity messages. Soft: within 0.55 m, no stop. 0.15 m is in the suggested 10–20 cm range and
 is reached reliably in SITL (2.9–9.7 cm measured over all runs). 0.55 m lets the drone pass smoothly while
 already turning to the next leg. Tolerances never act as collision margins; the safety radius
 does.</p>"""),
("Precision", "Why count distinct velocity messages?",
 """<p>The loop runs at 20 Hz but velocity may arrive at ~10 Hz or less. Without the sequence
 counter, one old slow message seen 20 times could fake "1 s stable".</p>"""),
# ---------------------------------------------------------------- yaw / frames
("Yaw & frames", "What is your yaw convention?",
 """<p>Absolute heading in the map frame, degrees, counter-clockwise positive, 0° = map +x axis.
 <code>yaw90</code> faces map +y; <code>yaw-45</code> is 45° clockwise from +x. Not relative to the
 current heading.</p>"""),
("Yaw & frames", "ENU vs NED: where do you convert?",
 """<p>Nowhere. MAVROS converts NED↔ENU. We only convert between MAVROS's local ENU and the
 planner's map frame, which are rotated by 90° about z.</p>"""),
("Yaw & frames", "How do you know the map frame is rotated by 90°?",
 """<p>At start MAVROS reports local (−7.00, 12.99) for a drone that spawns at (12.99, 7.00) in the
 Gazebo world, which is the map frame (the rack boxes in the world match the map racks).
 x_map = y_local, y_map = −x_local, so the offset is 90°.</p>"""),
("Yaw & frames", "How does the drone face the direction of travel without spinning?",
 """<p>Target yaw = <code>atan2(Δy, Δx)</code> of the current segment, unwrapped near the previous
 yaw (<code>unwrapNear</code>), so it always takes the turn of at most 180°. That is why the log
 can show "−540°": it is the continuous (unwrapped) value, the same direction as 180°.</p>"""),
("Yaw & frames", "How is yaw sent to ArduPilot?",
 """<p>As the orientation of the position setpoint: quaternion (0, 0, sin ψ/2, cos ψ/2) with
 ψ = map yaw + 90°. Position + yaw targets are latched in GUIDED, so no velocity stream is needed
 and GUID_TIMEOUT does not apply.</p>"""),
# ---------------------------------------------------------------- planner
("Planner", "How is the map built?",
 """<p>620 k points → PCL VoxelGrid at 0.15 m → sparse hash set of occupied voxels → rack boxes
 filled solid (the cloud only has their surfaces) → every voxel inflated by a 0.45 m sphere →
 copied into a dense 3D array for A*.</p>"""),
("Planner", "Why fill the racks?",
 """<p>The point cloud contains only the shelf surfaces; between the levels it looks free. Without
 filling, A* would plan through the shelves. Filling uses the real collision box sizes.</p>"""),
("Planner", "What heuristic do you use and why is it admissible?",
 """<p>The 3D octile distance <code>res·[(√3 − √2)·d1 + (√2 − 1)·d2 + d3]</code>, d sorted ascending.
 It is the exact cost of the cheapest path in empty space on a 26-connected grid. Obstacles can only
 make paths longer, so it never overestimates. It is also consistent, so the first time the goal
 is popped the path is optimal and no node is re-expanded.</p>"""),
("Planner", "Why not Manhattan or Euclidean distance?",
 """<p>Manhattan overestimates when diagonal moves are allowed (one diagonal step costs √2 but
 Manhattan counts 2): inadmissible, may give non-optimal paths. Euclidean is admissible but smaller
 than octile, so A* expands more voxels.</p>"""),
("Planner", "What is the corner-cutting check?",
 """<p>A diagonal move is allowed only if every voxel in the box it crosses is free; otherwise the
 path could slip between two blocked voxels that only touch at an edge/corner.</p>"""),
("Planner", "How do you guarantee termination?",
 """<p>Each voxel is closed at most once, so the open list empties (NO_PATH) if the goal is
 unreachable; a time limit (5 s) gives TIMEOUT on huge maps.</p>"""),
("Planner", "What does path simplification do, and is it safe?",
 """<p>Greedy line of sight: from each point jump to the farthest later point visible in the
 inflated map (segment sampled every quarter voxel). Every final segment is checked again before
 publishing. It is safe because the checks use the inflated map.</p>"""),
("Planner", "What is A*'s complexity?",
 """<p>O(E log V) with a binary heap; V = voxels expanded (at most all free ones), E ≤ 26·V. Memory:
 g, parent and closed per voxel (~9 bytes).</p>"""),
# ---------------------------------------------------------------- safety
("Safety", "How did you choose the safety radius?",
 """<p>Drone radius 0.33 m (body 0.35 m square, propellers at ±0.15 m) + ~0.12 m tracking error =
 0.45 m. It was confirmed by measurement: with the old 0.30 m the closest approach to a rack was
 0.37 m (4 cm from the props); with 0.45 m it was 0.61 m.</p>"""),
("Safety", "How do you prove there were no collisions?",
 """<p><code>plot_a1_2.py</code> computes, for every logged pose, the distance to the nearest map
 point (walls, pillars, trusses, floor excluded) or rack surface with a k-d tree, and reports the
 minimum against the drone radius. All logged missions stayed above 0.33 m (minimum 0.37 m, over
 the rack). The final recording is in <code>results/</code>.</p>"""),
("Safety", "Your safety radius is 0.45 m, so why was the clearance only 0.37 m?",
 """<p>Voxel quantization. Inflation grows spheres around voxel centres, but the real obstacle
 surface can be up to half a voxel (0.075 m) closer than the centre of its voxel. A route at the
 edge of the inflated space can therefore be ≈ 0.45 − 0.075 ≈ 0.37 m from the real surface. The
 drone followed the route exactly there. For a guaranteed 0.45 m, use 0.45 + half a voxel ≈ 0.53 m.</p>"""),
("Safety", "What is the weakest point of your solution?",
 """<p>Clearance over the racks: 0.37–0.38 m, only 4–5 cm more than the drone radius, from voxel
 quantization and, in <code>test_over_racks</code>, from the drone sinking ~0.15 m below its route on a
 descent. Options: a larger safety radius (0.5 m) at the cost
 of rejecting waypoints near walls, or a lower cruise speed on descents.</p>"""),
# ---------------------------------------------------------------- run / test
("Running", "How do you run an arbitrary mission file at the defence?",
 """<p><code>./run_a1_2.sh /path/to/mission.csv</code>. The parser validates it first; the
 executor plans every leg with A*.</p>"""),
("Running", "What does the run script check before starting the executor?",
 """<p>Gazebo publishes world stats (and its real-time factor), SITL booted, MAVROS has the
 set_mode/arming/takeoff/land services, /mavros/state is connected, a local pose arrives, and the
 planner node exists. GPS/EKF readiness is left to the executor.</p>"""),
("Running", "Why does arming take 20–40 s?",
 """<p>ArduPilot's pre-arm check needs the two simulated IMUs to agree for about 10 s of
 <i>simulated</i> time ("Gyros inconsistent" before that). At a real-time factor of 0.25–0.3 that is
 30–40 s of wall time. The executor retries every 2 s and logs the reason.</p>"""),
("Running", "What is in the rosbag?",
 """<p>/mavros/state, local pose and velocity, setpoints, FCU texts, plan requests, planned paths,
 planning status and the markers: enough to replay and plot the mission.</p>"""),
("Running", "Why do some plot lines have gaps?",
 """<p>During takeoff, landing and on the ground no setpoint is sent (ArduCopter flies those phases
 itself), so "commanded" values and the distance to target do not exist then. Measured values
 continue; the periods are labelled.</p>"""),
# ---------------------------------------------------------------- C++
("C++", "What does std::bind(&MissionExecutorNode::onPose, this, std::placeholders::_1) do?",
 """<p>It creates a callable that calls the member function <code>onPose</code> on this object,
 passing the message as the first argument (<code>_1</code>).</p>"""),
("C++", "What is a lambda capture like [this, request_id]?",
 """<p>The lambda copies <code>request_id</code> and the <code>this</code> pointer, so the response
 callback, which runs later, can access the node and knows which request it answers.</p>"""),
("C++", "What is std::lock_guard?",
 """<p>An RAII lock: it locks the mutex in its constructor and unlocks in its destructor at the
 end of the scope, also on early return or exception.</p>"""),
("C++", "Why enum class?",
 """<p>Scoped and strongly typed: <code>State::ARM</code> cannot be confused with an int or with
 another enum's value.</p>"""),
("C++", "Why is the planner's plan() const?",
 """<p>It does not modify the planner; all search data are local. That documents and guarantees
 there are no side effects, and would allow concurrent searches.</p>"""),
]


def render():
    out, last = [], None
    for topic, q, a in QA:
        if topic != last:
            out.append(f"<h3>{html.escape(topic)}</h3>")
            last = topic
        out.append(f'<details class="qa"><summary><span class="tag">{html.escape(topic)}</span>'
                   f'{html.escape(q)}</summary><div class="answer">{a}</div></details>')
    return (f"<p>{len(QA)} questions. Try to answer aloud before opening. "
            "Search filters these too.</p>" + "\n".join(out))
