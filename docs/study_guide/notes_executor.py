FILE = "src/lrs_psenak_valko/src/mission_executor_node.cpp"
TITLE = "Mission executor node (A1.2 core)"
INTRO = """
<p>This is the program that flies the mission. It is one ROS 2 node, a C++ class
<code>MissionExecutorNode</code>. It reads the mission file, runs a <b>state machine</b> 20 times a
second, talks to ArduPilot through <b>MAVROS</b> (topics in, setpoints and service calls out) and to
the <b>A* planner</b> (route requests and answers), and writes the flight log.</p>
<p>How to read it: the <b>constructor</b> sets everything up; the <b>callbacks</b> (<code>on...</code>)
only store incoming data; <b><code>tick()</code></b> (every 50 ms) contains all decisions. Every
state is a <code>case</code> in <code>tick()</code>, and the bigger states have a helper function
(<code>takeoffService</code>, <code>navigateTick</code>, <code>alignPathTick</code>,
<code>taskTick</code>).</p>
"""

NOTES = [
("#include \"lrs_psenak_valko/mission_model.h\"", "Our own header",
 """<p>Includes the mission data types (<code>Waypoint</code>, <code>Precision</code>,
 <code>TaskType</code>) and the CSV loader <code>mission::loadCsv()</code>. Quotes
 <code>"..."</code> = a project header, found through the <code>include/</code> directory that
 CMake adds.</p>"""),

("#include <algorithm>", "C++ standard library headers",
 """<ul>
 <li><code>algorithm</code>: <code>std::min</code>, <code>std::max</code>.</li>
 <li><code>chrono</code>: <code>steady_clock</code>, used for every timeout. A steady clock
 never jumps when the system time is changed.</li>
 <li><code>cmath</code>: <code>sqrt</code>, <code>atan2</code>, <code>sin</code>/<code>cos</code>,
 <code>remainder</code> (angle wrapping), <code>isfinite</code>.</li>
 <li><code>cstdint</code>: fixed-size integers (<code>uint64_t</code>...).</li>
 <li><code>filesystem</code>: creates the log folder. <code>fstream</code>: writes the CSV files.
 <code>iomanip</code>: <code>setprecision</code> for the CSV numbers.</li>
 <li><code>functional</code>: <code>std::bind</code> for callbacks. <code>future</code>: the
 service response type (<code>shared_future</code>).</li>
 <li><code>memory</code>: <code>shared_ptr</code>. <code>mutex</code>: thread safety.
 <code>stdexcept</code>: <code>invalid_argument</code>. <code>string</code>, <code>vector</code>.</li></ul>"""),

("#include <geometry_msgs/msg/pose_array.hpp>", "ROS 2 message and service types",
 """<p>Each header is one generated ROS 2 type:</p><ul>
 <li><code>PoseArray</code>: plan request (start + goal) and the planned route.
 <code>PoseStamped</code>: drone pose in, setpoint out. <code>TwistStamped</code>: velocity.</li>
 <li><code>mavros_msgs</code>: <code>State</code> (connected / armed / mode),
 <code>StatusText</code> (FCU text messages), <code>HomePosition</code>,
 <code>EstimatorStatus</code>; services <code>CommandBool</code> (arm/disarm),
 <code>CommandTOL</code> (takeoff/land), <code>SetMode</code>,
 <code>MessageInterval</code>.</li>
 <li><code>nav_msgs/Path</code>: the flown track for RViz. <code>NavSatFix</code>: GPS.
 <code>std_msgs/String</code>: the planner's text report.</li>
 <li><code>rclcpp/rclcpp.hpp</code>: the ROS 2 C++ client library itself (Node, timers,
 publishers...).</li></ul>"""),

("using namespace std::chrono_literals;", "Time literals",
 """<p>Allows writing durations like <code>500ms</code> or <code>2s</code>. (Most times here are
 computed from double parameters, so it is barely used.)</p>"""),

("class MissionExecutorNode final : public rclcpp::Node", "The node class and its constructor",
 """<p>The class <b>is a</b> ROS 2 node (inherits <code>rclcpp::Node</code>). <code>final</code> =
 nobody can inherit from it. The constructor calls the base constructor with the node name
 <code>"mission_executor_node"</code>, which is the name you see in <code>ros2 node list</code>.</p>"""),

("const std::string mission_file = declare_parameter", "Load the mission (required parameter)",
 """<p><code>declare_parameter&lt;T&gt;(name, default)</code> registers a ROS parameter and returns
 its value (from the command line <code>-p mission_file:=...</code>, or the default).</p>
 <ul><li>Empty path: throw, so the program stops with a clear message (caught in
 <code>main</code>).</li>
 <li><code>mission::loadCsv</code> parses the file; a bad line also throws, with its line
 number.</li></ul>"""),

("hard_radius_ = declare_parameter", "Precision parameters",
 """<ul><li><code>hard_radius</code> 0.15 m: tolerance of a <code>hard</code> waypoint.</li>
 <li><code>soft_radius</code> 0.55 m: tolerance of a <code>soft</code> waypoint (pass through).</li>
 <li><code>pass_radius</code> 0.45 m: when an intermediate route point counts as passed.</li>
 <li><code>settle_speed</code> 0.2 m/s and <code>settle_time</code> 1 s: "stopped" at a hard
 waypoint means slower than 0.2 m/s for 1 s.</li></ul>
 <p>Trailing underscore (<code>hard_radius_</code>) = a member variable of the class (naming
 convention).</p>"""),

("pose_stale_timeout_sec_ = declare_parameter", "Telemetry and loop-rate parameters",
 """<ul><li><code>pose_stale_timeout_sec</code> 5 s: no pose for 5 s while armed = the link is
 lost: fail.</li>
 <li><code>velocity_stale_timeout_sec</code> 3 s: older velocity is not trusted for "settled".</li>
 <li><code>local_position_rate_hz</code> 10: the rate we ask ArduPilot to stream its local
 position at.</li>
 <li><code>setpoint_rate_hz</code> 20: control loop and setpoint rate (the assignment requires
 10–20 Hz).</li></ul>"""),

("handshake_timeout_sec_ = declare_parameter", "Start-up and ground parameters",
 """<ul><li><code>handshake_timeout_sec</code> 180 s: max time for SET_MODE and for ARM.</li>
 <li><code>fcu_ready_timeout_sec</code> 600 s: max time to wait for GPS/EKF in WAIT_FCU (slow PC).</li>
 <li><code>gps_stable_sec</code> 5 s: the GPS fix must be stable this long before GUIDED/arm.</li>
 <li><code>ground_wait_sec</code> 3 s: pause on the ground during <code>landtakeoff</code>.</li>
 <li><code>disarm_grace_sec</code> 5 s: after touchdown, wait this long for LAND mode to disarm
 itself before sending our own disarm.</li></ul>"""),

("cruise_speed_ = declare_parameter", "Carrot, logging and exit parameters",
 """<ul><li><code>cruise_speed</code> 1 m/s: how fast the carrot setpoint slides along the
 route.</li>
 <li><code>max_lead</code> 1 m: the carrot may be at most this far ahead of the drone.</li>
 <li><code>log_dir</code>: folder for <code>flight_log.csv</code>; empty = no logging (the script
 sets it).</li>
 <li><code>exit_on_done</code>: end the program after DONE, so the script can continue.</li></ul>"""),

("mode_retry_interval_sec_ = declare_parameter", "Retry intervals and per-state timeouts",
 """<ul><li>Retry GUIDED / arming every 2 s.</li>
 <li><code>service_timeout_sec</code> 5 s: a service without an answer within 5 s = fail.</li>
 <li><code>planner_timeout_sec</code> 10 s, <code>navigation_timeout_sec</code> 120 s per leg,
 <code>takeoff_timeout_sec</code> 60 s, <code>landing_timeout_sec</code> 90 s.</li>
 <li><code>takeoff_climb_timeout_sec</code> 12 s: accepted takeoff but no climb: re-send.</li>
 <li><code>max_takeoff_attempts</code> 3: read as <code>int64_t</code> (ROS integer type), clamped
 to ≥ 0 and cast to <code>uint32_t</code>.</li></ul>"""),

("yaw_tolerance_deg_ = declare_parameter", "Yaw and frame parameters",
 """<ul><li><code>yaw_tolerance_deg</code> 5°, <code>yaw_hold_time_sec</code> 0.5 s: a heading is
 "reached" when within 5° for 0.5 s.</li>
 <li><code>yaw_timeout_sec</code> 15 s: max time for ALIGN_PATH and TASK.</li>
 <li><code>turn_alignment_threshold_deg</code> 20°: at a route corner sharper than this, stop and
 turn in place.</li>
 <li><code>map_to_local_yaw_offset_deg</code> 90°: the rotation between the planner's map frame
 and MAVROS's local frame.</li></ul>"""),

("if (hard_radius_ <= 0.0 || soft_radius_ <= hard_radius_", "Validate all parameters at start-up",
 """<p>One big <code>if</code>: any nonsensical value (negative time, soft tolerance smaller than
 hard, setpoint rate outside 10–20 Hz, <code>max_lead</code> not larger than
 <code>pass_radius</code>, yaw tolerance ≥ 45°...) throws and the node refuses to start.
 This is <b>fail fast</b>: bad configuration is caught on the ground, not in the air.</p>"""),

("state_group_ = create_callback_group", "Two callback groups",
 """<p>A <b>callback group</b> tells the executor which callbacks may run at the same time.
 Both are <code>MutuallyExclusive</code>: inside one group callbacks run one after another;
 two different groups may run in parallel on the <code>MultiThreadedExecutor</code>
 (see <code>main</code>).</p><ul>
 <li><code>state_group_</code>: all subscriptions (incoming data).</li>
 <li><code>timer_group_</code>: the control timer.</li></ul>
 <p>So a burst of incoming messages never delays the 20 Hz control loop. This is the
 <code>mt_executor_demo</code> pattern the assignment asks for.
 <code>SubscriptionOptions</code> carries the group into <code>create_subscription</code>.</p>"""),

("state_subscription_ = create_subscription<mavros_msgs::msg::State>(", "Subscribe to /mavros/state",
 """<p><code>/mavros/state</code> says whether MAVROS is <b>connected</b> to the FCU, whether the
 drone is <b>armed</b>, and its <b>mode</b> (GUIDED, LAND...). Queue depth 10.
 <code>std::bind(&amp;Class::onState, this, _1)</code> turns the member function into a callback;
 <code>_1</code> is the message.</p>"""),

("const rclcpp::QoS telemetry_qos = rclcpp::SensorDataQoS();", "Quality of Service profiles",
 """<p><b>QoS</b> decides how messages are delivered. A subscriber only receives messages when its
 QoS is compatible with the publisher's.</p><ul>
 <li><code>SensorDataQoS()</code> = <b>best effort</b>, small queue: old sensor data is useless,
 losing one sample is fine. MAVROS publishes pose/velocity best-effort; a <i>reliable</i>
 subscriber would receive nothing. Best effort is compatible with both.</li>
 <li><code>latched_qos</code> = reliable + <b>transient local</b>: a late subscriber still gets the
 last message (used for the home position, sent rarely).</li></ul>"""),

("pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(", "Pose and velocity",
 """<ul><li><code>/mavros/local_position/pose</code>: position + orientation from ArduPilot's EKF,
 in MAVROS's local ENU frame.</li>
 <li><code>/mavros/local_position/velocity_local</code>: linear velocity (for "settled" and
 touchdown).</li></ul>"""),

("gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(", "GPS, home, estimator",
 """<ul><li><code>/mavros/global_position/global</code>: GPS fix status (used for readiness).</li>
 <li><code>/mavros/home_position/home</code>: home position (latched QoS).</li>
 <li><code>/mavros/estimator_status</code>: EKF flags (stored, for diagnostics).</li></ul>"""),

("// FCU text messages (PreArm reasons", "FCU text messages",
 """<p><code>/mavros/statustext/recv</code> carries every text the flight controller prints:
 <code>PreArm: ...</code>, <code>Arm: Gyros inconsistent</code>, <code>EKF3 IMU0 is using
 GPS</code>... MAVProxy only shows them in its own console window. Subscribing here lets the
 executor log them and use "is using GPS" as a readiness signal.</p>"""),

("path_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(", "Planner answers",
 """<p><code>planned_path</code> (the route as a list of poses) and <code>planning_status</code>
 (the planner's text report). No leading slash = relative names, resolved in the node's namespace,
 matching the planner node.</p>"""),

("setpoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(", "Publishers",
 """<ul><li><code>/mavros/setpoint_position/local</code>: the position + yaw target. MAVROS sends
 it to ArduPilot as <code>SET_POSITION_TARGET_LOCAL_NED</code>. In GUIDED mode the copter flies
 to it.</li>
 <li><code>plan_request</code>: a PoseArray [start, goal] for the planner.</li>
 <li><code>mission/flown_path</code>: the flown track, for RViz.</li></ul>"""),

("mode_client_ = create_client<mavros_msgs::srv::SetMode>(\"/mavros/set_mode\");", "Service clients",
 """<p>A <b>service</b> is a request/response call (unlike a topic, you get an answer):</p><ul>
 <li><code>/mavros/set_mode</code>: switch flight mode (GUIDED).</li>
 <li><code>/mavros/cmd/arming</code>: arm (true) / disarm (false).</li>
 <li><code>/mavros/cmd/takeoff</code>, <code>/mavros/cmd/land</code>: MAV_CMD_NAV_TAKEOFF /
 NAV_LAND.</li>
 <li><code>/mavros/set_message_interval</code>: ask ArduPilot to send a MAVLink message at a given
 rate.</li></ul>"""),

("const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(", "The 20 Hz control timer",
 """<p>Period = 1/20 s = 50 ms, converted to nanoseconds. <code>create_wall_timer</code> calls
 <code>tick()</code> every period, in <code>timer_group_</code>. Wall timer = real time (not
 simulation time).</p>"""),

("openLogs();", "Finish construction",
 """<ul><li><code>openLogs()</code> creates the CSV files (if <code>log_dir</code> is set).</li>
 <li>Prints every waypoint so the log shows what was loaded (index, x/y/z, precision, task).
 <code>%zu</code> prints a <code>size_t</code>.</li>
 <li><code>transition(State::IDLE)</code>: the state machine starts.</li></ul>"""),

("private:", "The list of states",
 """<p><code>enum class State</code> = a type-safe list of names (cannot be mixed with integers).
 Every value is one phase of the mission; see the "big picture" chapter for the diagram.
 <code>WAIT_PREFETCH</code> = waiting for a route that was requested in advance;
 <code>GROUND_WAIT</code> and <code>RETAKEOFF</code> exist for <code>landtakeoff</code>.</p>"""),

("static const char* stateName(State state)", "State names for the log",
 """<p>Converts a state to text for log lines and the CSV. <code>static</code> = does not need an
 object. The <code>switch</code> covers every value; the final <code>return "UNKNOWN"</code>
 silences the compiler warning.</p>"""),

("void transition(State next)", "transition(): the only way to change state",
 """<p>Every state change goes through here, so the bookkeeping is always done:</p><ul>
 <li>Store the new state and the time it started (for state timeouts) and reset the "still
 waiting" log timer.</li>
 <li>TAKEOFF/RETAKEOFF: reset takeoff flags and remember the start altitude.</li>
 <li>SET_MODE / ARM: allow an immediate first attempt, reset rejection counters.</li>
 <li>NAVIGATE: the carrot starts at the current target.</li>
 <li>TASK/NAVIGATE/ALIGN: reset "settled" counters.</li>
 <li>Log <code>Mission state: X</code> (the timestamped log the assignment asks for).</li>
 <li><code>service_pending_ = false; ++service_request_id_</code>: any service answer still in
 flight for the <i>previous</i> state will be ignored (its id no longer matches).</li>
 <li>Reset <code>stable_since_</code> (time 0 = "not stable yet").</li></ul>"""),

("void fail(const std::string& reason)", "fail(): go to ERROR safely",
 """<p>Logs the reason with the state it happened in. If a pose is known, the target becomes the
 <b>current measured position and heading</b>. In ERROR setpoints keep being published, so the
 drone hovers where it is instead of continuing to an old target. Then ERROR.</p>"""),

("uint64_t beginServiceCall()", "Service call bookkeeping",
 """<p><code>beginServiceCall</code>: mark "a call is pending", remember when (for the 5 s
 timeout), and return a new <b>request id</b>.</p>
 <p><code>finishServiceCall(id)</code> is called in the response callback. It accepts the answer
 only if this exact call is still the pending one and we are not in ERROR. A late answer to an old
 request (after a state change) is thrown away. Without this, an old "arm accepted" could act on a
 new state.</p>"""),

("double timeoutForState() const", "Time limit of each state",
 """<p>Returns the maximum time the current state may last (0 = no limit). <code>tick()</code>
 fails the mission if it is exceeded. Several <code>case</code>s share one return. This is how
 "a missing response or state transition ends in a logged ERROR" is implemented.</p>"""),

("void logWaiting()", "Rate-limited 'still waiting' log",
 """<p>Prints <code>Still waiting in state X</code> at most every 5 s, so a waiting state does not
 flood the terminal at 20 Hz.</p>"""),

("void onState(const mavros_msgs::msg::State::SharedPtr message)", "onState(): store the FCU state",
 """<p><code>std::lock_guard</code> locks the mutex for the rest of the function (unlocks
 automatically at the closing brace, even on exceptions; this is RAII). It copies the message.
 All callbacks follow this pattern: <b>lock, store, return</b>. Decisions are made only in
 <code>tick()</code>.</p>"""),

("void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)", "onPose(): store the pose",
 """<ul><li>Store the pose, mark it received, and remember when (freshness checks).</li>
 <li>On the <b>first</b> pose: initialise the target to the current position (map frame) and
 heading, so a target exists before flight. Log the frame conversion; this line proves the 90°
 rotation (local (-7, 12.99) = map (12.99, 7)).</li></ul>"""),

("void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)", "onVelocity()",
 """<p>Stores the velocity and increments <code>velocity_sample_sequence_</code>. The counter lets
 the "settled" check count <b>different</b> messages, instead of the same old message seen 20
 times by the 20 Hz loop.</p>"""),

("void onGps(const sensor_msgs::msg::NavSatFix::SharedPtr message)", "onGps(): GPS fix stability",
 """<p><code>status &gt;= STATUS_FIX</code> means a fix. Losing the fix resets the timer; the first
 message with a fix starts it (<code>gps_fix_since_</code>). <code>fcuReady()</code> later
 requires the fix to have lasted <code>gps_stable_sec</code>.</p>"""),

("void onHome(const mavros_msgs::msg::HomePosition::SharedPtr message)", "onHome()",
 """<p>Home is valid when latitude/longitude are finite and not (0, 0). Stored for diagnostics.</p>"""),

("void onStatusText(const mavros_msgs::msg::StatusText::SharedPtr message)", "onStatusText(): FCU messages",
 """<ul><li>The first <code>"... is using GPS"</code> message means the EKF has an absolute
 position. That is the best readiness signal, so the flag <code>ekf_using_gps_</code> is set.</li>
 <li>Every other text is re-logged: severity ≤ WARNING (0–4: emergency... warning) as WARN, else
 INFO. This is how <code>FCU: Arm: Gyros inconsistent</code> appears in our log.</li></ul>"""),

("void onEstimatorStatus(const mavros_msgs::msg::EstimatorStatus::SharedPtr message)", "onEstimatorStatus()",
 """<p>Stores the EKF status flags and their time. Kept for diagnostics; the decisions use GPS +
 status text instead.</p>"""),

("void onPath(const geometry_msgs::msg::PoseArray::SharedPtr message)", "onPath(): which route is this?",
 """<p>The planner answers on one topic. The executor must know which request an answer
 belongs to:</p><ul>
 <li>if a <b>prefetch</b> (next leg) request is outstanding, the route is the prefetched one;</li>
 <li>else, if we are in WAIT_PLAN, it is the route for the current leg;</li>
 <li>otherwise it is ignored.</li></ul>"""),

("void onPlanningStatus(const std_msgs::msg::String::SharedPtr message)", "onPlanningStatus(): success or not",
 """<p>The planner's report is Slovak text. Success = it contains <code>[OK] TRASA NÁJDENÁ</code>
 ("route found"). The same prefetch / WAIT_PLAN routing as <code>onPath</code>. On failure the
 whole report is kept and becomes the error message.</p>"""),

("static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)", "Quaternion → yaw",
 """<p>ROS orientations are <b>quaternions</b> (x, y, z, w). The yaw (rotation about z) is
 <code>atan2(2(wz + xy), 1 − 2(y² + z²))</code>, the standard ZYX Euler formula. Result in
 (−π, π].</p>"""),

("static double unwrapNear(double reference, double angle)", "Angle unwrapping",
 """<p>Returns the angle equivalent to <code>angle</code> (same direction, ± k·360°) that is
 closest to <code>reference</code>. <code>remainder(a, 2π)</code> maps a difference into
 [−π, π]. Example: reference 170°, angle −170°: result 190°, so the drone turns 20°, not 340°.
 This is the "unwrap to avoid full-turn spins" from the assignment.</p>"""),

("static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)", "Yaw → quaternion",
 """<p>A pure rotation about z by angle ψ is the quaternion (0, 0, sin ψ/2, cos ψ/2). x and y stay
 0, so roll and pitch are not commanded (ArduPilot controls them itself).</p>"""),

("geometry_msgs::msg::Pose localToMapPose(const geometry_msgs::msg::Pose& local_pose) const", "MAVROS local → map",
 """<p>Rotates a pose by −offset (offset = 90°) about z:
 <code>x_m = c·x_l + s·y_l</code>, <code>y_m = −s·x_l + c·y_l</code>, yaw − offset. z is
 unchanged. With 90°: <code>x_m = y_l</code> (north), <code>y_m = −x_l</code> (west). Every pose
 that comes from MAVROS passes through here, so the rest of the code works only in the map frame
 (the frame of the planner and mission file).</p>"""),

("geometry_msgs::msg::Pose mapToLocalPose(const geometry_msgs::msg::Pose& map_pose) const", "map → MAVROS local",
 """<p>The inverse rotation (+offset). Used once: on the way out, in
 <code>publishSetpoint()</code>.</p>"""),

("double currentMapYaw() const", "Current heading in the map frame",
 """<p>Measured yaw from MAVROS minus the 90° offset.</p>"""),

("double distanceTo(const mission::Waypoint& point) const", "3D distance to a waypoint",
 """<p>Converts the current pose to map and returns the Euclidean distance √(dx²+dy²+dz²) to the
 waypoint. Used for the hard/soft arrival check.</p>"""),

("double speed() const", "Speed",
 """<p>Magnitude of the velocity vector (m/s).</p>"""),

("bool velocityFresh() const", "Freshness checks",
 """<p>Data counts as fresh if it was received and is younger than its stale timeout. Old data
 must never confirm "settled" or "landed".</p>"""),

("// Arming readiness: the EKF announced it uses GPS", "fcuReady(): may we try GUIDED and arming?",
 """<p>Checked in WAIT_FCU, in this order. The first failing check is reported in <code>why</code>
 (the log shows the reason every 5 s):</p><ol>
 <li>MAVROS connected to the FCU;</li><li>a fresh local pose;</li>
 <li>EKF said "is using GPS": ready; or else</li>
 <li>a fresh GPS message with a fix, stable for <code>gps_stable_sec</code>.</li></ol>
 <p>The fallback (4) exists because the "using GPS" text is sent only once: an executor started
 later would never see it. In the tests the GPS fix came first; arming then failed for ~25 s with
 "Gyros inconsistent" and was retried until it succeeded.</p>"""),

("void resetHardArrivalSamples()", "Reset the 'settled' detector",
 """<p>Zero stable samples, and time 0 means "not stable". Called whenever the drone is not (or no
 longer) within tolerance and slow.</p>"""),

("// ArduCopter's GUIDED mode drops a running takeoff", "setpointsAllowed(): the takeoff bug fix",
 """<p><b>Important for the exam.</b> Setpoints are published only in flight states (PLAN … TASK)
 and in ERROR (hold). Not during SET_MODE/ARM/TAKEOFF/LAND.</p>
 <p>Why: in GUIDED mode, a position target cancels ArduCopter's takeoff (the guided sub-mode
 switches from Takeoff to Position). Position control does not spin the motors up while landed,
 so the copter sits on the ground and auto-disarms after <code>DISARM_DELAY</code>. That was
 exactly the original symptom "arms, then disarms, never takes off".</p>"""),

("void publishSetpoint()", "publishSetpoint(): send the target to MAVROS",
 """<ul><li>Nothing to send without a pose or target, or when setpoints are not allowed.</li>
 <li>Stamp with the current time, frame "map" (MAVROS ignores frame_id for this topic and treats
 it as its local frame).</li>
 <li>Convert the target position map → local; build the orientation from <code>target_yaw_</code>
 + 90°.</li>
 <li>Publish. Called at 20 Hz at the start of every <code>tick()</code>.</li></ul>"""),

("void setTarget(const geometry_msgs::msg::Pose& pose, double yaw)", "setTarget()",
 """<p>Sets the target pose and the yaw, unwrapped near the previous yaw, so the commanded yaw
 never jumps by 360°.</p>"""),

("void requestPlan()", "requestPlan(): ask A* for the current leg",
 """<ul><li>Start = current pose (map frame); goal = the current mission waypoint
 (<code>orientation.w = 1</code> = no rotation, a valid quaternion).</li>
 <li>Clear the old answer flags, mark "not a prefetch".</li>
 <li><b>Transition first, publish second</b>: we must already be in WAIT_PLAN when the answer
 arrives, or <code>onPath</code> would ignore it.</li></ul>"""),

("void requestPrefetchPlan()", "requestPrefetchPlan(): plan the next leg in advance",
 """<p>While the drone flies to waypoint i, ask for the route i → i+1, starting at the
 <b>waypoint position</b> (where the drone will be). When the drone arrives, the route is
 usually ready and there is no hovering while A* runs. Returns immediately at the last
 waypoint. Remembers which waypoint the prefetch is for.</p>"""),

("bool activatePrefetchedPath()", "activatePrefetchedPath(): accept the prepared route",
 """<p>Called in WAIT_PREFETCH. Returns false = "still waiting".</p><ul>
 <li>No status yet: wait.</li>
 <li>Prefetch is for another waypoint: fail (should never happen; a safety check).</li>
 <li>Planner failed: fail with its report. Route empty: fail.</li>
 <li><b>Endpoint check</b>: if the planner moved ("snapped") the goal further than the
 waypoint's tolerance (0.15 hard / 0.55 soft), fail. Flying there would not reach the mission
 point.</li>
 <li>OK: take the route (<code>std::move</code>, no copy) and start navigating.</li></ul>"""),

("void beginPathNavigation()", "beginPathNavigation(): start flying a route",
 """<ul><li>Log the route to <code>planned_paths.csv</code>.</li>
 <li>Route with ≥ 2 points: target = hold the current position, yaw = direction of the first
 segment, then ALIGN_PATH (turn first, then fly).</li>
 <li>One point: fly straight to it.</li>
 <li>Immediately prefetch the next leg.</li></ul>"""),

("// ------------------------------------------------------------------ flight logging", "openLogs(): create the CSV files",
 """<p>Creates the folder (<code>create_directories</code>, error code instead of exception),
 opens both files, writes the CSV headers. If they cannot be opened, logging is disabled with a
 warning; the flight itself must not fail because of logging. <code>log_started_at_</code> is time
 0 of the log.</p>"""),

("void logSample()", "logSample(): the flown track for RViz",
 """<p>Every 0.2 s while armed, append the current map-frame pose to a <code>nav_msgs/Path</code>
 and publish it (RViz shows it as a line). Capped at 20 000 poses so memory cannot grow without
 limit.</p>"""),

("if (!flight_log_ || !have_pose_) return;", "One CSV row per tick",
 """<p>Writes <code>t, state, waypoint, armed, mode, cmd_valid, cmd x/y/z/yaw, x/y/z/yaw,
 speed</code>. <code>cmd_valid</code> = whether a setpoint was really being sent; this is why the
 plots have gaps during takeoff/landing. <code>deg()</code> wraps to ±180° and converts to
 degrees. The file is flushed every second, so a crash loses at most 1 s.</p>"""),

("void logPlannedPath()", "logPlannedPath()",
 """<p>Writes every point of the route being started (leg number, waypoint, point index, x, y, z)
 and flushes immediately. Read by the plot and 3D scripts.</p>"""),

("void tick()", "tick(): start of the 20 Hz loop",
 """<ul><li>Lock the mutex for the whole tick: callbacks cannot change data in the middle of a
 decision.</li>
 <li>Publish the setpoint and log a sample, in <b>every</b> state, before any early return (so a
 hold keeps being sent even in ERROR).</li>
 <li>One second after DONE: flush the logs and call <code>rclcpp::shutdown()</code>. The
 executor's <code>spin()</code> returns and the program exits; the run script then continues
 with plotting.</li></ul>"""),

("if (state_ == State::DONE || state_ == State::ERROR) return;", "Global checks 1: finished states, service timeout",
 """<p>DONE and ERROR make no further decisions. A service call pending longer than
 <code>service_timeout_sec</code> means MAVROS/the FCU did not answer: fail.</p>"""),

("const double state_timeout = timeoutForState();", "Global checks 2: state timeout",
 """<p>If the current state has lasted longer than its limit, fail. For ARM the message includes
 how many attempts were rejected and points to the PreArm messages.</p>"""),

("if (vehicle_state_.armed && have_pose_ &&", "Global checks 3: lost telemetry",
 """<p>Armed, but no pose for <code>pose_stale_timeout_sec</code>: the link is lost (in our tests:
 Gazebo paused, so SITL froze). Fail with a hint. Only while armed, because short gaps while
 disarmed (pre-arm) are harmless.</p>"""),

("const bool requires_guided_flight = state_ == State::TAKEOFF", "Global checks 4: FCU link in flight",
 """<p>The list of states in which the copter must be connected, armed and (mostly) in GUIDED. If
 the MAVROS link drops in any of them: fail.</p>"""),

("if (requires_guided_flight && !vehicle_state_.armed) {", "Global checks 5: unexpected disarm",
 """<p>In a flight state but not armed:</p><ul>
 <li>During TAKEOFF/RETAKEOFF, still on the ground (z &lt; start + 0.3 m), and retries left:
 ArduCopter auto-disarmed before lifting off. Retry the whole GUIDED → arm → takeoff sequence
 (max 3×).</li>
 <li>Anything else (e.g. disarmed in the air): fail.</li></ul>"""),

("if ((state_ == State::TAKEOFF || state_ == State::RETAKEOFF || state_ == State::ALIGN_PATH ||", "Global checks 6: still in GUIDED?",
 """<p>If the mode changed away from GUIDED during takeoff/navigation (e.g. RC override, failsafe
 RTL/LAND), stop: we must not fight the pilot or a failsafe. Then: past WAIT_FCU, nothing can be
 decided without a pose, so wait.</p>"""),

("switch (state_) {", "The state switch: IDLE",
 """<p>Each <code>case</code> below is one state. IDLE only moves on to WAIT_FCU (it exists so the
 log shows the start).</p>"""),

("case State::WAIT_FCU: {", "WAIT_FCU",
 """<ul><li>If <code>fcuReady()</code>: once, ask ArduPilot to send <code>LOCAL_POSITION_NED</code>
 (MAVLink message id 32) at 10 Hz, so pose updates are frequent enough for settling. The
 response callback just logs.</li>
 <li>Log why it is ready, then SET_MODE.</li>
 <li>Else log the reason every 5 s.</li>
 <li>Braces <code>{ }</code> around the case: needed because a variable (<code>why</code>) is
 declared inside it.</li></ul>"""),

("case State::SET_MODE:", "SET_MODE: request GUIDED",
 """<ul><li>Already GUIDED (reported by /mavros/state): go to ARM.</li>
 <li>Otherwise, if no call is pending, the service exists and the retry time has come: call
 <code>set_mode(custom_mode="GUIDED")</code> asynchronously.</li>
 <li>The lambda <code>[this, request_id](future){...}</code> runs when the answer arrives; it
 locks the mutex and checks the request id.</li>
 <li><code>mode_sent == false</code>: rejected by MAVROS, retry in 2 s. <code>mode_sent ==
 true</code> only means "sent": the FCU may still refuse (e.g. no position yet), so wait 2 s for
 /mavros/state before sending again. Earlier the code re-sent every 50 ms.</li></ul>"""),

("case State::ARM:", "ARM: arm the motors",
 """<ul><li>Armed: go to TAKEOFF, or RETAKEOFF during a landtakeoff.</li>
 <li>No fresh pose: wait.</li>
 <li>Call <code>cmd/arming(value=true)</code>. If refused (<code>success == false</code>), count
 it, log the MAVLink result code (4 = FAILED), retry in 2 s. The reason itself arrives as an FCU
 text (e.g. "Arm: Gyros inconsistent") and is logged by <code>onStatusText</code>.</li></ul>"""),

("case State::TAKEOFF:", "TAKEOFF and PLAN",
 """<ul><li>TAKEOFF: run <code>takeoffService()</code> with the altitude of the first waypoint.
 It returns <code>false</code> when the takeoff is complete; then PLAN.</li>
 <li>PLAN: send the plan request (which moves to WAIT_PLAN).</li></ul>"""),

("case State::WAIT_PLAN:", "WAIT_PLAN: check the answer",
 """<ul><li>Status says failure: fail with the planner's report.</li>
 <li>Status and route both arrived: reject an empty route; reject a route whose end is further
 from the goal than the waypoint tolerance (goal snapped too far); otherwise start the
 route.</li>
 <li>Else keep waiting (with the planner timeout).</li></ul>"""),

("case State::WAIT_PREFETCH:", "Dispatch to the flight helpers",
 """<p>WAIT_PREFETCH, ALIGN_PATH, NAVIGATE and TASK are implemented in their own functions to keep
 <code>tick()</code> readable.</p>"""),

("case State::LAND:", "LAND: command landing",
 """<p>Call <code>cmd/land</code> once (yaw = current target yaw). ArduCopter switches to LAND mode
 and descends by itself. Accepted: WAIT_LANDED; rejected: fail. No setpoints are sent now.</p>"""),

("case State::WAIT_LANDED:", "WAIT_LANDED: detect touchdown",
 """<p>On the ground = already disarmed, <b>or</b> z ≤ 0.15 m with speed ≤ 0.15 m/s from fresh
 data. Then set the earliest time for our own disarm (now + <code>disarm_grace_sec</code>) and go
 to WAIT_DISARMED.</p>"""),

("case State::WAIT_DISARMED:", "WAIT_DISARMED: the landing bug fix",
 """<p><b>Important for the exam.</b> ArduCopter's LAND mode disarms by itself once <i>its own</i>
 land detector agrees, about a second after touchdown. A disarm command before that is refused.
 The old code treated that refusal as fatal, and the mission died at the landtakeoff waypoint.
 Now:</p><ul>
 <li>still armed: after the grace time, send a disarm (refusal = retry in 2 s);</li>
 <li>disarmed: landtakeoff: GROUND_WAIT; final land: log the task as complete, then DONE.</li></ul>"""),

("case State::GROUND_WAIT:", "GROUND_WAIT",
 """<p>Stay on the ground for <code>ground_wait_sec</code> ("land, wait on the ground" from the
 assignment), then SET_MODE. The copter is in LAND mode and must be put back into GUIDED before
 it can be armed and take off again.</p>"""),

("case State::RETAKEOFF:", "RETAKEOFF, DONE, ERROR",
 """<p>RETAKEOFF: take off to this waypoint's altitude; when done, clear the landtakeoff flag, log
 the task, and continue with the next waypoint. DONE/ERROR: nothing (handled at the top of
 <code>tick()</code>).</p>"""),

("bool takeoffService(double altitude)", "takeoffService() part 1: takeoff target",
 """<p>Returns <code>true</code> = "still taking off", <code>false</code> = done. First call:
 remember a target above the current x/y at the requested altitude (used as the hold target after
 takeoff; it is not sent during takeoff).</p>"""),

("if (!takeoff_command_accepted_) {", "takeoffService() part 2: NAV_TAKEOFF",
 """<ul><li>Send <code>cmd/takeoff(altitude)</code> once (unless a call is pending or a retry
 time is set). ArduPilot interprets the altitude relative to home (the floor).</li>
 <li>Rejected: allow a retry in 2 s. Accepted: remember when (for the "no climb" check).</li>
 <li>Not accepted yet: return "still taking off".</li></ul>"""),

("const double confirmed_altitude = std::max(", "takeoffService() part 3: wait for the climb",
 """<p>Takeoff counts as complete at z ≥ max(start + 0.5 m, target − 0.15 m). While below:</p><ul>
 <li>armed, still on the ground, and more than <code>takeoff_climb_timeout_sec</code> since the
 accepted command: re-send the takeoff (max 3×, then fail);</li>
 <li>log progress every 5 s.</li></ul>"""),

("takeoff_disarm_retries_ = 0;", "takeoffService() part 4: done",
 """<p>Reset the retry counters and log the confirmed altitude. Returning <code>false</code> tells
 the caller to move on.</p>"""),

("void navigateTick()", "navigateTick() part 1: carrot target",
 """<p>While there are route points left: the current route point is the "vertex". Advance the
 carrot towards it; the target is the carrot position, with yaw = direction of the current
 segment. The drone always faces the direction of travel.</p>"""),

("const geometry_msgs::msg::Pose map_pose = localToMapPose(pose_.pose);", "navigateTick() part 2: passing a route point",
 """<p>A route point is passed when the carrot has reached it <b>and</b> the drone is within
 <code>pass_radius</code> (0.45 m). Intermediate points are pass-through, no stopping. Then the
 next point. If the new segment turns more than 20° from the old one, hold at the corner and turn
 in ALIGN_PATH first; gentle turns are flown without stopping.</p>"""),

("setTarget(path_.back(), target_yaw_);", "navigateTick() part 3: arriving at the waypoint",
 """<p>All route points passed; the target is the final point (the waypoint). Outside the tolerance
 radius: keep flying, reset "settled". <b>Soft</b> waypoint inside 0.55 m: done immediately, no
 stop (go to TASK).</p>"""),

("if (!velocityFresh()) {", "navigateTick() part 4: settling at a hard waypoint",
 """<p><b>Hard</b> waypoint inside 0.15 m: additionally the drone must be slow (≤ 0.2 m/s) on
 fresh data. Each <b>new</b> velocity message that is slow increments the counter (the first one
 starts the clock). At least 2 messages and ≥ <code>settle_time</code> (1 s) of continuous
 slowness: "arrived and stable", go to TASK. Any fast or stale sample resets it.</p>"""),

("// Carrot following: the setpoint slides along the planned segment", "advanceCarrot(): follow the straight line",
 """<p><b>Important for the exam.</b> The carrot is a point on the segment that moves towards the
 vertex by <code>cruise_speed / 20</code> per tick (e.g. 1 m/s → 5 cm per tick), but only while
 the drone is closer than <code>max_lead</code> (1 m) to it. The carrot waits for a slow
 drone.</p>
 <p>Why: if the setpoint jumps straight to the segment end, ArduCopter flies there with its own
 horizontal and vertical speed limits and leaves the straight line (0.56 m measured, e.g.
 descending early). The A* route is only guaranteed collision-free on the line, which matters
 most over a rack. Returns true when the carrot is at the vertex.</p>"""),

("void alignPathTick()", "alignPathTick(): turn before flying",
 """<p>Hovering at the target, wait until the heading error (wrapped to ±180°) is ≤ 5°
 continuously for 0.5 s, then NAVIGATE. Any larger error resets the hold time.</p>"""),

("double segmentYaw(size_t path_index) const", "segmentYaw(): direction of travel",
 """<p>Heading of the segment ending at point <code>path_index</code>:
 <code>atan2(Δy, Δx)</code> in the map frame. With no valid segment, keep the current yaw. This is
 the "atan2 of the segment" from the assignment.</p>"""),

("void taskTick()", "taskTick(): tasks without action",
 """<p><code>-</code> and <code>takeoff</code> need no action at the waypoint (the takeoff itself
 already happened in TAKEOFF): log completion and continue.</p>"""),

("case mission::TaskType::YAW: {", "taskTick(): yaw task",
 """<p>Convert degrees to radians, set the commanded yaw (unwrapped near the current target, so
 the shorter turn is taken). Compare with the measured map yaw; within 5° for 0.5 s: done.
 Absolute map-frame heading, CCW positive, 0° = map +x.</p>"""),

("case mission::TaskType::LAND:", "taskTick(): land and landtakeoff",
 """<p>Both go to LAND. <code>landtakeoff_pending_</code> decides what happens after the disarm:
 false = DONE (mission end), true = GROUND_WAIT → re-arm → RETAKEOFF.</p>"""),

("void logTaskComplete(mission::TaskType task)", "logTaskComplete()",
 """<p>The log line <code>Completed task 'X' at waypoint N</code>: proof that every task ran, in
 order.</p>"""),

("void advanceWaypoint()", "advanceWaypoint(): next waypoint",
 """<p>Increment the index. Past the last one: DONE. Otherwise, if the next route was prefetched,
 WAIT_PREFETCH (usually immediate); else PLAN (request it now).</p>"""),

("void disarmOnGround()", "disarmOnGround(): fallback disarm",
 """<p>Sends <code>cmd/arming(false)</code>. Whatever the answer, the next attempt is allowed
 2 s later; a refusal ("not landed yet") is only a warning. WAIT_DISARMED sees the real disarm in
 /mavros/state.</p>"""),

("mission::Mission mission_;", "Member variables: mission and vehicle state",
 """<p>The data the node remembers between ticks: the mission and the current waypoint/route index,
 the route, the current state, the last messages (FCU state, pose, velocity, target), "have
 received" flags, GPS/EKF readiness, service flags and counters, planner answer flags, the
 prefetch data, and the <b>mutex</b> that protects all of it.</p>"""),

("double hard_radius_ = 0.15, soft_radius_ = 0.55", "Member variables: parameters and logging",
 """<p>Copies of the parameters (default values shown, overwritten in the constructor), the carrot
 position, the log files and the flown path.</p>"""),

("std::chrono::steady_clock::time_point state_started_at_", "Member variables: timestamps",
 """<p>Steady-clock time points for timeouts, retries and freshness. <code>stable_since_</code> is
 an <code>rclcpp::Time</code> (time 0 = not stable).</p>"""),

("rclcpp::CallbackGroup::SharedPtr state_group_, timer_group_;", "Member variables: ROS handles",
 """<p>Shared pointers to the callback groups, subscriptions, publishers, service clients and the
 timer. They must be stored: if a handle is destroyed, the subscription/timer stops.</p>"""),

("int main(int argc, char** argv)", "main()",
 """<ul><li><code>rclcpp::init</code>: start ROS 2 (parses <code>--ros-args</code>).</li>
 <li>Create the node (the constructor may throw: bad parameter, bad mission file).</li>
 <li><code>MultiThreadedExecutor</code>: runs callbacks on several threads, so the two callback
 groups really run in parallel. <code>spin()</code> blocks until shutdown (Ctrl+C or our
 <code>rclcpp::shutdown()</code> after DONE).</li>
 <li>An exception: log FATAL and exit with code 1, which the run script reports.</li></ul>"""),
]
