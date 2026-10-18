FILE = "git:origin/A1_3:src/lrs_psenak_valko/src/outdoor_mission_node.cpp"
TITLE = "Outdoor mission node (A1.3)"
INTRO = """
<p>The A1.3 program (branch <code>A1_3</code>). One ROS 2 node flies the outdoor mission: wait for a
good GPS fix and capture the origin, take off to 5 m, fly a 5 × 5 m square with the nose locked on a
point of interest (POI), fly a figure-8 as a continuous trajectory with the nose along the tangent,
return and land.</p>
<p>Differences from A1.2 worth remembering: <b>single-threaded</b> executor (no mutex),
<code>/mavros/setpoint_raw/local</code> with <b>position + velocity + yaw</b>, a 30 Hz timer,
three ways out of a mission (ABORTING, STOPPED, FAULT), and all geometry maths in
<code>outdoor_math.h</code>.</p>
"""

NOTES = [
("// outdoor_mission_node.cpp -- A1.3 outdoor GPS mission.", "Header: the whole design in one comment",
 """<ul><li>The state sequence, and the two ways of leaving a mission: <b>ABORTING</b> (we switch the
 vehicle to a safe mode) and <b>STOPPED</b> (the pilot or the link took over, so we only stop
 sending).</li>
 <li>Everything is relative to the <b>launch point</b> (mean of stable GPS fixes): no hard-coded
 lat/lon.</li>
 <li>Frames: local ENU, yaw 0 = East, CCW+. Altitude is commanded as local z = launch z + 5 m and
 checked against <code>rel_alt</code>.</li>
 <li>POI as lat/lon or as an offset; only yaw is controlled (fixed camera), the downward look angle
 is logged.</li>
 <li>Heading policy per phase, and why one thread: no data races, no mutex.</li></ul>"""),
("#include \"outdoor_math.h\"", "Includes",
 """<p>The maths header (no ROS), the C++ standard library, and the ROS messages: pose/velocity,
 <code>PositionTarget</code> (raw setpoint with velocity), <code>NavSatFix</code> (GPS),
 <code>Float64</code> (compass heading, rel_alt), <code>UInt32</code> (satellites), <code>Trigger</code>
 (abort service), <code>Path</code> (RViz).</p>"""),
("using namespace std::chrono_literals;", "Short names",
 """<p><code>Vec2</code>/<code>Vec3</code> from the maths header and <code>SteadyClock</code> as an
 alias.</p>"""),
("class OutdoorMissionNode final : public rclcpp::Node", "Constructor: set-up in five steps",
 """<p>Declare parameters → validate them → check the mission envelope (geofence) <b>before anything
 can arm</b> → create topics/services/timer → open the CSV log. Then print the key numbers (lap time,
 peak speed) and where the POI comes from, and start in WAIT_GPS.</p>"""),
("enum class State", "States",
 """<p>WAIT_GPS, SET_MODE, ARM, TAKEOFF, TO_START, SQUARE, TO_CENTER, FIGURE8, RETURN, LAND, DONE,
 and the exits ABORTING → ABORTED, STOPPED, FAULT.</p>"""),
("static const char* stateName(State s)", "State names", """<p>Text for logs and the <code>/mission/state</code> topic.</p>"""),
("struct Params", "All parameters in one struct",
 """<ul><li>Geometry fixed by the assignment (5 m altitude, 5 m square, A = 3 m), but exposed as
 parameters so they are documented.</li>
 <li>Figure-8: ω = 0.30 rad/s, 1 lap, 3 s ramp.</li>
 <li>Motion: 30 Hz control, 1.5 m/s max, 0.5 m/s on the square, 0.5 m/s² carrot acceleration, yaw
 look-ahead 0.5 s, velocity feed-forward on.</li>
 <li>GPS gate: ≥ 8 satellites, σ ≤ 2 m, stable 3 s. Geofence 4 m / 5.5 m, tracking error 2 m.
 Timeouts.</li></ul>"""),
("void declareParameters()", "declareParameters()",
 """<p>Every field read as a ROS parameter with a unit in its name (<code>_m</code>,
 <code>_s</code>, <code>_rad_s</code>...). The POI lat/lon default to NaN = "not given".</p>"""),
("bool poiFromGeodetic() const", "POI source", """<p>True when both POI lat and lon are given.</p>"""),
("void validateParameters() const", "validateParameters(): refuse bad numbers",
 """<p>Throws for impossible values. Most important: the figure-8 peak speed
 <code>A·ω·√2</code> must not exceed <code>max_speed</code>; the error message even says the
 largest allowed ω.</p>"""),
("/// Pre-flight geofence arithmetic", "checkMissionEnvelope(): geofence before flight",
 """<p>Computes the largest distance from launch of anything the mission visits (corners and the
 whole figure-8). Over the limit: the node does not start. With a lat/lon POI the check is repeated
 after the origin is known.</p>"""),
("void createInterfaces()", "Subscriptions",
 """<p>All telemetry with <code>SensorDataQoS</code> (best effort, compatible with MAVROS).
 Callbacks only store the message and the time (single thread, so no mutex). The GPS callback also
 counts messages (<code>fix_seq_</code>) so the origin averaging adds each fix only once.</p>"""),
("// The home position is published once and latched", "Home position: latched QoS",
 """<p>Home is published once; a late subscriber only gets it with <b>transient local</b>
 durability.</p>"""),
("setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(", "Publishers, services, timer",
 """<ul><li><code>/mavros/setpoint_raw/local</code>: position + velocity + yaw in one message.</li>
 <li>Intended path (latched), flown path, <code>/mission/state</code> (latched, lets the rosbag be
 split into phases).</li>
 <li>Clients for set_mode, arming, takeoff; the <code>~/abort</code> service (Trigger) that only
 sets a flag.</li>
 <li>The control timer at 30 Hz calling <code>tick()</code>.</li></ul>"""),
("void openLog()", "CSV log", """<p>Header with commanded and measured values, the yaw/POI errors, rel_alt, compass, velocity command and look-down angle.</p>"""),
("void transition(State next)", "transition()",
 """<p>Logs <code>State: A -> B</code>, stores time, resets the "stable" timer and the carrot speed
 (every phase starts at rest), starts the figure-8 clock, publishes the state.</p>"""),
("static bool isStreaming(State s)", "Phase helpers",
 """<p><code>isStreaming</code>: states in which setpoints are sent (TAKEOFF is listed but excluded
 when publishing). <code>isTerminal</code>: nothing more to do. <code>airborne</code>: flying, so an
 abort must put the vehicle into a safe mode.</p>"""),
("void fault(const std::string& why)", "Three ways out",
 """<ul><li><code>fault</code>: error on the ground: just stop.</li>
 <li><code>beginAbort</code>: airborne: switch to <code>abort_mode</code> (LAND by default).</li>
 <li><code>stopSilently</code>: the pilot or link took over: send <b>nothing</b> any more, change
 nothing ("do not fight the pilot").</li></ul>"""),
("void logWaiting(const std::string& what)", "Rate-limited waiting log", """<p>At most every 5 s.</p>"""),
("// ---------------------------------------------------- service helpers", "Service helpers",
 """<p>Same idea as A1.2: one call pending at a time, an id per call, plus <code>canCallService</code>
 that allows at most one attempt per second.</p>"""),
("void trySetMode(const std::string& mode)", "trySetMode() / tryArm()",
 """<p>Asynchronous calls; refusals are logged and simply retried on a later tick.</p>"""),
("void tryTakeoff()", "tryTakeoff()",
 """<p>Altitude = 5 m above launch. Lat/lon left 0 (float32 would lose ~0.4 m; Copter ignores them in
 GUIDED). Yaw = NaN, because the MAVLink yaw field is a compass angle (NED), not our ENU yaw: sending
 our value would be in the wrong convention.</p>"""),
("double poseYaw() const", "Telemetry helpers", """<p>Yaw from the quaternion, position, speed, freshness.</p>"""),
("/// Downward look angle", "poiPitchDeg()",
 """<p>Angle below the horizon from the drone to the POI: atan2(height, horizontal distance). Only
 logged (54–64° in flight), because the camera is fixed.</p>"""),
("/// \"Usable fix\":", "fixGood(): is the GPS fix usable?",
 """<p>Fresh message, status ≥ FIX, finite lat/lon, enough satellites, and horizontal σ (from the
 covariance) ≤ 2 m. The reason for "no" is returned for the log.</p>"""),
("// ----------------------------------------------- origin (fix averaging)", "Fix averaging",
 """<p>Sums of lat/lon/alt and a count; each <b>new</b> fix (sequence number) is added once, because
 the 30 Hz tick runs faster than the GPS.</p>"""),
("// ---------------------------------------------------------- the origin", "captureOrigin(): define the mission",
 """<ul><li>Origin = mean of the stable fixes; the local tangent plane is built there; the local pose at
 that moment is the launch point.</li>
 <li>POI from lat/lon (geodetic → local) or from the offset. Corners around the POI, flight z =
 launch z + 5 m.</li>
 <li>Geofence check again (now with the real POI).</li>
 <li>The square list (closed: back to W1), log the origin, compare with MAVROS home, print the plan in
 lat/lon, publish the intended path.</li></ul>"""),
("Vec3 toLocal(const outdoor::Geodetic& g) const", "toLocal() / toGeodetic()",
 """<p>Tangent-plane ENU offsets plus the launch point = local coordinates, and back. Both directions
 are required by the assignment.</p>"""),
("/// Both conversion directions exercised at startup", "reportGeodeticPlan()",
 """<p>For the POI and every corner: local → lat/lon → local again, printing the round-trip error
 (~1e-9 m). Proof that both conversions work.</p>"""),
("void publishIntendedPath()", "publishIntendedPath()", """<p>The square and 200 points of the figure-8 as a Path for RViz.</p>"""),
("// ----------------------------------------------------------- motion", "flyCarrotTo(): smooth carrot",
 """<p>A carrot like in A1.2, plus a <b>trapezoidal speed profile</b>: speed rises by at most
 <code>max_accel·dt</code> per tick and is capped by <code>√(2·a·d)</code> (the speed from which it can
 still stop at the goal). The carrot waits while the drone is more than <code>max_lead</code> behind.
 Its own velocity is sent as feed-forward. Without the profile the velocity jumped 0 ↔ 0.5 m/s at each
 corner and the drone dipped its nose.</p>"""),
("void slewYawTo(double target, double dt)", "slewYawTo()", """<p>Turns the commanded yaw towards a target by at most 45°/s (shortest way, kept continuous).</p>"""),
("/// Transit leg: nose along the direction of travel", "Yaw on transit legs",
 """<p>Nose along the direction to the goal; within <code>transit_turn_radius</code> (0.7 m) of the goal
 turn to the heading the next phase needs.</p>"""),
("/// Yaw continuously aimed at the POI", "updateYawToPoi(): nose on the POI",
 """<p>Every tick: bearing from the <b>measured</b> position (the carrot is ahead of the drone), shifted
 by velocity × 0.5 s to compensate the yaw controller lag. Near the POI (&lt; 0.5 m) the bearing is
 undefined, so the yaw is held. Result: max 3.3° error on the square.</p>"""),
("void publishSetpoint()", "publishSetpoint(): PositionTarget",
 """<p><code>FRAME_LOCAL_NED</code> but MAVROS converts from ENU. The <code>type_mask</code> ignores
 acceleration and yaw rate (and velocity when feed-forward is off); position, velocity and yaw are
 used. <b>type_mask must not ignore yaw</b>, or the heading is not controlled.</p>"""),
("void tick()", "tick(): the 30 Hz loop",
 """<p>dt (clamped), status line once per second, stop in terminal states, service timeout (retry
 instead of hanging), safety checks, then the state's tick function. Setpoints are published only in
 streaming states and <b>not during TAKEOFF</b> (same reason as A1.2: a setpoint cancels the
 takeoff). Then the flown path and a CSV row.</p>"""),
("/// Returns false if the tick must stop here", "safetyChecks() 1: telemetry and abort",
 """<p>Stale pose: in the air → STOPPED (we cannot control safely), on the ground → FAULT. The abort
 flag from the service: airborne → ABORTING, on the ground → FAULT.</p>"""),
("if (isStreaming(state_)) {", "safetyChecks() 2: pilot, link, fence",
 """<p>Not connected / not armed / mode not GUIDED → STOPPED (the pilot wins). Our own geofence (4 m
 horizontally, 5.5 m up) and a tracking error over 2 m → ABORT.</p>"""),
("// Per-state timeouts", "safetyChecks() 3: timeouts",
 """<p>Every state has a limit (the figure-8's depends on ω, laps and ramp). Exceeded: abort in the air,
 fault on the ground.</p>"""),
("void tickWaitGps()", "WAIT_GPS",
 """<p>Connected, pose, usable fix; the stable window starts and fixes are averaged; after 3 s of an
 uninterrupted good fix the origin is captured and SET_MODE follows. Any bad fix restarts the
 window.</p>"""),
("void tickTakeoff(double)", "TAKEOFF",
 """<p>Send the takeoff once; confirmed when within 0.2 m of 5 m and slower than 0.15 m/s for 2 s (no
 fixed sleep, as the assignment requires). Then the carrot starts at the current position.</p>"""),
("/// Transit from the launch point to W1", "TO_START",
 """<p>Fly to W1 with the nose along the direction of travel, turn to the POI near W1, and start the
 square only once locked on (&lt; 5°), so the POI error is not polluted by the arrival turn.</p>"""),
("void tickSquare(double dt)", "SQUARE",
 """<p>Nose on the POI every tick, carrot to the next corner; the last corner leads to TO_CENTER.</p>"""),
("void tickToCenter(double dt)", "TO_CENTER",
 """<p>Fly to the POI, turn to the figure-8's starting tangent (45°), and wait until the drone is there,
 aligned (&lt; 10°) and calm. This avoids the "lunge" warned about in the assignment.</p>"""),
("void tickFigure8(double)", "FIGURE8: the continuous trajectory",
 """<ul><li>Time since start → phase θ with soft start and soft stop (<code>figure8Phase</code>).</li>
 <li>Position = POI + (A sin θ, A sin θ cos θ); velocity = derivative × dθ/dt (analytic
 feed-forward).</li>
 <li>Yaw = tangent <code>atan2(cos 2θ, cos θ)</code>, evaluated slightly ahead and unwrapped.</li>
 <li>At the end: hold the end point until the drone arrives (or a timeout), so the lap is not cut
 short, then RETURN.</li></ul>"""),
("void tickReturn(double dt)", "RETURN", """<p>Fly back above the launch point (nose along travel), then LAND.</p>"""),
("/// While we switch GUIDED -> LAND / abort mode", "pilotTookOver()",
 """<p>While we are switching modes, any mode other than GUIDED or ours means the pilot took over:
 stop instead of switching back every second.</p>"""),
("void tickLand()", "LAND", """<p>Switch to LAND; disarmed → DONE.</p>"""),
("void tickAborting()", "ABORTING", """<p>Switch to the abort mode; once there the node is passive (ABORTED).</p>"""),
("// ------------------------------------------------ status / logging output", "printStatus(): the assignment's print",
 """<p>Once per second: lat/lon/alt, position from launch from the pose <b>and</b> from GPS, rel_alt,
 yaw from the pose and from the compass converted with <code>90° − heading</code> and their
 difference (~0.14°), satellites. This is exactly the "print at any moment" acceptance check.</p>"""),
("void appendFlownPath()", "appendFlownPath()", """<p>Pose every 0.2 s into a Path for RViz (max 20 000).</p>"""),
("void writeLogRow()", "writeLogRow()",
 """<p>One CSV row per tick. Commanded values are NaN when no setpoint is being sent (takeoff,
 landing), so the plots do not show fake errors. The POI bearing error is computed during SQUARE.</p>"""),
("// ------------------------------------------------------------- members", "Members",
 """<p>Parameters, state and timers, telemetry, origin averaging, service flags, mission geometry
 (launch, command, carrot, velocity, POI, corners), output, ROS handles.</p>"""),
("int main(int argc, char** argv)", "main(): single-threaded on purpose",
 """<p><code>rclcpp::spin</code> uses one thread: callbacks and the timer never run at the same time, so
 no mutex is needed. A1.2 chose the multi-threaded pattern instead.</p>"""),
]
