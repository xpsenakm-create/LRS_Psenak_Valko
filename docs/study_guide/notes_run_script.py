FILE = "run_a1_2.sh"
TITLE = "One-command runner (run_a1_2.sh)"
INTRO = """<p>A Bash script that does everything an operator would otherwise do in five terminals:
build, start Gazebo → SITL → MAVROS with readiness checks between them, start the planner and the
executor, record a rosbag, and plot at the end. It is not part of the "robot"; it is the test
harness around it.</p>"""

NOTES = [
("#!/usr/bin/env bash", "Shebang and usage header",
 """<p>The first line tells Linux to run the file with <code>bash</code> (found via
 <code>env</code> on the PATH). The comment block is the manual: usage and every
 <code>A1_2_*</code> option with its default in brackets.</p>"""),
("set -euo pipefail", "Strict mode",
 """<ul><li><code>-e</code>: stop at the first command that fails.</li>
 <li><code>-u</code>: using an unset variable is an error (catches typos).</li>
 <li><code>-o pipefail</code>: a pipeline fails if any part fails, not just the last.</li></ul>"""),
("ROOT_DIR=\"$(cd -- \"$(dirname -- \"${BASH_SOURCE[0]}\")\" && pwd)\"", "Paths and options",
 """<ul><li><code>ROOT_DIR</code>: the absolute folder of the script, so it works from any current
 directory.</li>
 <li><code>MISSION_FILE="${1:-default}"</code>: first argument, or the example mission.</li>
 <li><code>${A1_2_X:-default}</code>: take the environment variable if set, else the default. Every
 option follows this pattern.</li>
 <li>The PIDs start empty; cleanup only stops what was started.</li></ul>"""),
("# BEGIN_FUNCS", "Small helper functions",
 """<ul><li><code>log</code>/<code>warn</code>: timestamped messages (warn to stderr).</li>
 <li><code>is_uint</code>/<code>is_number</code>: regex checks for option values.</li>
 <li><code>float_ge</code>: Bash only does integer maths, so <code>awk</code> compares
 decimals.</li></ul>"""),
("# First /world/<name>/stats topic", "Talking to Gazebo and checking processes",
 """<ul><li><code>gz_stats_topic</code>: lists Gazebo topics and picks
 <code>/world/&lt;name&gt;/stats</code>. Empty = Gazebo is not running. <code>timeout</code> stops
 a hanging <code>gz</code> call.</li>
 <li><code>gz_stat</code>: reads one stats message and extracts one field
 (<code>real_time_factor</code>, <code>paused</code>) with awk.</li>
 <li><code>sitl_running</code>: is an <code>arducopter</code> process alive?
 <code>mavros_running</code>: does the <code>/mavros/set_mode</code> service exist?</li></ul>"""),
("wait_for_log() { # file pattern timeout_s", "wait_for_log()",
 """<p>Polls a file every 2 s until a regex appears or the timeout passes (<code>SECONDS</code> =
 Bash's built-in seconds-since-start counter).</p>"""),
("# Factor by which wall-clock timeouts must grow", "Time scaling",
 """<p>If Gazebo runs at real-time factor r &lt; 0.8, the executor's wall-clock timeouts are
 multiplied by ⌈1/r⌉ (max 10). Example: r = 0.28 → ×4. <code>scaled v s</code> prints v·s with
 one decimal.</p>"""),
("usage() {", "Usage text",
 """<p>Printed for <code>-h</code>/<code>--help</code> or wrong arguments.</p>"""),
("if (($# > 1)); then usage >&2; exit 2; fi", "Check arguments and options",
 """<ul><li>At most one argument; <code>--help</code> prints usage.</li>
 <li>A relative mission path is made absolute (relative to where you ran the script); it must
 exist.</li>
 <li>Timeouts must be positive integers; <code>MIN_RTF</code> a number; on/off options 0 or 1.
 <code>${!name}</code> = indirect expansion (the value of the variable whose name is in
 <code>name</code>).</li>
 <li>When starting the simulator, the three LRS-URK scripts must exist and be executable.</li>
 <li>Exit code 2 = usage error.</li></ul>"""),
("ROS_SETUP=\"/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash\"", "Source ROS 2",
 """<p>Loads ROS 2 Jazzy into this shell (PATH, Python paths...). <code>set +u</code> around it:
 ROS's setup script uses unset variables, which strict mode would reject.</p>"""),
("# Build first, before the simulator exists", "Build the package",
 """<p><code>colcon build --packages-select lrs_psenak_valko</code> compiles only our package.
 It runs <b>before</b> the simulator: compiling uses all CPU cores and would slow Gazebo,
 which causes "Accels/Gyros inconsistent" pre-arm failures. With <code>A1_2_SKIP_BUILD=1</code>
 it only checks that the executable exists. Then source our workspace
 (<code>install/setup.bash</code>) so <code>ros2 run lrs_psenak_valko ...</code> works.</p>"""),
("stop_group() { # pid [signal]", "stop_group(): reliable stopping",
 """<p>Processes are started with <code>setsid</code>, so each one leads its own process group;
 <code>kill -SIG -- -PID</code> signals the whole group (<code>ros2 run</code> plus the real
 node). It escalates: the given signal, then TERM, then KILL, waiting up to 10 s each. It was
 added because <code>ros2 bag record</code> once ignored SIGINT and hung the cleanup.</p>"""),
("cleanup() {", "cleanup(): runs on exit, Ctrl+C or error",
 """<ul><li><code>trap -</code>: disarm the trap so cleanup cannot run twice.</li>
 <li>Stop executor, then rosbag with <b>SIGINT</b> (that lets it close the bag file properly),
 planner, RViz.</li>
 <li>If a flight log exists: run the plots (PNG + terminal summary), then open the plot windows
 and the 3D view <b>detached</b> (<code>setsid ... &amp; disown</code>), so they stay open after the
 script ends.</li>
 <li>With <code>A1_2_STOP_SIM=1</code>: kill MAVROS, SITL, MAVProxy and Gazebo by name.</li></ul>
 <p><code>trap cleanup EXIT INT TERM</code> registers it for normal exit, Ctrl+C and kill.</p>"""),
("# ---------------------------------------------------------------- simulator start-up", "pick_terminal()",
 """<p>Chooses how to open new windows: none without a display; otherwise gnome-terminal, xterm or
 x-terminal-emulator, whichever exists (<code>command -v</code>).</p>"""),
("# launch_component NAME SCRIPT:", "launch_component(): one simulator window",
 """<ul><li>Writes a small launcher script into <code>log/a1_2_sim/</code> that runs the LRS-URK
 script through <code>script -qefc</code>. <code>script</code> records the output to a log
 <b>and</b> keeps a real terminal, which MAVProxy needs.</li>
 <li>At the end the window waits for Enter, so you can read errors.</li>
 <li>Opens it with <code>bash -ic</code>: an <i>interactive</i> shell reads
 <code>~/.bashrc</code> (ROS and ArduPilot environment), exactly like a terminal you open
 yourself.</li></ul>"""),
("wait_gazebo_realtime() {", "wait_gazebo_realtime()",
 """<p>Every 2 s reads <code>paused</code> and <code>real_time_factor</code>:</p><ul>
 <li>paused: warn once ("press Play").</li>
 <li>RTF ≥ 0.8 three times in a row: good, continue.</li>
 <li>RTF below half the target five times: this PC will not get faster by waiting; continue
 after ~15 s instead of the full timeout (this used to waste 90 s every run).</li>
 <li>Timeout: warn and continue anyway.</li></ul>"""),
("if [[ \"$START_SIM\" == 1 ]]; then", "1) Gazebo",
 """<p>Reuse a running Gazebo (stats topic exists), or open <code>run_gazebo.sh</code> and wait up
 to 120 s for the world. Then wait for real-time speed.</p>"""),
("# 2) ArduPilot SITL", "2) SITL",
 """<p>Started only after Gazebo runs, because SITL's sensors come from Gazebo (lock-step). Waits
 for MAVProxy's own boot lines (<code>Detected vehicle</code>, <code>Received N
 parameters</code>). MAVProxy's <code>--console</code> shows the FCU's PreArm/EKF messages only in
 its GUI window, so they cannot be read from the log. The old script waited for them and always
 ran into a 240 s timeout. If the process runs but prints nothing for 60 s, continue anyway.</p>"""),
("# 3) MAVROS", "3) MAVROS",
 """<p>Reuse or open <code>run_mavros.sh</code> (connects to SITL on UDP 14550).</p>"""),
("# ---------------------------------------------------------------- planner + MAVROS link", "Start the planner",
 """<p>Runs <code>map_planner_node</code> with <code>config/planner.yaml</code> straight from
 <code>src/</code> (no rebuild needed after editing it), and tells it where to save the inflated
 occupancy PCD. <code>&amp;</code> = in the background; <code>$!</code> = its PID.</p>"""),
("printf 'Waiting for map planner and MAVROS", "Readiness loop",
 """<p>Every <code>POLL_INTERVAL</code> seconds, until <code>READY_TIMEOUT</code>:</p><ul>
 <li>the planner process must still live (else exit with its error);</li>
 <li><code>/map_planner_node</code> exists in <code>ros2 node list</code>;</li>
 <li>the four MAVROS services exist;</li>
 <li><code>/mavros/state</code> says <code>connected: true</code>;</li>
 <li>a <code>/mavros/local_position/pose</code> message arrives.</li></ul>
 <p>Progress is printed every 10 s. Not ready in time: exit with an explanation.</p>"""),
("# GPS fix / EKF readiness is NOT waited for here", "Time scaling of the executor",
 """<p>GPS/EKF readiness is left to the executor (it has the FCU messages). The script measures
 the current RTF and computes the scale (or uses <code>A1_2_TIME_SCALE</code>). The executor
 arguments are collected in a Bash <b>array</b> (<code>EXEC_ARGS</code>), so values with spaces
 stay intact. If the scale is ≥ 1.5, every wall-clock timeout is multiplied, and
 <code>cruise_speed</code> is <b>divided</b>, so the carrot moves at ~1 m/s of simulated
 time.</p>"""),
("# ---------------------------------------------------------------- run folder, rosbag, RViz", "Run folder, rosbag, RViz",
 """<ul><li>Folder <code>log/a1_2_runs/&lt;date&gt;_&lt;time&gt;_&lt;mission&gt;</code>, with a copy of the
 mission; its path goes to the executor as <code>log_dir</code>.</li>
 <li><code>ros2 bag record -o</code> with an explicit topic list (state, pose, velocity,
 setpoints, FCU texts, planner requests/answers, markers). <code>/mission/flown_path</code> was
 removed from the list: it re-sends the whole track 5× per second and made the bag 38 MB.</li>
 <li>Optional RViz with our layout.</li></ul>"""),
("printf 'Starting mission: %s\\n' \"$MISSION_FILE\"", "Start the executor and wait",
 """<p>Runs the executor with all collected arguments and waits for it. <code>wait PID ||
 EXEC_RC=$?</code> stores the exit code without triggering <code>set -e</code>. Exit 0 = mission
 DONE (the executor exits by itself). Then the script ends, which runs <code>cleanup</code>
 (plots).</p>"""),
]
