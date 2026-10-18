FILE = "git:origin/A1_3:run_a1_3.sh"
TITLE = "Runner (run_a1_3.sh, A1.3)"
INTRO = """<p>Opens Gazebo (outdoor world), SITL, MAVROS and the mission node in their own terminals,
optionally records a rosbag and the CSV log into <code>results/A1_3/&lt;run&gt;/</code>, and plots after
Ctrl+C.</p>"""

NOTES = [
("#!/usr/bin/env bash", "Header and strict mode", """<p>Slovak comments; <code>set -euo pipefail</code> as in our scripts.</p>"""),
("ROOT_DIR=", "Finding LRS-URK and the workspace",
 """<p><code>resolve_lrs_urk</code> tries <code>LRS_URK_DIR</code>, folders near the script and
 common places in <code>$HOME</code>, and returns the first that contains
 <code>scripts/run_gazebo.sh</code>. <code>resolve_install_setup</code> looks for
 <code>install/setup.bash</code>.</p>"""),
("WORLD=\"fei_lrs_outdoor.world\"", "Defaults",
 """<p>The outdoor world and the SITL home (must match the world's georeference). Options for geometry,
 POI and recording start empty.</p>"""),
("usage() {", "Usage text", """<p>All options: world, square, amplitude, altitude, POI as lat/lon or offset, <code>--record</code>.</p>"""),
("while (($#)); do", "Argument parsing",
 """<p>A <code>case</code> per option; <code>--record</code> takes an optional name (default
 <code>run_&lt;date&gt;</code>); unknown arguments are passed to the node.</p>"""),
("# Pri --record ide všetko", "Run folder and checks",
 """<p>The run folder must not exist yet; POI lat and lon must be given together; LRS-URK must be
 found.</p>"""),
("ROS_SETUP=", "Source ROS and build", """<p>Source ROS 2, <code>colcon build</code>, source the workspace.</p>"""),
("open_terminal() {", "open_terminal()", """<p>Opens a command in gnome-terminal or xterm (or in the background) and waits for Enter at the end.</p>"""),
("cleanup() {", "cleanup(): close the recording properly",
 """<p>SIGINT to the node and to <code>ros2 bag record</code> (the bag writes its metadata), wait up to
 10 s for <code>metadata.yaml</code>, then plot from the CSV. The simulator stays open unless
 <code>KILL_ALL=1</code>.</p>"""),
("# 1. Gazebo", "Start the simulation",
 """<p>Gazebo with the outdoor world, then SITL with the matching home, then MAVROS, with short pauses.
 (Our A1.2 script waits for real readiness signals instead of fixed sleeps.)</p>"""),
("to_double() {", "Node parameters",
 """<p>Converts options to floating-point text (ROS rejects "5" for a double parameter) and collects
 the <code>-p</code> arguments.</p>"""),
("# Geofence automaticky prispôsobíme", "Automatic geofence for other sizes",
 """<p>For a non-default geometry, computes the required radius (POI offset + max(half-diagonal of the
 square, A) + 1 m) and altitude limit, so the node's own pre-flight check does not reject the
 mission.</p>"""),
("if ((RECORD_BAG)); then", "Log path and node start",
 """<p>With recording, the CSV goes into the run folder. Then the node is started in its own
 terminal.</p>"""),
("# Voliteľný bag.", "Rosbag",
 """<p>An explicit topic list: pose, velocity, setpoints, intended/flown path, mission state, GPS,
 rel_alt, compass, home, MAVROS state.</p>"""),
("echo \"\"", "Summary and wait", """<p>Prints the abort command and waits until Ctrl+C (which runs cleanup).</p>"""),
]
