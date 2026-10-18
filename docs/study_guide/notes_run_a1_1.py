FILE = "run_a1_1.sh"
TITLE = "Planner runner (run_a1_1.sh, A1.1)"
INTRO = """<p>Builds the package, starts the planner and RViz, waits until the node exists and sends one
route request. No simulator is needed for A1.1.</p>"""

NOTES = [
("#!/usr/bin/env bash", "Defaults",
 """<p>Strict mode, default start (12, 4, 2) and goal (16, 4, 2), RViz on.</p>"""),
("usage() {", "Options",
 """<p><code>--safety-radius</code> (aliases <code>--inflation-radius</code>, <code>--radius</code>),
 <code>--start X Y Z</code>, <code>--goal X Y Z</code>, <code>--no-rviz</code>.</p>"""),
("while (($#)); do", "Parsing and validation",
 """<p>Reads the options; the radius must be a number; start and goal only together; every coordinate
 must match a number regex.</p>"""),
("ROS_SETUP=", "ROS and build", """<p>Source ROS, build only our package, source the workspace.</p>"""),
("PCD_OUTPUT=", "Cleanup", """<p>Where the inflated map is saved; on exit RViz and the node are stopped.</p>"""),
("ROS_ARGS=(--ros-args", "Start the planner and RViz",
 """<p>The YAML parameters, the PCD output path, an optional radius override; RViz with our layout.</p>"""),
("READY=0", "Wait and request",
 """<p>Up to 20 s for <code>/map_planner_node</code>, then one <code>ros2 topic pub --once
 /plan_request</code> with start and goal. The planner keeps running for more requests until
 Ctrl+C.</p>"""),
]
