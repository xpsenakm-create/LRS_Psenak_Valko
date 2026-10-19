FILE = "plot_a1_2.py"
TITLE = "Flight plots and checks (plot_a1_2.py)"
INTRO = """<p>Reads one run folder and produces <code>topview.png</code> and
<code>overview.png</code>, plus a terminal summary: arrival error per waypoint, minimum clearance
to obstacles, maximum deviation from the planned route. This is how "no collisions" and "hard
waypoints reached" are <b>measured</b>, not just watched in Gazebo.</p>"""

NOTES = [
("#!/usr/bin/env python3", "Docstring: what the script does",
 """<p>The module docstring is the manual: inputs, figures, the definition of clearance, and
 usage. <code>python3 plot_a1_2.py</code> without arguments plots the newest run.</p>"""),
("import argparse", "Imports and constants",
 """<ul><li><code>argparse</code>: command-line options; <code>csv</code>: reading the logs;
 <code>glob</code>: finding the newest run; <code>re</code>: reading two values from the YAML.</li>
 <li><code>numpy</code>: all data are numpy arrays (fast vector maths).</li>
 <li>Paths to the map, the planner YAML and the runs folder, relative to the script.</li>
 <li><code>HARD_RADIUS</code>/<code>SOFT_RADIUS</code>: the executor's tolerances, for the
 tolerance markers.</li>
 <li><code>STATE_COLORS</code>: background colours for the state shading.</li></ul>"""),
("def read_csv(path):", "read_csv() and load_flight()",
 """<p><code>csv.DictReader</code> gives one dict per row (column name → text). Rows without a
 time are dropped. <code>load_flight</code> turns every numeric column into a numpy array
 (empty/"nan" → NaN) and keeps <code>state</code> and <code>mode</code> as text arrays.</p>"""),
("def load_paths(path):", "load_paths()",
 """<p>Groups the rows of <code>planned_paths.csv</code> by leg number: each leg has its waypoint
 number and an N×3 array of points.</p>"""),
("def load_mission(path):", "load_mission()",
 """<p>A small mission CSV reader (same format as the C++ parser, less strict: the C++ parser has
 already validated the file).</p>"""),
("def load_planner_yaml(path):", "load_planner_yaml()",
 """<p>Reads only <code>safety_radius</code> and <code>rack_boxes</code> with regular
 expressions, so PyYAML is not needed. The rack list is split into groups of 7.</p>"""),
("def load_pcd_ascii(path):", "load_pcd_ascii()",
 """<p>A PCD file is a text header then the points. Counts header lines up to
 <code>DATA</code>, then <code>np.loadtxt</code> reads x y z (620 k rows, a few seconds).</p>"""),
("def box_surface_points(box, step=0.1):", "box_surface_points()",
 """<p>Samples the six faces of a rack box on a 10 cm grid (rotated by its yaw). The racks are
 then obstacles even where the point cloud has holes.</p>"""),
("def box_corners_xy(box):", "box_corners_xy()",
 """<p>The rotated rectangle of a box for the top view (5 points, closed outline).</p>"""),
("def airborne_mask(data):", "airborne_mask()",
 """<p>True where the drone is armed and higher than 0.3 m. Only airborne samples count for
 collision risk.</p>"""),
("def clearance(data, obstacles):", "clearance(): distance to the nearest obstacle",
 """<p>Builds a <b>k-d tree</b> (<code>scipy.spatial.cKDTree</code>) over all obstacle points.
 A query then finds the nearest point in O(log n) instead of comparing with 600 k points. One
 query per logged pose gives the distance from the drone centre to the nearest wall, pillar, truss
 or rack. The floor is excluded by the caller; altitude is plotted separately.</p>"""),
("def ground_spans(data):", "ground_spans()",
 """<p>Finds the time intervals spent on the ground <b>after</b> the first takeoff (landtakeoff,
 final landing), longer than 1 s. They are labelled in every panel.</p>"""),
("def label_ground(ax, data, spans):", "label_ground()",
 """<p>Writes "on ground (landtakeoff)" or "landed (end)" at the top of each such interval
 (<code>get_xaxis_transform()</code>: x in data units, y as a fraction of the axis
 height).</p>"""),
("def arrivals(data, mission):", "arrivals(): accuracy at each waypoint",
 """<p>For every waypoint, the first log row where the executor was in TASK for it is the moment
 it declared "reached". The 3D distance from the logged position to the mission point is the
 arrival error (plotted against the tolerance). None = never reached.</p>"""),
("def route_deviation(data, legs):", "route_deviation(): distance to the planned line",
 """<p>For every sample in NAVIGATE/ALIGN_PATH: the distance from the drone to the nearest point
 of the planned route of that waypoint. Vectorised point-to-segment distance: project onto each
 segment (<code>u</code> clamped to [0, 1]), take the minimum. Measured 0.56 m before carrot
 following, ≤ 0.15 m along the climbs with it.</p>"""),
("def break_wraps(deg):", "break_wraps()",
 """<p>Yaw jumps from +180° to −180° when it crosses the back. A line plot would draw a vertical
 stroke there; inserting NaN breaks the line instead.</p>"""),
("def shade_states(ax, data):", "shade_states()",
 """<p>Colours the background by state (takeoff blue, align orange, task green, landing red),
 merging consecutive rows with the same state into one span.</p>"""),
("def plot_topview(ax, data, legs, mission, boxes, safety, obstacles_xy):", "plot_topview()",
 """<ul><li>Grey dots: map points between 0.3 and 4 m height (walls, pillars).</li>
 <li>Racks filled red; dashed outline = rack grown by the safety radius.</li>
 <li>Purple dashed: every planned A* route; blue: flown track while armed.</li>
 <li>Arrows every ~3 s: drone heading, showing that it faces the direction of travel.</li>
 <li>Waypoints: square = hard, circle = soft, with task and z; labels of waypoints at the same
 place are stacked.</li></ul>"""),
("def main():", "main(): options and loading",
 """<p>Options: run folder, map, YAML, drone radius (0.33 m = centre to propeller tip),
 <code>--no-show</code>. Without a folder: newest run. The <code>Agg</code> backend renders to
 files only (used by the run script); otherwise an interactive window backend.</p>"""),
("    data = load_flight(os.path.join(run_dir, \"flight_log.csv\"))", "Load everything",
 """<p>Flight log, routes, mission and planner values.</p>"""),
("    cloud = load_pcd_ascii(args.map)", "Build the obstacle set",
 """<p>Map points above 15 cm (floor excluded) plus the rack surfaces, rounded to 5 cm and
 de-duplicated (<code>np.unique</code>), which makes the k-d tree smaller and faster. A second
 set (0.3–4 m) is used for the top-view drawing.</p>"""),
("    clear_all = clearance(data, obstacles)", "Compute the checks",
 """<p>Clearance for every sample; only airborne samples count as a risk. Plus ground intervals
 and arrival errors.</p>"""),
("    # ---- summary in the terminal", "Terminal summary",
 """<p>Duration, final state, per-waypoint error vs tolerance (or NOT REACHED), and the minimum
 airborne clearance with its time and position. "OK" if it is larger than the drone radius,
 otherwise "COLLISION RISK".</p>"""),
("    # ---- top view", "Top-view figure",
 """<p>Window title with the run name, draw, save <code>topview.png</code> at 150 dpi.</p>"""),
("    # ---- time series", "Overview figure: altitude",
 """<p>A 3×2 grid. Altitude: measured z, commanded z (only where a setpoint was really sent,
 <code>cmd_valid</code>; hence the gaps during takeoff/landing), and markers when each waypoint
 was reached.</p>"""),
("    ax = axs[0, 1]", "Heading panel",
 """<p>Measured vs commanded yaw (wrap jumps removed), from −180° to 180° in 45° steps.</p>"""),
("    ax = axs[1, 0]", "Distance to target and speed",
 """<p>Distance between the drone and the point it is flying to (with the carrot it stays ≤ the
 1 m lead), and the speed.</p>"""),
("    ax = axs[1, 1]", "Clearance panel",
 """<p>Green = airborne clearance, grey = on the ground; red dashed = drone radius; orange dotted
 = planner safety radius. The green line must stay above the red one.</p>"""),
("    ax = axs[2, 0]", "Arrival error bars",
 """<p>One bar per waypoint (green within tolerance, red outside), black marks = tolerance (0.15
 hard, 0.55 soft).</p>"""),
("    ax = axs[2, 1]", "Route deviation panel",
 """<p>Distance to the planned route while navigating, with the margin the safety radius leaves
 beyond the drone radius (safety_radius − 0.33 m). The maximum is printed.</p>"""),
("    for a in axs[:2].ravel():", "Finish and save",
 """<p>Axis labels and grids, the title, the explanation of the gaps at the bottom
 (<code>fig.text</code>), <code>tight_layout</code> leaving room for it, save
 <code>overview.png</code>, and show the windows unless <code>--no-show</code>.</p>"""),
("if __name__ == \"__main__\":", "Entry point",
 """<p>Runs <code>main()</code> only when the file is executed, not when it is imported (the 3D
 viewer imports it for its loaders).</p>"""),
]
