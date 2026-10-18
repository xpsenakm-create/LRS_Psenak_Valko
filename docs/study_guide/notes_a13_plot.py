FILE = "git:origin/A1_3:plot_rosbag.py"
TITLE = "Plots from CSV or rosbag (plot_rosbag.py, A1.3)"
INTRO = """<p>Reads the node's CSV log or a rosbag and draws the square with the POI, the figure-8, the
yaw error and a six-panel overview. Slovak labels.</p>"""

NOTES = [
("#!/usr/bin/env python3", "Docstring and imports", """<p>What is drawn and how to call it; matplotlib is required.</p>"""),
("def parse_csv_log(csv_path):", "parse_csv_log()",
 """<p>Reads every CSV row into lists; empty/NaN commanded values become NaN, incomplete rows (from
 Ctrl+C) are skipped.</p>"""),
("def parse_rosbag(bag_path):", "parse_rosbag(): open the bag",
 """<p><code>rosbag2_py.SequentialReader</code> with CDR serialization, then a map topic → type.</p>"""),
("    while reader.has_next():", "Read the messages",
 """<p>Deserialize pose (yaw from the quaternion), setpoints (yaw wrapped) and the state topic, with
 times relative to the first message.</p>"""),
("    # Setpointy a stavy chodia v iných časoch ako póza", "Align setpoints and states with the pose",
 """<p>For each pose sample, take the last setpoint and state before it (zero-order hold). A setpoint
 older than 0.5 s is treated as "not streaming" (NaN), so takeoff/landing do not show fake errors.
 Then the POI and tracking errors.</p>"""),
("def filter_by_states(data, valid_states):", "filter_by_states()", """<p>Keeps only the samples of chosen phases (e.g. SQUARE).</p>"""),
("def detect_geometry(data, user_square=None, user_amp=None):", "detect_geometry()",
 """<p>Square side and amplitude from the options, or estimated from the commanded x during SQUARE /
 FIGURE8.</p>"""),
("def plot_single_square(data_sq, square_side=5.0, output_file=None):", "Square plot", """<p>Flown track, POI, corners and heading arrows.</p>"""),
("def plot_single_figure8(data_f8, amplitude=3.0, output_file=None):", "Figure-8 plot", """<p>Ideal curve vs flown track.</p>"""),
("def plot_single_yaw_error(data, output_file=None):", "Yaw error plot",
 """<p>|nose − bearing to POI| during the square and |nose − tangent| during the figure-8, with mean and
 maximum in the legend and a 10° reference line.</p>"""),
("def plot_comprehensive(data, square_side=5.0, amplitude=3.0, output_file=None, show=True):", "Six-panel overview",
 """<p>Square, figure-8, yaw error, yaw over time, altitude, tracking error.</p>"""),
("def main():", "main()",
 """<p>CSV or bag (or the default CSV), geometry, the three single plots, the overview.</p>"""),
]
