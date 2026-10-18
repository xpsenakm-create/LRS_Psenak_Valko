FILE = "src/lrs_psenak_valko/config/planner.yaml"
TITLE = "Planner parameters (config/planner.yaml)"
INTRO = """<p>The planner's ROS parameters, loaded with <code>--params-file</code>. The run script
reads it straight from <code>src/</code>, so a change takes effect at the next run without a
rebuild.</p>"""

NOTES = [
("map_planner_node:", "Node name and map",
 """<p>The top-level key must equal the node name; <code>ros__parameters</code> holds the values.
 <code>map_path</code> is relative to the installed package.</p>"""),
("    voxel_size: 0.15", "Voxel size and safety radius",
 """<p><b>voxel_size 0.15 m</b>: resolution of the map and of A*. <b>safety_radius 0.45 m</b> = 0.33 m
 drone radius (centre to propeller tip) + ~0.12 m tracking margin; raised from 0.30 after a test
 flight passed 0.37 m from a rack.</p>"""),
("    occupancy_pcd_path:", "Saved occupancy and racks",
 """<p>Where the inflated occupancy is saved (the script overrides it with an absolute path).
 <code>require_rack_boxes</code>: refuse to start without racks. <code>rack_boxes</code>: two racks,
 each centre (x, y, z), size (6.50 × 1.05 × 3.76 m) and yaw 0.</p>"""),
("    allow_diagonal: true", "A* settings",
 """<p>26-connected, optimal (<code>heuristic_weight 1.0</code>), 5 s limit, no expansion limit,
 snap blocked endpoints up to 5 voxels (0.75 m), simplify the path.</p>"""),
]
