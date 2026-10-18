FILE = "src/lrs_psenak_valko/include/lrs_psenak_valko/a_star_planner.h"
TITLE = "A* interface (a_star_planner.h, A1.1)"
INTRO = """<p>The types and the class that the planner node uses. Reading the header first gives
the overview; the implementation follows in the next chapter.</p>"""

NOTES = [
("// a_star_planner.h", "Purpose and includes",
 """<p>3D A* on a 26-connected voxel grid, independent of ROS and PCL: it only needs the grid
 geometry and a function that says whether a voxel is blocked.</p>"""),
("namespace astar", "Vec3",
 """<p>A minimal 3D point (x, y, z in metres).</p>"""),
("/// Geometry of the (already inflated) voxel grid.", "GridInfo",
 """<p>Voxel size, world coordinates of the minimum corner of voxel (0, 0, 0), and the number of
 voxels per axis.</p>"""),
("using OccupancyFn = std::function<bool(int, int, int)>;", "OccupancyFn",
 """<p>"Is voxel (x, y, z) blocked?" as a function object. The node passes a lambda that asks
 the inflated voxel grid. This decouples A* from the map data structure.</p>"""),
("struct Params", "Params",
 """<p>26 or 6 neighbours, heuristic weight (1 = optimal), time and expansion limits, snapping
 radius, and whether to simplify the path.</p>"""),
("enum class Status", "Status", """<p>All possible outcomes of a search.</p>"""),
("struct Result", "Result",
 """<p>Status, path, timing and statistics (expansions, lengths before/after simplification),
 and diagnostics (snapping, closest point to the goal) for the human-readable report.</p>"""),
("class AStarPlanner", "The class",
 """<p>The constructor copies the occupancy once (<code>refresh()</code>); <code>plan()</code> is
 <code>const</code>: it does not modify the planner, so the same planner could be used from
 several threads. Public helpers convert coordinates and answer "is this point free?".</p>"""),
("private:", "Private parts",
 """<p><code>linear()</code> maps (x, y, z) to the flat array index. The flat
 <code>blocked_</code> array is the dense grid; <code>moves26_</code>/<code>moves6_</code> are the
 precomputed neighbour offsets with their costs.</p>"""),
]
