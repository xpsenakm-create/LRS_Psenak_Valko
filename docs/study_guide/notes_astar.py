FILE = "src/lrs_psenak_valko/src/a_star_planner.cpp"
TITLE = "A* search (a_star_planner.cpp, A1.1)"
INTRO = """<p>The path search itself. It works on a dense 3D array of voxels in which every voxel
is "free" or "blocked" (already inflated by the safety radius, so the drone can be treated as a
point). It returns a list of 3D points from start to goal. It knows nothing about ROS, so it is
easy to test.</p>
<div class="box"><b>A* in one paragraph</b>
For every voxel n, <code>g(n)</code> = cheapest known cost from the start, <code>h(n)</code> =
an estimate of the remaining cost to the goal, <code>f = g + h</code>. Always expand the open
voxel with the smallest f. If h never overestimates (<b>admissible</b>) the first time the goal
is taken out of the queue its path is optimal; if h is also <b>consistent</b> (h(a) ≤ cost(a,b)
+ h(b)) no voxel ever needs to be expanded twice.</div>"""

NOTES = [
("// =============================================================================", "File summary",
 """<p>The header comment lists the design: inflated grid, octile heuristic (admissible and
 consistent), no corner squeezing, guaranteed termination, statistics.</p>"""),
("#include \"lrs_psenak_valko/a_star_planner.h\"", "Header and namespace",
 """<p>The class declaration and helper types (<code>Vec3</code>, <code>GridInfo</code>,
 <code>Params</code>, <code>Result</code>) are in the header. Everything is in
 <code>namespace astar</code>.</p>"""),
("const char* toString(Status s)", "Status names",
 """<p>Text for each result: SUCCESS, START/GOAL_OUT_OF_BOUNDS, START/GOAL_OCCUPIED, NO_PATH
 (provably unreachable), TIMEOUT (limit hit, a path may exist).</p>"""),
("AStarPlanner::AStarPlanner(", "Constructor: the 26 moves",
 """<p>Three nested loops over dx, dy, dz ∈ {−1, 0, 1} without (0, 0, 0) give the 26
 neighbours. The cost of a move is its Euclidean length: <code>resolution · √(number of non-zero
 axes)</code> = 1, √2 or √3 voxels. The 6 face neighbours are stored separately (for
 <code>allow_diagonal = false</code>). Then the occupancy is copied.</p>"""),
("void AStarPlanner::refresh()", "refresh(): dense copy of the occupancy",
 """<p>Asks the occupancy function once per voxel and stores 1/0 in a flat byte array. During the
 search a lookup is then just an array index, which is fast. Called once at construction.</p>"""),
("bool AStarPlanner::inBounds(int x, int y, int z) const", "Bounds and blocked checks",
 """<p><code>inBounds</code>: index inside the grid. <code>isBlocked</code>: outside counts as
 blocked, so the search can never leave the map. <code>linear(x, y, z) = (z·sy + y)·sx + x</code>
 is the position in the flat array.</p>"""),
("bool AStarPlanner::worldToIndex(", "World ↔ voxel index",
 """<p>index = <code>floor((p − origin) / resolution)</code>; returns false if outside.
 <code>indexToWorld</code> returns the voxel <b>centre</b> (+0.5).</p>"""),
("Vec3 AStarPlanner::indexToWorld(", "Voxel centre", """<p>See above.</p>"""),
("bool AStarPlanner::isBlockedWorld(", "Is a world point blocked?",
 """<p>Outside the grid = blocked. Used for segment checks.</p>"""),
("// Octile heuristic in 3D.", "The 3D octile heuristic",
 """<p>Sort the axis distances d1 ≤ d2 ≤ d3 (in voxels). In empty space the cheapest 26-connected
 path takes d1 steps diagonal in 3 axes (√3 each), then d2 − d1 steps diagonal in 2 axes
 (√2), then d3 − d2 straight steps (1):</p>
 <p><code>h = res · [√3·d1 + √2·(d2 − d1) + (d3 − d2)] = res · [(√3 − √2)·d1 + (√2 − 1)·d2 + d3]</code></p>
 <p>Obstacles can only make the real path longer, so h is <b>admissible</b>; it is also
 consistent, so the result is optimal on the grid. Euclidean distance would also be admissible
 but smaller (more expansions). Manhattan distance overestimates when diagonals are allowed (not
 admissible). With 6-connectivity, Manhattan is exact and is used.</p>"""),
("// A diagonal move is only legal", "No squeezing through corners",
 """<p>A diagonal move is allowed only if <b>every</b> voxel in the 2×2(×2) box it crosses is free.
 Otherwise the path could pass between two blocked voxels that touch only at an edge or corner, a
 gap no real drone fits through.</p>"""),
("bool AStarPlanner::findNearestFree(", "findNearestFree(): snapping",
 """<p>If start/goal is blocked: search a cube of ±radius voxels for the free voxel with the
 smallest squared distance. Used when <code>snap_radius_cells</code> &gt; 0 (5 in our config).</p>"""),
("// Sample the straight segment a->b", "segmentFree(): line of sight",
 """<p>Checks a straight segment by sampling it every quarter voxel. Thanks to the inflation, a
 tiny miss at a voxel corner is still inside the safety margin.</p>"""),
("// Greedy line-of-sight shortcutting", "simplify(): fewer, longer segments",
 """<p>From point i, take the <b>farthest</b> later point j that is still directly visible
 (segment free), add it, continue from j. The staircase of voxel steps becomes a few straight
 segments (e.g. 39 → 5 points), which the drone flies smoothly.</p>"""),
("Result AStarPlanner::plan(", "plan(): timing helpers",
 """<p>Start a timer; lambdas <code>elapsedMs</code> and <code>finish(status)</code> (sets status
 and planning time) keep the many return points short.</p>"""),
("// ---- 1. validate & snap start / goal", "Step 1: check and snap the endpoints",
 """<p>Convert start/goal to indices (outside the grid: error). If one is blocked: snap to the
 nearest free voxel if allowed, else START/GOAL_OCCUPIED. Remember whether snapping happened
 (reported, and the executor checks how far the goal moved).</p>"""),
("// ---- 2. A*", "Step 2: data structures",
 """<p>Per voxel: <code>g</code> (cost so far, ∞), <code>parent</code> (where we came from, −1),
 <code>closed</code> (already expanded): 4 + 4 + 1 bytes per voxel.</p>"""),
("struct Entry { float f, g; uint32_t idx; };", "The open list (priority queue)",
 """<p><code>std::priority_queue</code> is a max-heap, so the comparator is inverted: the entry
 with the <b>smallest f</b> comes first. On equal f the larger g wins (a node closer to the goal),
 which reduces expansions. The start is pushed with g = 0, f = w·h. <code>w</code> =
 <code>heuristic_weight</code> (1 = optimal A*, &gt; 1 = faster weighted A*).</p>"""),
("while (!open.empty())", "Main loop: take the best voxel",
 """<ul><li>Pop the smallest f. <b>Lazy deletion</b>: a voxel can be in the queue several times
 (with different f); if it is already closed, skip the stale copy.</li>
 <li>Goal popped: found (optimal because the heuristic is consistent).</li>
 <li>Every 1024 expansions check the time limit, and the optional expansion limit.</li>
 <li>Decode x, y, z from the flat index. Track the voxel closest to the goal (for the error report
 when no path exists).</li></ul>"""),
("for (const Move& m : moves)", "Relax the neighbours",
 """<p>For each move: skip blocked targets and corner-squeezing diagonals, skip closed voxels.
 New cost <code>ng = g + move cost</code>; if better than the known g, store it and the parent and
 push with <code>f = ng + w·h</code>. This is the standard A* "relaxation".</p>"""),
("if (std::isfinite(best_d2))", "No path: diagnostics",
 """<p>Store the closest point reached. Open list empty: NO_PATH (the goal is provably unreachable
 in this grid); limit hit: TIMEOUT.</p>"""),
("// ---- 3. reconstruct path", "Step 3: rebuild the path",
 """<p>Follow <code>parent</code> from the goal back to the start, converting to voxel centres,
 then reverse. Replace the first/last centre by the exact requested start/goal (unless snapped).
 Count points and length before simplification.</p>"""),
("res.path = params_.simplify_path ? simplify(raw) : raw;", "Simplify and measure",
 """<p>Line-of-sight shortcutting (if enabled), final length, SUCCESS. The planner node then
 checks the simplified path once more against the inflated map before publishing it.</p>"""),
("}  // namespace astar", "Notes for the documentation",
 """<p>The author's own summary: heuristic proof, why the corner check exists, complexity
 O(E log V) with a binary heap, memory per voxel, and why the search always terminates.
 Useful exam material.</p>"""),
]
