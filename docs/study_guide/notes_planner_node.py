FILE = "src/lrs_psenak_valko/src/map_planner_node.cpp"
TITLE = "Planner node (map_planner_node.cpp, A1.1)"
INTRO = """<p>The ROS 2 node around A*. At start-up it builds the occupancy map; afterwards it
answers every <code>plan_request</code> with <code>planned_path</code> +
<code>planning_status</code>. The executor is its only client in A1.2.</p>"""

NOTES = [
("// 3D A* planner node.", "Purpose and includes",
 """<p>PCL (Point Cloud Library) loads and down-samples the map; <code>pcl_conversions</code>
 converts PCL clouds to ROS <code>PointCloud2</code> for RViz.</p>"""),
("class MapPlannerNode final : public rclcpp::Node", "The node",
 """<p>Node name <code>map_planner_node</code> (the run script waits for it to appear).</p>"""),
("const std::string package_share = ament_index_cpp::get_package_share_directory", "Map path",
 """<p>A relative <code>map_path</code> is resolved inside the installed package
 (<code>install/.../share/lrs_psenak_valko</code>), so it works from any directory.</p>"""),
("const double leaf_size = declare_parameter<double>(\"voxel_size\", 0.15);", "Map parameters",
 """<p>Voxel size, safety radius, rack boxes (7 numbers each) and whether racks are required
 (true for the hangar, so a missing config cannot silently allow flying through shelves).</p>"""),
("astar_params_.allow_diagonal = declare_parameter", "A* parameters",
 """<p>Copied into the planner's <code>Params</code>.</p>"""),
("if (!std::isfinite(leaf_size) || leaf_size <= 0.0)", "Validation",
 """<p>Rejects invalid values: voxel size ≤ 0, negative radius, rack list not a multiple of 7,
 missing racks when required, heuristic weight &lt; 1 (would overestimate and could produce
 non-optimal and odd paths), negative limits.</p>"""),
("map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(", "Visualisation publishers and map loading",
 """<p>Map and inflated occupancy are published <b>transient local</b> (latched): RViz started
 later still receives them. Then <code>loadMap</code> builds everything and the rack outlines are
 published.</p>"""),
("path_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(\"planned_path\", 10);", "Answer topics and callback groups",
 """<p><code>planned_path</code> and <code>planning_status</code>. Two callback groups so a long
 search never blocks the occupancy query, run by a multi-threaded executor (see main).</p>"""),
("rclcpp::SubscriptionOptions planning_sub_options;", "Request subscriptions",
 """<p><code>plan_request</code> → <code>handleRequest</code>; <code>occupancy_query</code> →
 <code>handleOccupancyQuery</code> (a point in, "blocked?" out).</p>"""),
("RCLCPP_INFO(get_logger(), \"Ready: plan via", "Ready message", """<p>Logs the map statistics.</p>"""),
("void loadMap(const std::string& path, const std::string& occupancy_pcd_path,", "loadMap(): load and down-sample",
 """<p>Load the PCD (error if missing/empty), log its bounds, and down-sample with PCL
 <code>VoxelGrid</code>: one point per 0.15 m voxel.</p>"""),
("grid_ = std::make_unique<VoxelOccupancyGrid>(leaf_size);", "Build, fill racks, inflate",
 """<p>Voxel grid from the cloud → fill each rack box solid → inflate by the safety radius
 (order matters: racks must be filled before inflating).</p>"""),
("pcl::PointCloud<pcl::PointXYZ>::Ptr map_points", "Publish and save",
 """<p>Publish the map; save the inflated voxels to <code>map_voxels_with_racks_inflated.pcd</code>
 (the 3D viewer shows it with key <code>i</code>) and publish them.</p>"""),
("const VoxelOccupancyGrid::IndexBounds bounds = grid_->inflatedBounds();", "Dense grid for A*",
 """<p>The sparse grid is copied into a dense box covering all inflated voxels plus one voxel of
 padding, so the outside is never "free". Sizes are checked (max 400 M cells). The lambda converts
 A*'s local indices to the sparse grid's global indices.</p>"""),
("bool pathIsCollisionFree(const std::vector<astar::Vec3>& path) const", "Independent collision check",
 """<p>Before publishing, every point and every segment (sampled every quarter voxel) is checked
 against the inflated map once more. This is a second, independent safety check after A*.</p>"""),
("void publishRackBoxes(const std::vector<double>& boxes)", "Rack outlines for RViz",
 """<p>Computes the 8 corners of each rotated box (bit tricks: corner bit 0/1/2 = ±x/±y/±z) and
 draws its 12 edges as a LINE_LIST marker.</p>"""),
("void handleRequest(const geometry_msgs::msg::PoseArray::SharedPtr request)", "handleRequest(): one route",
 """<p>Exactly two poses required (start, goal). Run A*. On failure publish only the
 report.</p>"""),
("const bool collision_free = pathIsCollisionFree(result.path);", "Check, then publish",
 """<p>A colliding path is never published (the report says why). Otherwise the route goes out as
 a PoseArray, an RViz line marker, and the report with <code>[OK] TRASA NÁJDENÁ</code>.
 The executor needs both the route and the report.</p>"""),
("Human readable report (Slovak)", "Report helpers",
 """<p>Number/point formatting, distance, and <code>nearestFree</code> (closest free voxel
 within 3 m, used to suggest a fix).</p>"""),
("void describeBadEndpoint(std::ostringstream& o, const char* name, const astar::Vec3& p,", "describeBadEndpoint()",
 """<p>Explains why a start/goal is unusable: outside the map (which axis), inside an obstacle,
 or only inside the safety zone, with the nearest free point and how to fix it.</p>"""),
("std::string buildReport(const astar::Result& r, const astar::Vec3& s, const astar::Vec3& g,", "buildReport()",
 """<p>The full Slovak report: start, goal, result line (the executor searches for
 <code>[OK] TRASA NÁJDENÁ</code>), reason and tips on failure.</p>"""),
("// ---- success (path may still fail the independent collision check) ----", "Report on success",
 """<p>Points before/after simplification, lengths, time, snapping notes, and the point
 list with segment and cumulative lengths.</p>"""),
("void handleOccupancyQuery(const geometry_msgs::msg::PointStamped::SharedPtr query)", "Occupancy query and members",
 """<p>Answers "is this point blocked in the inflated map?". Then the member variables.</p>"""),
("int main(int argc, char** argv)", "main()",
 """<p>Same pattern as the executor: init, create the node, MultiThreadedExecutor, spin; errors
 are logged as FATAL with exit code 1.</p>"""),
]
