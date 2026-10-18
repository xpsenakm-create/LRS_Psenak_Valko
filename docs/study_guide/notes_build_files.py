FILE = "src/lrs_psenak_valko/CMakeLists.txt"
TITLE = "Build description (CMakeLists.txt)"
INTRO = """<p><code>colcon build</code> runs CMake with this file. It says what to compile, with
which libraries, and what to install.</p>"""

NOTES = [
("cmake_minimum_required(VERSION 3.8)", "Project and C++ standard",
 """<p>CMake version, project name (= ROS package name), C++17 required (needed for
 <code>std::filesystem</code>, structured bindings...).</p>"""),
("# A* je bez optimalizácie", "Release build by default",
 """<p>"A* is about 10× slower without optimisation": if no build type is given, build
 <code>Release</code> (-O3).</p>"""),
("find_package(ament_cmake REQUIRED)", "Dependencies",
 """<p>Every library the code uses: ROS 2 core (<code>rclcpp</code>), message packages, MAVROS
 messages, <code>nav_msgs</code> (flown path), PCL (point clouds).</p>"""),
("# --- A1.1: map + A* planner", "Planner executable",
 """<p>Built from the node and the A* file, with our <code>include/</code> directory and PCL.
 <code>ament_target_dependencies</code> adds the ROS packages' include paths and libraries.</p>"""),
("# --- A1.2: mission file model + executor", "Mission library and executor",
 """<p>The parser is a small static library (<code>mission_model</code>) that exports the
 <code>include/</code> directory; the executor links it and the ROS packages it uses.</p>"""),
("install(TARGETS map_planner_node mission_executor_node", "Install",
 """<p>Executables go to <code>lib/lrs_psenak_valko</code>, which is where <code>ros2 run</code>
 looks. Maps, config and missions go to <code>share/</code>. <code>ament_package()</code> generates
 the package metadata.</p>"""),
]
