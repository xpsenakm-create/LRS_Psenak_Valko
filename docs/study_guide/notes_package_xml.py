FILE = "src/lrs_psenak_valko/package.xml"
TITLE = "Package manifest (package.xml)"
INTRO = """<p>ROS 2's description of the package: name, version, and dependencies (used by
<code>rosdep</code> and colcon to order builds).</p>"""

NOTES = [
("<?xml version=\"1.0\"?>", "Identity",
 """<p>Format 3 manifest: name, version, description, maintainer, licence.</p>"""),
("<buildtool_depend>ament_cmake</buildtool_depend>", "Dependencies and build type",
 """<p><code>buildtool_depend</code>: built with ament_cmake. Each <code>depend</code> is needed to
 build and to run (mirrors CMakeLists). <code>build_type</code> tells colcon how to build it.</p>"""),
]
