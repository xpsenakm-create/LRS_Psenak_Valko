FILE = "src/lrs_psenak_valko/include/lrs_psenak_valko/mission_model.h"
TITLE = "Mission data types (mission_model.h)"
INTRO = """<p>The header declares how one mission is represented in memory. It is deliberately
independent of ROS, so the parser could be unit-tested on its own.</p>"""

NOTES = [
("#pragma once", "Include guard and headers",
 """<p><code>#pragma once</code>: the compiler includes this file only once per translation unit,
 even if several headers include it. It needs <code>string</code> and <code>vector</code>.</p>"""),
("namespace mission", "Namespace and Precision",
 """<p>Everything lives in <code>namespace mission</code>, so names like <code>Task</code> cannot
 clash with other code. <code>Precision</code> has two values: <code>SOFT</code> (pass through)
 and <code>HARD</code> (stop and settle).</p>"""),
("enum class TaskType", "TaskType",
 """<p>The five task kinds of the mission file: <code>NONE</code> (<code>-</code>),
 <code>TAKEOFF</code>, <code>LAND</code>, <code>LAND_TAKEOFF</code>, <code>YAW</code>.</p>"""),
("struct Task", "Task and Waypoint",
 """<p>A <code>Task</code> is its type plus, for YAW, the angle in degrees. A <code>Waypoint</code>
 is x, y, z (metres, map frame), a precision and a task. The <code>= 0.0</code> initialisers give
 safe defaults.</p>"""),
("using Mission = std::vector<Waypoint>;", "Mission and the public functions",
 """<p>A mission is an ordered list of waypoints. <code>loadCsv</code> reads a file;
 <code>toString</code> turns precisions and tasks into text for log lines.</p>"""),
]
