FILE = "src/lrs_psenak_valko/src/mission_model.cpp"
TITLE = "Mission file parser (mission_model.cpp)"
INTRO = """<p>Reads the mission CSV. It is strict on purpose: at the defence an unseen mission file
is given, and a typo must stop the program on the ground with a clear message, not cause a strange
flight.</p>"""

NOTES = [
("#include \"lrs_psenak_valko/mission_model.h\"", "Headers",
 """<p>Its own header, plus <code>cmath</code> (<code>isfinite</code>), <code>fstream</code>
 (read the file), <code>sstream</code> (split a line), <code>stdexcept</code>
 (<code>runtime_error</code>).</p>"""),
("namespace mission", "Anonymous namespace",
 """<p><code>namespace { }</code> without a name = the helpers inside are visible only in this
 file (internal linkage), like <code>static</code> functions in C.</p>"""),
("std::string trim(const std::string& value)", "trim()",
 """<p>Removes leading/trailing spaces, tabs and <code>\\r\\n</code> (a file saved on Windows ends
 lines with \\r). An all-space string becomes empty.</p>"""),
("double parseNumber(const std::string& text, size_t line, const char* field)", "parseNumber(): strict numbers",
 """<p><code>std::stod</code> converts text to a double and reports how many characters it used
 (<code>parsed</code>). The value is rejected if conversion fails, if not the <b>whole</b> field was a
 number (e.g. <code>1.5m</code>), or if it is not finite (<code>nan</code>, <code>inf</code>). The
 error names the line and the field.</p>"""),
("Mission loadCsv(const std::string& path)", "loadCsv(): open the file",
 """<p>Opens the file (error if missing) and prepares the result, the current line and the line
 counter.</p>"""),
("while (std::getline(input, row)) {", "Read line by line, split by commas",
 """<ul><li>Count lines (for error messages), trim, skip empty lines and <code>#</code>
 comments.</li>
 <li>Split on <code>,</code> and trim every field. Exactly 5 fields are required.</li>
 <li>The first data-like line that starts with <code>x,y,z</code> is a header: skip it once.</li></ul>"""),
("Waypoint waypoint;", "Fields → Waypoint",
 """<ul><li>x, y, z through <code>parseNumber</code>.</li>
 <li>precision must be exactly <code>soft</code> or <code>hard</code>.</li>
 <li>task: <code>-</code> or empty = none; <code>takeoff</code>, <code>land</code>,
 <code>landtakeoff</code>; anything starting with <code>yaw</code> is a yaw task, the rest of the
 text (<code>substr(3)</code>) is the angle, so <code>yaw-45</code> → −45°. Anything else
 throws.</li></ul>"""),
("if (mission.empty()) throw", "No waypoints = error",
 """<p>A file with only comments is an error. Otherwise the list is returned (moved, not
 copied).</p>"""),
("const char* toString(Precision precision)", "toString() helpers",
 """<p>Text names for log lines. A <code>switch</code> over all task types; the last
 <code>return</code> is never reached with valid values.</p>"""),
]
