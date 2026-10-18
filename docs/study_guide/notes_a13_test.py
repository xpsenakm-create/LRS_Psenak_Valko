FILE = "git:origin/A1_3:src/lrs_psenak_valko/src/outdoor_math_test.cpp"
TITLE = "Unit test of the maths (outdoor_math_test.cpp, A1.3)"
INTRO = """<p>A self-made test program for <code>outdoor_math.h</code>, built with plain g++ (no ROS,
no test framework). It prints <code>ALL TESTS PASSED</code> or every failed check with its line.</p>"""

NOTES = [
("// Build & run (no ROS needed):", "How to run it and the CHECK macros",
 """<p>Build command in the first lines. <code>CHECK(cond, ...)</code> counts and prints a failure
 (with <code>__LINE__</code> and the condition text <code>#cond</code>); <code>NEAR(a, b, tol)</code> is
 a CHECK for |a − b| ≤ tol. The <code>do { } while (0)</code> makes a macro behave like one
 statement.</p>"""),
("int main()", "Compass ↔ ENU yaw",
 """<p>The four compass directions map to the expected ENU yaw, the result is always in (−180, 180],
 and converting back gives the input (round trip every 7.5°).</p>"""),
("// ---- wrapping", "Wrapping edge cases", """<p>3π → π, −π → +π, −180° → 180°, −90° → 270°.</p>"""),
("// ---- geodetic <-> ENU", "GPS ↔ metres",
 """<p>A point 50 m east comes out ~50 m east and 0 north; local → geodetic → local returns the same
 point to 1e-9 m.</p>"""),
("// ---- figure-8", "Figure-8 geometry and speed",
 """<p>θ = 0 is the centre, tangent 45° there and 135° at θ = π, extent exactly 6 × 3 m, peak speed
 A√2 per unit ω, and the ω limit 0.354 rad/s for 1.5 m/s.</p>"""),
("// yaw is continuous after unwrapping", "Yaw continuity",
 """<p>Over a lap the unwrapped yaw never jumps more than 0.01 rad and ends where it started (no net
 winding). The raw atan2 does jump by ~2π, which proves unwrapping is needed.</p>"""),
("// ---- phase ramp", "Soft start / stop",
 """<p>θ and its rate are continuous where the ramps begin and end, the rate starts and ends at 0, and
 θ ends exactly at 2π after the predicted duration.</p>"""),
("// ---- square", "Square and bearings",
 """<p>Corner positions, the geofence radius 3.536 m, and bearing North = +90° ENU.</p>"""),
("// ---- slew", "Slew and result",
 """<p>From 3.0 rad towards −3.0 rad the short way is through +π (3.1, not 2.9). Exit code 0 only when
 every check passed.</p>"""),
]
