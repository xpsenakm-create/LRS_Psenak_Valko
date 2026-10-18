FILE = "git:origin/A1_3:src/lrs_psenak_valko/src/outdoor_math.h"
TITLE = "Outdoor mission maths (outdoor_math.h, A1.3)"
INTRO = """<p>All A1.3 formulas in one header without ROS: angle conventions and wrapping, GPS ↔
metres, the figure-8 and its speed, the soft start/stop, the square and the geofence radius. Because it
has no ROS dependency it is unit-tested with plain <code>g++</code> (next chapter).</p>"""

NOTES = [
("// outdoor_math.h -- A1.3 outdoor mission: all the maths, no ROS dependency.", "Conventions in the header",
 """<p><b>ENU yaw</b>: 0 = East, counter-clockwise positive (pose and setpoints). <b>Compass
 heading</b>: 0 = North, clockwise (<code>compass_hdg</code>). Relation: <code>yaw_ENU = 90° −
 heading</code>, wrapped to (−180°, 180°]. Frame: x East, y North, z up.</p>"""),
("namespace outdoor", "Constants and vectors",
 """<p>π, degree/radian factors, 2D/3D points and their distances.</p>"""),
("/// Wrap into (-pi, pi].", "Wrapping",
 """<p><code>remainder</code> gives [−π, π]; the −π end is moved to +π so the range is half-open and
 every angle has exactly one value. Degree versions to (−180, 180] and [0, 360).</p>"""),
("/// compass heading (0 = N, clockwise, deg) -> ENU yaw", "Compass ↔ ENU yaw",
 """<p>The two one-line conversions. North (0°) → yaw 90°, East (90°) → 0°, and back.</p>"""),
("/// Return the angle equivalent to `angle` (mod 2*pi) that lies closest to `reference`.", "unwrapNear()",
 """<p>Same as A1.2: keeps a sequence of yaw commands continuous (no ±2π jump).</p>"""),
("/// Rotate `current` towards `target` by at most `max_step` radians", "slewAngle()",
 """<p>Shortest signed difference, clamped to ±max_step, added to the (unwrapped) current angle: a
 rate-limited turn that never goes the long way.</p>"""),
("/// ENU yaw of the direction from `from` to `to`", "bearingYaw()",
 """<p><code>atan2(Δy, Δx)</code>: the ENU direction from one point to another (used for the POI and
 transit legs).</p>"""),
("// Geodetic <-> local ENU", "Local tangent plane: the idea",
 """<p>Near the origin the Earth is treated as flat: east = Δlon · (N + h)·cos(lat0), north = Δlat ·
 (M + h), with the WGS-84 radii N (prime vertical) and M (meridian) evaluated once at the origin.
 The error grows like d²/(2R): far below a millimetre over this mission.</p>"""),
("struct Geodetic", "LocalTangentPlane: constructor",
 """<p>WGS-84 constants (a = 6 378 137 m, f = 1/298.257…), e² = f(2 − f), then N = a/√(1 − e² sin² φ)
 and M = a(1 − e²)/(1 − e² sin² φ)^{3/2}. Stored as metres per radian of latitude and of
 longitude.</p>"""),
("Enu toEnu(const Geodetic& g) const", "toEnu() / toGeodetic()",
 """<p>Degrees difference → radians → metres (longitude difference wrapped, so crossing ±180° works),
 and the inverse. Altitude is just the difference.</p>"""),
("// Figure-8 (lemniscate of Gerono)", "Figure-8: formulas",
 """<p>x = A sin θ, y = A sin θ cos θ; derivatives A cos θ and A cos 2θ; tangent yaw =
 atan2(dy, dx); speed = A·θ̇·√(cos²θ + cos²2θ), maximum A·ω·√2 at the crossing (θ = 0, π).</p>"""),
("struct Figure8Sample { double x, y, dx, dy; };", "figure8(), yaw, speed",
 """<p>Position and derivative at θ; tangent yaw; speed; peak speed; and the largest ω for a speed
 limit, <code>v_max / (A√2)</code> = 0.354 rad/s for 1.5 m/s.</p>"""),
("/// Duration of a run from theta = 0 to th_end", "figure8Duration()",
 """<p>Each linear ramp covers only half of what full rate would, so start + stop ramps together cost
 exactly <code>ramp_s</code> extra seconds.</p>"""),
("/// Phase theta(s) after s seconds.", "figure8Phase(): soft start and soft stop",
 """<p>The rate dθ/dt rises linearly 0 → ω during the first ramp (θ = ω s²/(2·ramp)), stays at ω, and
 falls back to 0 so that θ reaches exactly 2π at the end. Why: the figure-8 starts and ends at the
 crossing, where it is fastest (1.27 m/s); a hard start would demand that speed from standstill.</p>"""),
("/// d(theta)/dt at time s", "figure8PhaseRate()",
 """<p>The analytic derivative of the phase, used for the velocity feed-forward
 (velocity = derivative of position × dθ/dt).</p>"""),
("// Waypoint square around the POI.", "squareCorners()",
 """<p>W1 = NW, W2 = NE, W3 = SE, W4 = SW around the POI (clockwise seen from above).</p>"""),
("/// Largest horizontal distance from the LAUNCH point", "missionMaxRadius(): geofence radius",
 """<p>The maximum distance from launch over the POI, the four corners and 720 samples of the
 figure-8. With the POI at launch: the corners dominate, 2.5·√2 = 3.54 m (limit 4 m).</p>"""),
]
