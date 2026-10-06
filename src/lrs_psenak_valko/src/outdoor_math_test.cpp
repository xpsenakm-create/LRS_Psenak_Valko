// Build & run (no ROS needed):
//   g++ -std=c++17 -O2 -Wall -Wextra outdoor_math_test.cpp -o outdoor_math_test && ./outdoor_math_test
#include <cstdio>
#include <cstdlib>

#include "outdoor_math.h"

using namespace outdoor;

static int g_fail = 0;
#define CHECK(cond, ...)                                  \
  do {                                                    \
    if (!(cond)) {                                        \
      ++g_fail;                                           \
      std::printf("FAIL line %d: %s  ", __LINE__, #cond); \
      std::printf(__VA_ARGS__);                           \
      std::printf("\n");                                  \
    }                                                     \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol), "got %.9f expected %.9f", (double)(a), (double)(b))

int main()
{
  // ---- compass <-> ENU yaw -------------------------------------------------
  NEAR(headingDegToEnuYawDeg(0), 90, 1e-9);      // North  -> yaw +90 (ENU)
  NEAR(headingDegToEnuYawDeg(90), 0, 1e-9);      // East   -> yaw 0
  NEAR(headingDegToEnuYawDeg(180), -90, 1e-9);   // South  -> yaw -90
  NEAR(headingDegToEnuYawDeg(270), 180, 1e-9);   // West   -> +180 (range is (-180,180])
  NEAR(headingDegToEnuYawDeg(359), 91, 1e-9);
  for (double h = 0; h < 360; h += 7.5) {
    const double y = headingDegToEnuYawDeg(h);
    CHECK(y > -180.0 && y <= 180.0, "yaw %.3f out of range", y);
    NEAR(wrapDeg180(enuYawDegToHeadingDeg(y) - h), 0, 1e-9);   // round trip
  }

  // ---- wrapping ------------------------------------------------------------
  NEAR(wrapPi(3 * kPi), kPi, 1e-12);
  NEAR(wrapPi(-kPi), kPi, 1e-12);
  NEAR(wrapDeg180(-180), 180, 1e-12);
  NEAR(wrapDeg360(-90), 270, 1e-12);

  // ---- geodetic <-> ENU ----------------------------------------------------
  const LocalTangentPlane ltp(48.1486, 17.1077, 150.0);   // Bratislava-ish, only a test value
  const Enu east50 = ltp.toEnu({48.1486, 17.1077 + 50.0 / (111320.0 * std::cos(48.1486 * kDeg2Rad)), 150.0});
  NEAR(east50.e, 50.0, 0.1);                               // ~50 m east (spherical estimate)
  NEAR(east50.n, 0.0, 1e-6);
  const Enu p{12.3, -45.6, 4.2};
  const Enu q = ltp.toEnu(ltp.toGeodetic(p));
  NEAR(q.e, p.e, 1e-9); NEAR(q.n, p.n, 1e-9); NEAR(q.u, p.u, 1e-9);

  // ---- figure-8 ------------------------------------------------------------
  const double A = 3.0;
  const Figure8Sample c0 = figure8(A, 0.0);
  NEAR(c0.x, 0, 1e-12); NEAR(c0.y, 0, 1e-12);              // t = 0 is the centre (POI)
  NEAR(figure8Yaw(0.0) * kRad2Deg, 45.0, 1e-9);            // first pass: heading NE
  NEAR(figure8Yaw(kPi) * kRad2Deg, 135.0, 1e-9);           // second pass through the crossing: NW
  // extent 6 x 3 m
  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, vmax = 0;
  for (int i = 0; i <= 100000; ++i) {
    const double th = 2 * kPi * i / 100000.0;
    const Figure8Sample s = figure8(A, th);
    xmin = std::min(xmin, s.x); xmax = std::max(xmax, s.x);
    ymin = std::min(ymin, s.y); ymax = std::max(ymax, s.y);
    vmax = std::max(vmax, figure8Speed(A, 1.0, th));
  }
  NEAR(xmax - xmin, 6.0, 1e-6);
  NEAR(ymax - ymin, 3.0, 1e-6);                            // y in [-A/2, +A/2]
  NEAR(vmax, A * std::sqrt(2.0), 1e-6);                    // peak speed per omega = A*sqrt(2)
  NEAR(figure8PeakSpeed(A, 0.35), 1.4849, 1e-3);
  NEAR(figure8MaxOmega(A, 1.5), 0.3536, 1e-3);

  // yaw is continuous after unwrapping: no 2*pi jumps, and no net winding over a lap
  double yaw = figure8Yaw(0.0), max_step = 0.0, yaw_min = yaw, yaw_max = yaw;
  const int N = 20000;
  for (int i = 1; i <= N; ++i) {
    const double th = 2 * kPi * i / N;
    const double y_new = unwrapNear(yaw, figure8Yaw(th));
    max_step = std::max(max_step, std::fabs(y_new - yaw));
    yaw = y_new; yaw_min = std::min(yaw_min, yaw); yaw_max = std::max(yaw_max, yaw);
  }
  CHECK(max_step < 0.01, "max yaw step %.5f rad", max_step);
  NEAR(yaw, figure8Yaw(0.0), 1e-9);                        // back at the start value: net winding 0
  std::printf("figure-8 yaw range over a lap: %.1f .. %.1f deg, max step %.4f rad (N=%d)\n",
              yaw_min * kRad2Deg, yaw_max * kRad2Deg, max_step, N);
  // The RAW atan2 DOES jump by ~2*pi somewhere -> unwrap is really needed:
  double raw_max_jump = 0.0, prev = figure8Yaw(0.0);
  for (int i = 1; i <= N; ++i) {
    const double y_new = figure8Yaw(2 * kPi * i / N);
    raw_max_jump = std::max(raw_max_jump, std::fabs(y_new - prev)); prev = y_new;
  }
  std::printf("raw atan2 biggest jump: %.3f rad\n", raw_max_jump);

  // ---- phase ramp ----------------------------------------------------------
  NEAR(figure8Phase(0.3, 3.0, 0.0), 0.0, 1e-12);
  NEAR(figure8Phase(0.3, 3.0, 3.0), 0.3 * 1.5, 1e-12);                       // continuous at s = ramp
  NEAR(figure8Phase(0.3, 3.0, 10.0), 0.3 * (10.0 - 1.5), 1e-12);
  NEAR(figure8Phase(0.3, 0.0, 10.0), 3.0, 1e-12);                            // ramp disabled
  const double h = 1e-6;                                                     // rate is continuous
  NEAR((figure8Phase(0.3, 3.0, 3.0 + h) - figure8Phase(0.3, 3.0, 3.0 - h)) / (2 * h), 0.3, 1e-6);
  NEAR((figure8Phase(0.3, 3.0, h) - figure8Phase(0.3, 3.0, 0.0)) / h, 0.0, 1e-5);   // starts at rate 0

  // ---- square --------------------------------------------------------------
  const auto sq = squareCorners({0, 0}, 5.0);
  NEAR(sq[0].x, -2.5, 1e-12); NEAR(sq[0].y, 2.5, 1e-12);   // W1 = NW
  NEAR(sq[2].x, 2.5, 1e-12);  NEAR(sq[2].y, -2.5, 1e-12);  // W3 = SE
  NEAR(missionMaxRadius({0, 0}, 5.0, 3.0), 2.5 * std::sqrt(2.0), 1e-6);      // corners dominate: 3.536 m
  NEAR(bearingYaw({0, 0}, {0, 1}) * kRad2Deg, 90.0, 1e-9);                   // North = +90 ENU
  NEAR(bearingYaw({0, 0}, {1, 0}) * kRad2Deg, 0.0, 1e-9);

  // ---- slew ----------------------------------------------------------------
  NEAR(slewAngle(3.0, -3.0, 0.1), 3.1, 1e-12);             // goes the SHORT way (through +pi), stays unwrapped

  if (g_fail == 0) std::printf("ALL TESTS PASSED\n");
  else std::printf("%d CHECK(S) FAILED\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
