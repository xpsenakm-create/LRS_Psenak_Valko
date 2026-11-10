// Build & run (no ROS needed):
//   g++ -std=c++17 -O2 -Wall -Wextra -I/usr/include/eigen3 ekf8_test.cpp -o ekf8_test && ./ekf8_test
// (colcon also builds it: ros2 run lrs_psenak_valko ekf8_test)
#include <cstdio>
#include <cstdlib>
#include <random>

#include "ekf8.h"

using namespace loc;

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

static Vec8 std0()
{
  Vec8 s;
  s << 0.6, 0.6, 0.9, 0.5, 0.5, 0.5, 0.1, 0.05;
  return s;
}

static bool covarianceHealthy(const Mat8& P)
{
  if (!P.allFinite()) return false;
  if ((P - P.transpose()).cwiseAbs().maxCoeff() > 1e-12) return false;
  return Eigen::LLT<Mat8>(P).info() == Eigen::Success;      // positive definite
}

int main()
{
  // ---- yaw wrapping: [-pi, pi) ----------------------------------------------------------
  NEAR(wrapAngle(0.5), 0.5, 1e-12);
  NEAR(wrapAngle(kPi), -kPi, 1e-12);            // +pi belongs to the other end of the interval
  NEAR(wrapAngle(-kPi), -kPi, 1e-12);
  NEAR(wrapAngle(3.0 * kPi), -kPi, 1e-9);
  NEAR(wrapAngle(2.0 * kPi + 0.25), 0.25, 1e-12);
  NEAR(wrapAngle(-kPi - 0.1), kPi - 0.1, 1e-12);
  for (double a = -20.0; a < 20.0; a += 0.37) {
    const double w = wrapAngle(a);
    CHECK(w >= -kPi && w < kPi, "wrap(%f) = %f", a, w);
    NEAR(std::remainder(w - a, 2.0 * kPi), 0.0, 1e-9);
  }

  // ---- prediction: constant velocity and yaw = integral of (w - b) -------------------------
  {
    Ekf8 f;
    f.initialize(10.0, {1.0, 2.0, 0.5}, 0.3, std0());
    Vec8 x = f.x();
    x(VX) = 0.5; x(VY) = -0.2; x(VZ) = 0.1; x(BG) = 0.02;
    f.setState(x, f.P());
    f.imu(10.0, 0.12);                                              // first sample: only held
    for (int i = 1; i <= 200; ++i) f.imu(10.0 + 0.01 * i, 0.12);   // 2 s of 100 Hz gyro
    NEAR(f.time(), 12.0, 1e-9);
    NEAR(f.x()(X), 1.0 + 0.5 * 2.0, 1e-9);
    NEAR(f.x()(Y), 2.0 - 0.2 * 2.0, 1e-9);
    NEAR(f.x()(Z), 0.5 + 0.1 * 2.0, 1e-9);
    NEAR(f.x()(YAW), 0.3 + (0.12 - 0.02) * 2.0, 1e-9);
    CHECK(f.P()(X, X) > 0.36 && f.P()(YAW, YAW) > 0.01, "covariance must grow without measurements");
    CHECK(covarianceHealthy(f.P()), "P after prediction");
  }

  // ---- yaw wraps across +-pi during prediction --------------------------------------------
  {
    Ekf8 f;
    f.initialize(0.0, Vec3::Zero(), kPi - 0.05, std0());
    f.imu(0.0, 0.5);
    f.imu(1.0, 0.5);                        // +0.5 rad over 1 s
    NEAR(f.x()(YAW), -kPi + 0.45, 1e-9);
    // and a yaw measurement on the other side of the cut gives a SMALL innovation
    const UpdateResult r = f.updateYaw(1.0, kPi - 0.15, 0.01, nullptr);
    NEAR(r.innovation[0], -0.6, 1e-9);   // (pi-0.15) - (-pi+0.45) = 2pi-0.6, wrapped -0.6
    CHECK(f.x()(YAW) >= -kPi && f.x()(YAW) < kPi, "corrected yaw is wrapped");
  }

  // ---- body-velocity Jacobian equals the numerical derivative -----------------------------
  {
    const double yaw = 0.7;
    const Vec3 v(0.4, -0.3, 0.2);
    const double c = std::cos(yaw), s = std::sin(yaw), eps = 1e-6;
    const Vec3 dh_dyaw = (worldToBody(v, yaw + eps) - worldToBody(v, yaw - eps)) / (2.0 * eps);
    NEAR(dh_dyaw.x(), -s * v.x() + c * v.y(), 1e-8);
    NEAR(dh_dyaw.y(), -c * v.x() - s * v.y(), 1e-8);
    NEAR(dh_dyaw.z(), 0.0, 1e-12);
    const Vec3 back = bodyToWorld(worldToBody(v, yaw), yaw);
    NEAR((back - v).norm(), 0.0, 1e-12);
  }

  // ---- stationary drone: gyro bias becomes observable through VO yaw ----------------------
  // Truth: at rest at (-7, 13, 0), yaw pi/2, gyro bias +0.02 rad/s.  GPS 5 Hz, VO 30 Hz, IMU 100 Hz.
  {
    std::mt19937 rng(7);
    std::normal_distribution<double> n01(0.0, 1.0);
    SensorFusion fusion;
    const Vec3 p_true(-7.0, 13.0, 0.0);
    const double yaw_true = kPi / 2.0, bias = 0.02;
    UpdateResult ry, rv, rg;
    for (int k = 0; k <= 3000; ++k) {        // 30 s at 100 Hz
      const double t = 100.0 + 0.01 * k;
      fusion.imu(t, bias + 0.004 * n01(rng));
      if (k % 20 == 0) {
        const Vec3 z = p_true + Vec3(0.6 * n01(rng), 0.6 * n01(rng), 0.9 * n01(rng));
        fusion.gps(t, z, nullptr, &rg);
      }
      if (k % 3 == 0)
        fusion.vo(t, yaw_true + 0.005 * n01(rng), Vec3(0.03 * n01(rng), 0.03 * n01(rng), 0.03 * n01(rng)), &ry, &rv);
    }
    CHECK(fusion.ready(), "filter initialised");
    const Vec8& x = fusion.ekf().x();
    NEAR(x(BG), bias, 0.003);
    NEAR(x(YAW), yaw_true, 0.02);
    CHECK((x.segment<3>(X) - p_true).head<2>().norm() < 0.3, "xy error %.3f", (x.segment<3>(X) - p_true).head<2>().norm());
    CHECK(std::sqrt(fusion.ekf().P()(BG, BG)) < 0.003, "bias std %.4f", std::sqrt(fusion.ekf().P()(BG, BG)));
    CHECK(covarianceHealthy(fusion.ekf().P()), "P after 30 s of updates");
  }

  // ---- innovation gate: a 5 m GPS outlier is rejected for its d2, normal fixes are not ------
  {
    SensorFusion fusion;
    UpdateResult ry, rv, rg;
    fusion.gps(0.0, Vec3(0, 0, 0), nullptr, &rg);
    fusion.vo(0.0, 0.0, Vec3::Zero(), &ry, &rv);
    std::mt19937 rng(3);
    std::normal_distribution<double> n01(0.0, 1.0);
    int rejected = 0;
    for (int k = 1; k <= 500; ++k) {          // 100 s of GPS, the drone at rest
      const double t = 0.2 * k;
      fusion.imu(t, 0.0);
      fusion.vo(t, 0.0, Vec3::Zero(), &ry, &rv);
      fusion.gps(t, Vec3(0.6 * n01(rng), 0.6 * n01(rng), 0.9 * n01(rng)), nullptr, &rg);
      if (rg.status == UpdateResult::Status::REJECTED_GATE) ++rejected;
    }
    CHECK(rejected <= 3, "consistent GPS rejected %d times out of 500", rejected);
    // worst case for the gate: the whole 5 m along z (largest sigma)
    fusion.gps(100.2, Vec3(0.0, 0.0, 5.0), nullptr, &rg);
    CHECK(rg.status == UpdateResult::Status::REJECTED_GATE, "outlier d2 = %.1f", rg.d2);
    CHECK(rg.d2 > 16.27, "d2 = %.1f", rg.d2);
    CHECK(std::fabs(fusion.ekf().x()(Z)) < 0.5, "rejected outlier must not move the state");
    // lock-out guard: 5 rejections in a row, then the 6th sample is accepted (forced)
    for (int k = 1; k <= 4; ++k) fusion.gps(100.2 + 0.2 * k, Vec3(0.0, 0.0, 5.0), nullptr, &rg);
    CHECK(rg.status == UpdateResult::Status::REJECTED_GATE, "5th rejection");
    fusion.gps(101.4, Vec3(0.0, 0.0, 5.0), nullptr, &rg);
    CHECK(rg.status == UpdateResult::Status::ACCEPTED_FORCED, "6th accepted by the guard");
  }

  // ---- timestamps: duplicate, slightly late, too old ---------------------------------------
  {
    SensorFusion fusion;
    UpdateResult ry, rv, rg;
    fusion.gps(4.0, Vec3::Zero(), nullptr, &rg);
    CHECK(fusion.vo(5.0, 0.0, Vec3::Zero(), &ry, &rv) == SensorFusion::Input::INITIALISED, "init");
    NEAR(fusion.ekf().time(), 5.0, 1e-12);       // initialised at the later of the two stamps
    CHECK(fusion.imu(5.01, 0.0) == SensorFusion::Input::USED, "imu");
    CHECK(fusion.imu(5.01, 0.0) == SensorFusion::Input::DUPLICATE, "duplicate stamp");
    CHECK(fusion.imu(5.005, 0.0) == SensorFusion::Input::DUPLICATE, "out of order within the sensor");
    CHECK(fusion.gps(3.9, Vec3::Zero(), nullptr, &rg) == SensorFusion::Input::DUPLICATE, "older than the last GPS");
    // newer than the last GPS but 30 ms older than the state (arrived after a later IMU sample)
    CHECK(fusion.gps(4.98, Vec3::Zero(), nullptr, &rg) == SensorFusion::Input::USED, "30 ms late GPS is used");
    CHECK(rg.late && rg.accepted(), "flagged late and applied");
    NEAR(fusion.ekf().time(), 5.01, 1e-12);      // the state never goes back in time
    fusion.imu(6.0, 0.0);
    CHECK(fusion.vo(5.5, 0.0, Vec3::Zero(), &ry, &rv) == SensorFusion::Input::TOO_OLD, "0.5 s old VO");
    CHECK(ry.status == UpdateResult::Status::REJECTED_OLD, "VO yaw not applied");
  }

  // ---- 15 s GPS outage while flying: finite, bounded drift, smooth recovery ----------------
  {
    std::mt19937 rng(11);
    std::normal_distribution<double> n01(0.0, 1.0);
    SensorFusion fusion;
    UpdateResult ry, rv, rg;
    const Vec3 v_true(0.0, -0.5, 0.0);            // flying south at 0.5 m/s, nose north-west
    const double yaw_true = 2.0;
    const Vec3 vo_bias(0.01, -0.01, 0.005);       // correlated VO error the filter does not model
    double max_err = 0.0, max_jump = 0.0;
    Vec3 prev = Vec3::Zero();
    for (int k = 0; k <= 6000; ++k) {             // 60 s; GPS off between 20 and 35 s
      const double t = 0.01 * k;
      const Vec3 p_true = Vec3(1.0, 2.0, 2.0) + v_true * t;
      fusion.imu(t, -0.015 + 0.004 * n01(rng));
      if (k % 3 == 0)
        fusion.vo(t, yaw_true, worldToBody(v_true, yaw_true) + vo_bias + 0.03 * Vec3(n01(rng), n01(rng), n01(rng)), &ry, &rv);
      if (k % 20 == 0 && (t < 20.0 || t >= 35.0))
        fusion.gps(t, p_true + Vec3(0.6 * n01(rng), 0.6 * n01(rng), 0.9 * n01(rng)), nullptr, &rg);
      if (!fusion.ready()) continue;
      const Vec3 p = fusion.ekf().x().segment<3>(X);
      if (t > 5.0) {
        max_err = std::max(max_err, (p - p_true).head<2>().norm());
        max_jump = std::max(max_jump, (p - prev).norm() - 0.5 * 0.01);
      }
      prev = p;
      CHECK(fusion.ekf().x().allFinite() && covarianceHealthy(fusion.ekf().P()), "t=%.2f", t);
      if (!fusion.ekf().x().allFinite()) break;
    }
    CHECK(max_err < 1.0, "max xy error %.3f m", max_err);
    CHECK(max_jump < 0.25, "largest single-step change %.3f m", max_jump);
  }

  if (g_fail == 0) std::printf("ekf8_test: all checks passed\n");
  return g_fail == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
