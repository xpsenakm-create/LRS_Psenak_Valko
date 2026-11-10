// ekf8.h -- Assignment 2: simplified eight-state EKF for drone localisation.  No ROS dependency,
// so exactly the same code runs in the ROS node (localization_estimator_node.cpp), in the offline
// replay (ekf8_replay.cpp) and in the unit test (ekf8_test.cpp).
//
// STATE (order fixed by the assignment), frame `map` = MAVROS-compatible ENU:
//   x = [x, y, z, vx, vy, vz, yaw, b_g]^T
//   position [m], ENU velocity [m/s], ENU yaw [rad] wrapped to [-pi, pi), gyro Z bias [rad/s]
//
// PREDICTION (on every IMU sample, dt from the header stamps):
//   p   <- p + v dt                   constant velocity
//   v   <- v                          + white acceleration noise
//   yaw <- wrap(yaw + (w_z - b_g) dt) measured gyro rate minus the estimated bias
//   b_g <- b_g                        + random walk
//   P   <- F P F^T + Q(dt)            F is exact: the model is linear in the state
//
// MEASUREMENTS (asynchronous, each one first brings the state to its own stamp):
//   GPS position      z = p                    H = [I3 0 0 0]                 linear
//   VO yaw            z = yaw                  H = e_yaw^T, innovation wrapped
//   VO body velocity  z = Rz(yaw)^T v          H = dh/d(v, yaw)                EKF Jacobian
//                     (or, as an option, the linear-KF approximation: z' = Rz(yaw_est) z_body
//                      compared with v, H = [0 I3 0 0])
//   gyro at rest      z = w_z                  H = e_bg^T                       static alignment:
//                     only from initialisation until the first motion (true yaw rate = 0)
//
// Every update: r = z - h(x), S = H P H^T + R, d2 = r^T S^-1 r (the gate statistic),
// K = P H^T S^-1 (solved with LDLT, never inverted), Joseph-form covariance update.
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace loc
{

constexpr double kPi = 3.14159265358979323846;

// State indices, in the order required by the assignment.
enum Index : int { X = 0, Y, Z, VX, VY, VZ, YAW, BG, N };

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec8 = Eigen::Matrix<double, N, 1>;
using Mat8 = Eigen::Matrix<double, N, N>;

/// Wrap into [-pi, pi).  Used for the yaw state AND for every yaw innovation.
inline double wrapAngle(double a)
{
  return a - 2.0 * kPi * std::floor((a + kPi) / (2.0 * kPi));
}

/// Yaw-only rotation of an ENU vector into the body frame (what VO reports).
inline Vec3 worldToBody(const Vec3& v, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * v.x() + s * v.y(), -s * v.x() + c * v.y(), v.z()};
}

inline Vec3 bodyToWorld(const Vec3& v, double yaw)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z()};
}

/// Continuous-time process noise densities.
struct ProcessNoise
{
  double accel_xy = 0.2;       // [m/s^2/sqrt(Hz)] white acceleration: how wrong "constant velocity" is
  double accel_z = 0.3;        // [m/s^2/sqrt(Hz)]
  double yaw_rate = 0.005;     // [rad/s/sqrt(Hz)] gyro white noise (+ tilt coupling) integrated into yaw
  double gyro_bias_rw = 2e-5;  // [rad/s/sqrt(s)]  gyro bias random walk
};

/// Chi-square innovation gate with a lock-out guard.
///
/// d2 = r^T S^-1 r follows a chi-square distribution with dim(z) degrees of freedom when the
/// filter is consistent, so the threshold is a quantile of that distribution (16.27 = 99.9 % for
/// 3 DOF).  If the ESTIMATE itself has gone wrong every sample would be rejected for ever, so
/// after `max_consecutive` rejections in a row the next sample is accepted again (0 = never).
struct Gate
{
  enum class Verdict { ACCEPT, REJECT, ACCEPT_FORCED };

  double threshold = 0.0;  // <= 0 disables the gate
  int max_consecutive = 0;
  int rejected_in_row = 0;

  Verdict check(double d2)
  {
    if (threshold <= 0.0 || d2 <= threshold) { rejected_in_row = 0; return Verdict::ACCEPT; }
    if (max_consecutive > 0 && rejected_in_row >= max_consecutive) {
      rejected_in_row = 0;
      return Verdict::ACCEPT_FORCED;
    }
    ++rejected_in_row;
    return Verdict::REJECT;
  }
};

/// What happened to one measurement (returned to the caller for logging / plotting).
struct UpdateResult
{
  enum class Status { ACCEPTED, ACCEPTED_FORCED, REJECTED_GATE, REJECTED_OLD, REJECTED_NUMERIC };

  Status status = Status::REJECTED_NUMERIC;
  bool late = false;   // stamp slightly older than the state; applied at the state time
  double d2 = std::numeric_limits<double>::quiet_NaN();
  int dof = 0;
  std::array<double, 3> innovation{{0.0, 0.0, 0.0}};

  bool accepted() const { return status == Status::ACCEPTED || status == Status::ACCEPTED_FORCED; }
};

class Ekf8
{
public:
  /// How a stamp relates to the current state time.
  enum class TimeStatus { ON_TIME, LATE, TOO_OLD };

  Ekf8() = default;

  void setProcessNoise(const ProcessNoise& q) { q_ = q; }
  /// Measurements at most this much older than the state are applied at the state time
  /// (error ~ v * lag, i.e. millimetres); older ones are rejected.
  void setMaxLag(double seconds) { max_lag_ = seconds; }

  void initialize(double t, const Vec3& position, double yaw, const Vec8& std0)
  {
    x_.setZero();
    x_.segment<3>(X) = position;
    x_(YAW) = wrapAngle(yaw);
    P_ = std0.cwiseProduct(std0).asDiagonal();
    t_ = t;
    rate_hold_ = x_(BG);   // until the first gyro sample: no net rotation
    initialized_ = true;
  }

  /// Direct state access for tests and tools (the filter itself never needs it).
  void setState(const Vec8& x, const Mat8& P)
  {
    x_ = x;
    x_(YAW) = wrapAngle(x_(YAW));
    P_ = P;
  }

  void reset() { initialized_ = false; }
  bool initialized() const { return initialized_; }
  double time() const { return t_; }
  const Vec8& x() const { return x_; }
  const Mat8& P() const { return P_; }
  double rateHold() const { return rate_hold_; }

  /// Gyro sample: propagate to its stamp with the previously held rate (zero-order hold), then
  /// hold the new rate.  A slightly late sample only replaces the held rate.
  TimeStatus imu(double t, double w_z)
  {
    const TimeStatus ts = alignTo(t);
    if (ts != TimeStatus::TOO_OLD) rate_hold_ = w_z;
    return ts;
  }

  /// Bring the state to time t (prediction with the held gyro rate).
  TimeStatus alignTo(double t)
  {
    if (t > t_) {
      predict(t - t_, rate_hold_);
      t_ = t;
      return TimeStatus::ON_TIME;
    }
    if (t == t_) return TimeStatus::ON_TIME;
    return (t_ - t <= max_lag_) ? TimeStatus::LATE : TimeStatus::TOO_OLD;
  }

  void predict(double dt, double w_z)
  {
    if (!(dt > 0.0)) return;
    x_.segment<3>(X) += dt * x_.segment<3>(VX);
    x_(YAW) = wrapAngle(x_(YAW) + (w_z - x_(BG)) * dt);

    Mat8 F = Mat8::Identity();
    F(X, VX) = F(Y, VY) = F(Z, VZ) = dt;
    F(YAW, BG) = -dt;

    // Discretised white-noise models: position/velocity per axis (constant velocity), and
    // yaw/bias (yaw' = -b + noise, b' = noise), so the bias random walk also leaks into yaw.
    Mat8 Q = Mat8::Zero();
    const double dt2 = dt * dt, dt3 = dt2 * dt;
    auto cv = [&](int p, int v, double sigma) {
      const double q = sigma * sigma;
      Q(p, p) = q * dt3 / 3.0;
      Q(p, v) = Q(v, p) = q * dt2 / 2.0;
      Q(v, v) = q * dt;
    };
    cv(X, VX, q_.accel_xy);
    cv(Y, VY, q_.accel_xy);
    cv(Z, VZ, q_.accel_z);
    const double qy = q_.yaw_rate * q_.yaw_rate, qb = q_.gyro_bias_rw * q_.gyro_bias_rw;
    Q(YAW, YAW) = qy * dt + qb * dt3 / 3.0;
    Q(YAW, BG) = Q(BG, YAW) = -qb * dt2 / 2.0;
    Q(BG, BG) = qb * dt;

    P_ = F * P_ * F.transpose() + Q;
    P_ = 0.5 * (P_ + P_.transpose());
  }

  // ------------------------------------------------------------ measurement models
  UpdateResult updateGps(double t, const Vec3& z, const Mat3& R, Gate* gate)
  {
    UpdateResult res;
    if (!prepare(t, res)) return res;
    Eigen::Matrix<double, 3, N> H = Eigen::Matrix<double, 3, N>::Zero();
    H.block<3, 3>(0, X).setIdentity();
    const Vec3 r = z - x_.segment<3>(X);
    return update<3>(r, H, R, gate, res);
  }

  UpdateResult updateYaw(double t, double yaw, double variance, Gate* gate)
  {
    UpdateResult res;
    if (!prepare(t, res)) return res;
    Eigen::Matrix<double, 1, N> H = Eigen::Matrix<double, 1, N>::Zero();
    H(0, YAW) = 1.0;
    Eigen::Matrix<double, 1, 1> r, R;
    r(0) = wrapAngle(yaw - x_(YAW));
    R(0, 0) = variance;
    return update<1>(r, H, R, gate, res);
  }

  /// Zero angular rate update (static alignment): while the drone is known to stand still the
  /// true yaw rate is 0, so the raw gyro sample measures the bias directly: z = w_z, h(x) = b_g.
  UpdateResult updateGyroBias(double t, double w_z, double variance, Gate* gate)
  {
    UpdateResult res;
    if (!prepare(t, res)) return res;
    Eigen::Matrix<double, 1, N> H = Eigen::Matrix<double, 1, N>::Zero();
    H(0, BG) = 1.0;
    Eigen::Matrix<double, 1, 1> r, R;
    r(0) = w_z - x_(BG);
    R(0, 0) = variance;
    return update<1>(r, H, R, gate, res);
  }

  /// VO linear twist, expressed in base_link.  ekf = true: genuine nonlinear model
  /// h(x) = Rz(yaw)^T v with its Jacobian.  ekf = false: linear-KF approximation, the
  /// measurement is rotated into ENU with the CURRENT yaw estimate (yaw uncertainty ignored).
  UpdateResult updateBodyVelocity(double t, const Vec3& v_body, const Mat3& R, bool ekf, Gate* gate)
  {
    UpdateResult res;
    if (!prepare(t, res)) return res;
    const double yaw = x_(YAW), c = std::cos(yaw), s = std::sin(yaw);
    const Vec3 v = x_.segment<3>(VX);
    Eigen::Matrix<double, 3, N> H = Eigen::Matrix<double, 3, N>::Zero();
    if (ekf) {
      const Vec3 h = worldToBody(v, yaw);
      H(0, VX) = c;  H(0, VY) = s;  H(0, YAW) = -s * v.x() + c * v.y();
      H(1, VX) = -s; H(1, VY) = c;  H(1, YAW) = -c * v.x() - s * v.y();
      H(2, VZ) = 1.0;
      return update<3>(v_body - h, H, R, gate, res);
    }
    Mat3 Rz;
    Rz << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    H.block<3, 3>(0, VX).setIdentity();
    return update<3>(Rz * v_body - v, H, Rz * R * Rz.transpose(), gate, res);
  }

private:
  bool prepare(double t, UpdateResult& res)
  {
    const TimeStatus ts = alignTo(t);
    if (ts == TimeStatus::TOO_OLD) { res.status = UpdateResult::Status::REJECTED_OLD; return false; }
    res.late = (ts == TimeStatus::LATE);
    return true;
  }

  template <int M>
  UpdateResult update(const Eigen::Matrix<double, M, 1>& r, const Eigen::Matrix<double, M, N>& H,
                      const Eigen::Matrix<double, M, M>& R, Gate* gate, UpdateResult res)
  {
    using MatM = Eigen::Matrix<double, M, M>;
    res.dof = M;
    for (int i = 0; i < M && i < 3; ++i) res.innovation[i] = r(i);

    const MatM S = H * P_ * H.transpose() + R;
    const Eigen::LDLT<MatM> ldlt(S);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
      res.status = UpdateResult::Status::REJECTED_NUMERIC;
      return res;
    }
    res.d2 = r.dot(ldlt.solve(r));
    if (!std::isfinite(res.d2)) { res.status = UpdateResult::Status::REJECTED_NUMERIC; return res; }

    res.status = UpdateResult::Status::ACCEPTED;
    if (gate) {
      const Gate::Verdict v = gate->check(res.d2);
      if (v == Gate::Verdict::REJECT) { res.status = UpdateResult::Status::REJECTED_GATE; return res; }
      if (v == Gate::Verdict::ACCEPT_FORCED) res.status = UpdateResult::Status::ACCEPTED_FORCED;
    }

    // K = P H^T S^-1, computed as K^T = S^-1 (H P) because P and S are symmetric.
    const Eigen::Matrix<double, N, M> K = ldlt.solve(H * P_).transpose();
    Vec8 x_new = x_ + K * r;
    x_new(YAW) = wrapAngle(x_new(YAW));
    const Mat8 IKH = Mat8::Identity() - K * H;
    Mat8 P_new = IKH * P_ * IKH.transpose() + K * R * K.transpose();   // Joseph form
    P_new = 0.5 * (P_new + P_new.transpose());
    if (!x_new.allFinite() || !P_new.allFinite()) {
      res.status = UpdateResult::Status::REJECTED_NUMERIC;   // keep the previous, valid state
      return res;
    }
    x_ = x_new;
    P_ = P_new;
    return res;
  }

  ProcessNoise q_;
  double max_lag_ = 0.05;
  bool initialized_ = false;
  double t_ = 0.0;
  double rate_hold_ = 0.0;
  Vec8 x_ = Vec8::Zero();
  Mat8 P_ = Mat8::Identity();
};

// ============================================================================================
// Sensor fusion policy on top of Ekf8: initialisation, per-sensor duplicate detection, R
// selection and gates.  The ROS node only parses messages and logs what this class reports;
// the offline replay drives the very same object from a CSV file.
// ============================================================================================

struct FusionConfig
{
  ProcessNoise q;
  // initial standard deviations (position from the first GPS fix, yaw from the first VO yaw)
  double std0_pos_xy = 0.6, std0_pos_z = 0.9, std0_vel = 0.5, std0_yaw = 0.1, std0_bias = 0.05;
  // measurement noise
  Vec3 gps_std{0.6, 0.6, 0.9};
  bool gps_use_message_covariance = true;
  // VO: inflated above the per-sample noise in the message (0.03 m/s, 0.3 deg) because the real
  // VO error is dominated by its slowly drifting bias, which a white R cannot describe.
  double vo_velocity_std = 0.2, vo_yaw_std = 0.5;
  bool use_vo_velocity = true, use_vo_yaw = true;
  bool vo_velocity_ekf = true;
  // gates (d2 thresholds, <= 0 disables)
  double gps_gate = 16.27, vo_velocity_gate = 0.0, vo_yaw_gate = 0.0;
  int gate_max_consecutive = 5;
  double max_lag_s = 0.05;
  // static alignment (zero angular rate update) from initialisation until the drone first moves
  bool zaru_enabled = true;
  double zaru_rate_std = 0.005;       // [rad/s] R of "true yaw rate = 0" (gyro noise 0.004 + margin)
  double zaru_motion_speed = 0.15;    // [m/s] low-passed VO speed above this = motion, alignment ends
  double zaru_filter_tau = 0.5;       // [s]   low-pass time constant of the VO speed
  double zaru_gate = 10.83;           // d2 gate (1 DOF, 99.9 %): a larger gyro residual = rotation
  int zaru_rotation_samples = 3;      // this many gated samples in a row also end the alignment
};

enum class Sensor : int { GPS = 0, VO_VELOCITY = 1, VO_YAW = 2, IMU = 3 };

class SensorFusion
{
public:
  enum class Input { USED, NOT_READY, INITIALISED, DUPLICATE, TOO_OLD };

  explicit SensorFusion(const FusionConfig& cfg = FusionConfig()) { configure(cfg); }

  void configure(const FusionConfig& cfg)
  {
    cfg_ = cfg;
    ekf_.setProcessNoise(cfg.q);
    ekf_.setMaxLag(cfg.max_lag_s);
    gps_gate_ = Gate{cfg.gps_gate, cfg.gate_max_consecutive, 0};
    vel_gate_ = Gate{cfg.vo_velocity_gate, cfg.gate_max_consecutive, 0};
    yaw_gate_ = Gate{cfg.vo_yaw_gate, cfg.gate_max_consecutive, 0};
    zaru_gate_ = Gate{cfg.zaru_gate, 0, 0};
  }

  /// Forget everything (used only when simulation time jumps backwards, i.e. a world reset).
  void reset()
  {
    ekf_.reset();
    have_gps0_ = have_yaw0_ = false;
    static_ = true;
    static_ended_event_ = false;
    vo_speed_lp_ = 0.0;
    t_vo_speed_ = -1.0;
    zaru_updates_ = 0;
    zaru_rejected_in_row_ = 0;
    last_.fill(-std::numeric_limits<double>::infinity());
    configure(cfg_);
  }

  const Ekf8& ekf() const { return ekf_; }
  const FusionConfig& config() const { return cfg_; }
  bool ready() const { return ekf_.initialized(); }
  /// True while the static alignment runs (initialised, the drone has not moved yet).
  bool staticAlignment() const { return cfg_.zaru_enabled && static_; }
  long zaruUpdates() const { return zaru_updates_; }
  /// Reports once that the static alignment ended (for a single log line in the node).
  bool takeStaticEnded()
  {
    const bool e = static_ended_event_;
    static_ended_event_ = false;
    return e;
  }

  Input imu(double t, double w_z)
  {
    if (!fresh(Sensor::IMU, t)) return Input::DUPLICATE;
    if (!ready()) return Input::NOT_READY;
    if (ekf_.imu(t, w_z) == Ekf8::TimeStatus::TOO_OLD) return Input::TOO_OLD;
    // Standing still: the gyro sample is bias + noise.  This pins the bias before takeoff
    // independently of the slowly drifting VO yaw.
    if (staticAlignment()) {
      const UpdateResult r = ekf_.updateGyroBias(t, w_z, cfg_.zaru_rate_std * cfg_.zaru_rate_std, &zaru_gate_);
      if (r.accepted()) {
        ++zaru_updates_;
        zaru_rejected_in_row_ = 0;
      } else if (r.status == UpdateResult::Status::REJECTED_GATE && ++zaru_rejected_in_row_ >= cfg_.zaru_rotation_samples) {
        endStatic();   // the gyro sees a real rotation: not standing still any more
      }
    }
    return Input::USED;
  }

  /// R_msg: covariance from the message (used when configured and valid).
  Input gps(double t, const Vec3& z, const Mat3* R_msg, UpdateResult* out)
  {
    if (!fresh(Sensor::GPS, t)) return Input::DUPLICATE;
    if (!ready()) {
      gps0_ = z; t_gps0_ = t; have_gps0_ = true;
      return tryInitialize() ? Input::INITIALISED : Input::NOT_READY;
    }
    Mat3 R = cfg_.gps_std.cwiseProduct(cfg_.gps_std).asDiagonal();
    if (cfg_.gps_use_message_covariance && R_msg && R_msg->allFinite() && R_msg->diagonal().minCoeff() > 0.0)
      R = *R_msg;
    *out = ekf_.updateGps(t, z, R, &gps_gate_);
    return out->status == UpdateResult::Status::REJECTED_OLD ? Input::TOO_OLD : Input::USED;
  }

  /// One VO message: yaw first (the velocity Jacobian then uses the corrected yaw), then velocity.
  Input vo(double t, double yaw, const Vec3& v_body, UpdateResult* out_yaw, UpdateResult* out_vel)
  {
    if (!fresh(Sensor::VO_VELOCITY, t)) return Input::DUPLICATE;
    if (!ready()) {
      yaw0_ = yaw; t_yaw0_ = t; have_yaw0_ = true;
      return tryInitialize() ? Input::INITIALISED : Input::NOT_READY;
    }
    detectMotion(t, v_body);
    if (cfg_.use_vo_yaw)
      *out_yaw = ekf_.updateYaw(t, yaw, cfg_.vo_yaw_std * cfg_.vo_yaw_std, &yaw_gate_);
    if (cfg_.use_vo_velocity) {
      const Mat3 R = Mat3::Identity() * (cfg_.vo_velocity_std * cfg_.vo_velocity_std);
      *out_vel = ekf_.updateBodyVelocity(t, v_body, R, cfg_.vo_velocity_ekf, &vel_gate_);
    }
    const bool old = (cfg_.use_vo_yaw && out_yaw->status == UpdateResult::Status::REJECTED_OLD) ||
                     (cfg_.use_vo_velocity && out_vel->status == UpdateResult::Status::REJECTED_OLD);
    return old ? Input::TOO_OLD : Input::USED;
  }

  double gateThreshold(Sensor s) const
  {
    switch (s) {
      case Sensor::GPS: return gps_gate_.threshold;
      case Sensor::VO_VELOCITY: return vel_gate_.threshold;
      case Sensor::VO_YAW: return yaw_gate_.threshold;
      default: return 0.0;
    }
  }

private:
  /// Ends the static alignment for good once the low-passed VO speed shows motion (takeoff).
  /// One VO sample has ~0.04 m/s noise per axis, the low-pass removes it.
  void detectMotion(double t, const Vec3& v_body)
  {
    if (!static_) return;
    const double dt = t_vo_speed_ < 0.0 ? 0.0 : t - t_vo_speed_;
    t_vo_speed_ = t;
    const double a = cfg_.zaru_filter_tau > 0.0 ? std::min(1.0, dt / cfg_.zaru_filter_tau) : 1.0;
    vo_speed_lp_ += a * (v_body.norm() - vo_speed_lp_);
    if (vo_speed_lp_ > cfg_.zaru_motion_speed) endStatic();
  }

  void endStatic()
  {
    if (!static_) return;
    static_ = false;
    static_ended_event_ = cfg_.zaru_enabled;
  }

  /// Duplicate / out-of-order detection per sensor (stamps must strictly increase).
  bool fresh(Sensor s, double t)
  {
    double& last = last_[static_cast<int>(s)];
    if (!(t > last)) return false;
    last = t;
    return true;
  }

  /// Deliberate initialisation: position from the first GPS fix, yaw from the first VO yaw
  /// (vo_odom starts aligned with the map), velocity zero and bias zero with wide uncertainty.
  bool tryInitialize()
  {
    if (!have_gps0_ || !have_yaw0_) return false;
    Vec8 std0;
    std0 << cfg_.std0_pos_xy, cfg_.std0_pos_xy, cfg_.std0_pos_z, cfg_.std0_vel, cfg_.std0_vel, cfg_.std0_vel,
      cfg_.std0_yaw, cfg_.std0_bias;
    ekf_.initialize(std::max(t_gps0_, t_yaw0_), gps0_, yaw0_, std0);
    return true;
  }

  FusionConfig cfg_;
  Ekf8 ekf_;
  Gate gps_gate_, vel_gate_, yaw_gate_, zaru_gate_;
  std::array<double, 4> last_{{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}};
  bool have_gps0_ = false, have_yaw0_ = false;
  bool static_ = true, static_ended_event_ = false;
  double vo_speed_lp_ = 0.0, t_vo_speed_ = -1.0;
  long zaru_updates_ = 0;
  int zaru_rejected_in_row_ = 0;
  Vec3 gps0_ = Vec3::Zero();
  double t_gps0_ = 0.0, yaw0_ = 0.0, t_yaw0_ = 0.0;
};

}  // namespace loc
