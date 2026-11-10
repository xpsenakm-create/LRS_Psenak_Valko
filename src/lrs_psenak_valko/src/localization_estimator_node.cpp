// localization_estimator_node.cpp -- A2: eight-state EKF localisation from the three
// /localization/ sensors (the maths is in ekf8.h, shared with the offline replay and the test).
//
//   /localization/gps  (PoseWithCovarianceStamped, map, 5 Hz)    -> position update (gated)
//   /localization/vo   (Odometry, vo_odom -> base_link, 30 Hz)   -> yaw + body-velocity update
//   /localization/imu  (Imu, base_link, 100 Hz, only w_z valid)   -> prediction
//   ==> /estimator/odometry (Odometry, frame map, child base_link, 100 Hz)
//
// The node subscribes to NOTHING else (no truth, no MAVROS).  Every time is a header stamp in
// simulation time; the timeout watchdog compares the node clock (use_sim_time: true) with the
// last stamp of each sensor, so both sides are simulation time.
//
// Logged (and published on /estimator/events, so they are in the bag) -- never at sensor rate:
//   initialisation, sensor LOST / BACK transitions, every rejected GPS sample with its d2,
//   a periodic status line (rates from header stamps, estimate, sigmas, counters).
//
// Single-threaded executor on purpose: callbacks never run concurrently, the filter needs no mutex.

#include "ekf8.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using loc::Vec3;
using loc::Mat3;
using loc::Sensor;
using loc::SensorFusion;
using loc::UpdateResult;

namespace
{
constexpr double kRad2Deg = 180.0 / loc::kPi;

double stampSeconds(const builtin_interfaces::msg::Time& s) { return s.sec + s.nanosec * 1e-9; }

builtin_interfaces::msg::Time toStamp(double t)
{
  builtin_interfaces::msg::Time s;
  const double sec = std::floor(t);
  s.sec = static_cast<int32_t>(sec);
  s.nanosec = static_cast<uint32_t>(std::min(999999999.0, std::round((t - sec) * 1e9)));
  return s;
}

const char* statusName(UpdateResult::Status s)
{
  switch (s) {
    case UpdateResult::Status::ACCEPTED: return "accepted";
    case UpdateResult::Status::ACCEPTED_FORCED: return "accepted (lock-out guard)";
    case UpdateResult::Status::REJECTED_GATE: return "REJECTED by gate";
    case UpdateResult::Status::REJECTED_OLD: return "too old";
    case UpdateResult::Status::REJECTED_NUMERIC: return "numerically invalid";
  }
  return "?";
}
}  // namespace

class LocalizationEstimatorNode final : public rclcpp::Node
{
public:
  LocalizationEstimatorNode() : Node("localization_estimator")
  {
    declareParameters();
    fusion_.configure(cfg_);
    createInterfaces();
    const auto& c = cfg_;
    RCLCPP_INFO(get_logger(),
      "Q: accel_xy=%.3f accel_z=%.3f m/s^2/sqrt(Hz), yaw_rate=%.4f rad/s/sqrt(Hz), gyro_bias_rw=%.1e rad/s/sqrt(s)",
      c.q.accel_xy, c.q.accel_z, c.q.yaw_rate, c.q.gyro_bias_rw);
    RCLCPP_INFO(get_logger(),
      "R: GPS std=(%.2f, %.2f, %.2f) m%s | VO velocity std=%.3f m/s (%s) | VO yaw std=%.3f rad%s%s",
      c.gps_std.x(), c.gps_std.y(), c.gps_std.z(), c.gps_use_message_covariance ? " (message covariance preferred)" : "",
      c.vo_velocity_std, c.vo_velocity_ekf ? "EKF body-velocity model" : "rotated with yaw estimate, linear KF",
      c.vo_yaw_std, c.use_vo_yaw ? "" : " [VO yaw OFF]", c.use_vo_velocity ? "" : " [VO velocity OFF]");
    RCLCPP_INFO(get_logger(),
      "P0 std: pos_xy=%.2f pos_z=%.2f m, vel=%.2f m/s, yaw=%.3f rad, bias=%.3f rad/s | GPS gate d2 <= %.2f "
      "(lock-out guard after %d), VO gates %.2f / %.2f, max lag %.3f s",
      c.std0_pos_xy, c.std0_pos_z, c.std0_vel, c.std0_yaw, c.std0_bias, c.gps_gate, c.gate_max_consecutive,
      c.vo_velocity_gate, c.vo_yaw_gate, c.max_lag_s);
    event(Level::INFO, "Estimator waiting for the first GPS fix and the first VO yaw (initialisation)");
  }

private:
  enum class Level { INFO, WARN, ERROR };

  /// Per-sensor bookkeeping: timeout watchdog and rate measured from HEADER stamps.
  struct Channel
  {
    std::string name;
    double timeout = 1.0;
    bool seen = false, lost = false;
    double last_stamp = -std::numeric_limits<double>::infinity();
    double lost_at = 0.0;
    bool report_next_update = false; // log the first update after a gap
    // statistics window
    long n = 0;
    double first = 0.0, last = 0.0;
    long rejected = 0, late = 0, dropped = 0, bad_frame = 0;

    void arrived(double stamp)
    {
      if (n == 0) first = stamp;
      last = stamp;
      ++n;
    }
    double rate() const { return (n > 1 && last > first) ? (n - 1) / (last - first) : 0.0; }
    void resetWindow() { n = 0; }
  };

  // ----------------------------------------------------------------------- parameters
  void declareParameters()
  {
    auto& c = cfg_;
    c.q.accel_xy = declare_parameter<double>("process_noise.accel_xy", c.q.accel_xy);
    c.q.accel_z = declare_parameter<double>("process_noise.accel_z", c.q.accel_z);
    c.q.yaw_rate = declare_parameter<double>("process_noise.yaw_rate", c.q.yaw_rate);
    c.q.gyro_bias_rw = declare_parameter<double>("process_noise.gyro_bias_rw", c.q.gyro_bias_rw);

    c.std0_pos_xy = declare_parameter<double>("initial_std.position_xy", c.std0_pos_xy);
    c.std0_pos_z = declare_parameter<double>("initial_std.position_z", c.std0_pos_z);
    c.std0_vel = declare_parameter<double>("initial_std.velocity", c.std0_vel);
    c.std0_yaw = declare_parameter<double>("initial_std.yaw", c.std0_yaw);
    c.std0_bias = declare_parameter<double>("initial_std.gyro_bias", c.std0_bias);

    const auto gps_std = declare_parameter<std::vector<double>>(
      "gps.position_std", {c.gps_std.x(), c.gps_std.y(), c.gps_std.z()});
    if (gps_std.size() != 3) throw std::invalid_argument("gps.position_std needs 3 values");
    c.gps_std = Vec3(gps_std[0], gps_std[1], gps_std[2]);
    c.gps_use_message_covariance = declare_parameter<bool>("gps.use_message_covariance", c.gps_use_message_covariance);
    c.gps_gate = declare_parameter<double>("gps.gate_d2", c.gps_gate);

    c.vo_velocity_std = declare_parameter<double>("vo.velocity_std", c.vo_velocity_std);
    c.vo_yaw_std = declare_parameter<double>("vo.yaw_std", c.vo_yaw_std);
    c.use_vo_velocity = declare_parameter<bool>("vo.use_velocity", c.use_vo_velocity);
    c.use_vo_yaw = declare_parameter<bool>("vo.use_yaw", c.use_vo_yaw);
    const std::string model = declare_parameter<std::string>("vo.velocity_model", "ekf");
    if (model != "ekf" && model != "rotate") throw std::invalid_argument("vo.velocity_model must be 'ekf' or 'rotate'");
    c.vo_velocity_ekf = (model == "ekf");
    c.vo_velocity_gate = declare_parameter<double>("vo.velocity_gate_d2", c.vo_velocity_gate);
    c.vo_yaw_gate = declare_parameter<double>("vo.yaw_gate_d2", c.vo_yaw_gate);

    c.gate_max_consecutive = static_cast<int>(declare_parameter<int64_t>("gate_max_consecutive_rejects", c.gate_max_consecutive));
    c.max_lag_s = declare_parameter<double>("max_measurement_lag_s", c.max_lag_s);

    gps_.name = "GPS";
    vo_.name = "VO";
    imu_.name = "IMU";
    gps_.timeout = declare_parameter<double>("timeouts.gps_s", 1.0);
    vo_.timeout = declare_parameter<double>("timeouts.vo_s", 0.5);
    imu_.timeout = declare_parameter<double>("timeouts.imu_s", 0.2);
    time_reset_s_ = declare_parameter<double>("time_reset_threshold_s", 1.0);
    stats_period_s_ = declare_parameter<double>("stats_period_s", 5.0);

    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    vo_frame_ = declare_parameter<std::string>("frames.vo", "vo_odom");
    body_frame_ = declare_parameter<std::string>("frames.body", "base_link");
    odom_topic_ = declare_parameter<std::string>("output_topic", "/estimator/odometry");

    if (c.q.accel_xy <= 0.0 || c.q.accel_z <= 0.0 || c.q.yaw_rate <= 0.0 || c.q.gyro_bias_rw < 0.0)
      throw std::invalid_argument("process noise must be > 0");
    if (c.gps_std.minCoeff() <= 0.0 || c.vo_velocity_std <= 0.0 || c.vo_yaw_std <= 0.0)
      throw std::invalid_argument("measurement standard deviations must be > 0");
    if (c.max_lag_s < 0.0 || stats_period_s_ <= 0.0) throw std::invalid_argument("max lag / stats period");
  }

  // ----------------------------------------------------------------------- interfaces
  void createInterfaces()
  {
    const auto qos = rclcpp::SensorDataQoS();   // the simulator publishes best effort
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/localization/gps", qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { onGps(*m); });
    vo_sub_ = create_subscription<nav_msgs::msg::Odometry>("/localization/vo", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onVo(*m); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/localization/imu", qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { onImu(*m); });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/estimator/debug", 10);
    innovation_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/estimator/innovation", 50);
    events_pub_ = create_publisher<std_msgs::msg::String>("/estimator/events", rclcpp::QoS(100).transient_local());

    // Node clock = simulation time: the watchdog compares like with like (stamps are sim time).
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this]() { watchdog(); });
  }

  void event(Level level, const char* fmt, ...)
  {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    switch (level) {
      case Level::INFO: RCLCPP_INFO(get_logger(), "%s", buf); break;
      case Level::WARN: RCLCPP_WARN(get_logger(), "%s", buf); break;
      case Level::ERROR: RCLCPP_ERROR(get_logger(), "%s", buf); break;
    }
    std_msgs::msg::String msg;
    msg.data = buf;
    events_pub_->publish(msg);
  }

  bool frameOk(Channel& ch, const std::string& got, const std::string& want, const char* what)
  {
    if (got == want) return true;
    ++ch.bad_frame;
    RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, 5000, "%s %s is '%s', expected '%s' -- message ignored",
      ch.name.c_str(), what, got.c_str(), want.c_str());
    return false;
  }

  /// Common arrival handling: recovery transition, rate statistics.
  void arrived(Channel& ch, double stamp)
  {
    if (ch.lost) {
      ch.lost = false;
      ch.report_next_update = true;
      event(Level::INFO, "%s BACK after a %.1f s gap (stamp %.2f) -- resuming updates, no reset", ch.name.c_str(),
        stamp - ch.last_stamp, stamp);
    }
    ch.seen = true;
    ch.last_stamp = stamp;
    ch.arrived(stamp);
  }

  // ----------------------------------------------------------------------- callbacks
  void onImu(const sensor_msgs::msg::Imu& m)
  {
    if (!frameOk(imu_, m.header.frame_id, body_frame_, "frame_id")) return;
    const double t = stampSeconds(m.header.stamp);
    if (imu_.seen && t < imu_.last_stamp - time_reset_s_) {
      event(Level::WARN, "Simulation time jumped back %.2f -> %.2f s (world reset?) -- estimator restarts",
        imu_.last_stamp, t);
      fusion_.reset();
      for (Channel* ch : {&gps_, &vo_, &imu_}) { ch->seen = ch->lost = false; ch->last_stamp = -1e18; ch->resetWindow(); }
      initialised_logged_ = false;
    }
    arrived(imu_, t);
    const SensorFusion::Input in = fusion_.imu(t, m.angular_velocity.z);
    if (in == SensorFusion::Input::DUPLICATE) { ++imu_.dropped; return; }
    if (in == SensorFusion::Input::TOO_OLD) { ++imu_.dropped; return; }
    if (in != SensorFusion::Input::USED) return;
    publishEstimate();
  }

  void onGps(const geometry_msgs::msg::PoseWithCovarianceStamped& m)
  {
    if (!frameOk(gps_, m.header.frame_id, map_frame_, "frame_id")) return;
    const double t = stampSeconds(m.header.stamp);
    arrived(gps_, t);
    const auto& p = m.pose.pose.position;
    const auto& cv = m.pose.covariance;   // row-major 6x6 (x, y, z, rx, ry, rz)
    Mat3 R;
    R << cv[0], cv[1], cv[2], cv[6], cv[7], cv[8], cv[12], cv[13], cv[14];
    const double sxy_before = sigmaXY();
    UpdateResult r;
    const SensorFusion::Input in = fusion_.gps(t, Vec3(p.x, p.y, p.z), &R, &r);
    if (in == SensorFusion::Input::INITIALISED) { logInitialised(); return; }
    if (in == SensorFusion::Input::DUPLICATE || in == SensorFusion::Input::TOO_OLD) { ++gps_.dropped; return; }
    if (in != SensorFusion::Input::USED) return;
    if (r.late) ++gps_.late;
    publishInnovation(t, Sensor::GPS, r);
    const double rn = std::sqrt(r.innovation[0] * r.innovation[0] + r.innovation[1] * r.innovation[1] +
                                r.innovation[2] * r.innovation[2]);
    if (r.status == UpdateResult::Status::REJECTED_GATE) {
      ++gps_.rejected;
      event(Level::WARN,
        "GPS REJECTED at t=%.2f: d2 = r^T S^-1 r = %.1f > gate %.2f (|r| = %.2f m, r = (%.2f, %.2f, %.2f)); "
        "a valid fix exceeds the gate with probability 0.1 %% (chi-square, 3 DOF)",
        t, r.d2, cfg_.gps_gate, rn, r.innovation[0], r.innovation[1], r.innovation[2]);
    } else if (r.status == UpdateResult::Status::ACCEPTED_FORCED) {
      event(Level::WARN, "GPS accepted at t=%.2f after %d rejections in a row (gate lock-out guard): d2=%.1f",
        t, cfg_.gate_max_consecutive, r.d2);
    } else if (r.status == UpdateResult::Status::REJECTED_NUMERIC) {
      event(Level::ERROR, "GPS update at t=%.2f skipped: numerically invalid (state kept)", t);
    }
    if (gps_.report_next_update && r.accepted()) {
      gps_.report_next_update = false;
      event(Level::INFO, "First GPS update after the gap: |r| = %.2f m, d2 = %.1f, sigma_xy %.2f -> %.2f m",
        rn, r.d2, sxy_before, sigmaXY());
    }
  }

  void onVo(const nav_msgs::msg::Odometry& m)
  {
    if (!frameOk(vo_, m.header.frame_id, vo_frame_, "frame_id")) return;
    if (!frameOk(vo_, m.child_frame_id, body_frame_, "child_frame_id")) return;   // twist is body-frame
    const double t = stampSeconds(m.header.stamp);
    arrived(vo_, t);
    const auto& q = m.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto& v = m.twist.twist.linear;
    UpdateResult ry, rv;
    const SensorFusion::Input in = fusion_.vo(t, yaw, Vec3(v.x, v.y, v.z), &ry, &rv);
    if (in == SensorFusion::Input::INITIALISED) { logInitialised(); return; }
    if (in == SensorFusion::Input::DUPLICATE || in == SensorFusion::Input::TOO_OLD) { ++vo_.dropped; return; }
    if (in != SensorFusion::Input::USED) return;
    if ((cfg_.use_vo_yaw && ry.late) || (cfg_.use_vo_velocity && rv.late)) ++vo_.late;
    for (const auto& [sensor, r, use] : {std::make_tuple(Sensor::VO_YAW, ry, cfg_.use_vo_yaw),
                                          std::make_tuple(Sensor::VO_VELOCITY, rv, cfg_.use_vo_velocity)}) {
      if (!use) continue;
      publishInnovation(t, sensor, r);
      if (!r.accepted() && r.status != UpdateResult::Status::REJECTED_OLD) {
        ++vo_.rejected;
        RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 2000, "VO %s %s at t=%.2f (d2=%.1f)",
          sensor == Sensor::VO_YAW ? "yaw" : "velocity", statusName(r.status), t, r.d2);
      }
    }
    if (vo_.report_next_update) {
      vo_.report_next_update = false;
      event(Level::INFO, "First VO update after the gap: velocity d2 = %.1f, yaw innovation = %.2f deg",
        rv.d2, ry.innovation[0] * kRad2Deg);
    }
  }

  void logInitialised()
  {
    if (initialised_logged_) return;
    initialised_logged_ = true;
    const auto& x = fusion_.ekf().x();
    event(Level::INFO,
      "Estimator INITIALISED at t=%.2f: position (%.2f, %.2f, %.2f) from the first GPS fix, yaw %.1f deg from the "
      "first VO yaw, velocity 0 and gyro bias 0 with wide uncertainty", fusion_.ekf().time(), x(loc::X), x(loc::Y),
      x(loc::Z), x(loc::YAW) * kRad2Deg);
  }

  // ----------------------------------------------------------------------- watchdog / status
  void watchdog()
  {
    const double now_s = now().seconds();
    for (Channel* ch : {&gps_, &vo_, &imu_}) {
      // stamp vs. node clock: both simulation time (no false alarm when /clock first jumps from 0)
      if (!ch->seen || ch->lost || now_s - ch->last_stamp <= ch->timeout) continue;
      ch->lost = true;
      ch->lost_at = now_s;
      const char* carry = ch == &gps_ ? "VO velocity + IMU prediction carry the position"
                        : ch == &vo_ ? "GPS + constant-velocity model carry the estimate"
                                     : "prediction continues at each GPS/VO stamp with the last gyro rate";
      event(Level::WARN, "%s LOST: no message for %.2f s (last stamp %.2f) -- %s", ch->name.c_str(),
        now_s - ch->last_stamp, ch->last_stamp, carry);
    }
    if (now_s - last_stats_ >= stats_period_s_) {
      last_stats_ = now_s;
      printStatus();
    }
  }

  double sigmaXY() const
  {
    if (!fusion_.ready()) return std::nan("");
    const auto& P = fusion_.ekf().P();
    return std::sqrt(std::max(P(loc::X, loc::X), P(loc::Y, loc::Y)));
  }

  void printStatus()
  {
    char rates[160];
    std::snprintf(rates, sizeof(rates), "GPS %.2f Hz, VO %.1f Hz, IMU %.1f Hz", gps_.rate(), vo_.rate(), imu_.rate());
    for (Channel* ch : {&gps_, &vo_, &imu_}) ch->resetWindow();
    if (!fusion_.ready()) {
      RCLCPP_INFO(get_logger(), "[not initialised] rates from header stamps: %s", rates);
      return;
    }
    const auto& x = fusion_.ekf().x();
    const auto& P = fusion_.ekf().P();
    RCLCPP_INFO(get_logger(),
      "[t=%.1f] rates from header stamps: %s | p=(%.2f, %.2f, %.2f) yaw=%.1f deg b_g=%+.4f rad/s | "
      "sigma xy=%.2f z=%.2f m yaw=%.2f deg b_g=%.4f | GPS rejected %ld late %ld dropped %ld | VO late %ld dropped %ld",
      fusion_.ekf().time(), rates, x(loc::X), x(loc::Y), x(loc::Z), x(loc::YAW) * kRad2Deg, x(loc::BG), sigmaXY(),
      std::sqrt(P(loc::Z, loc::Z)), std::sqrt(P(loc::YAW, loc::YAW)) * kRad2Deg, std::sqrt(P(loc::BG, loc::BG)),
      gps_.rejected, gps_.late, gps_.dropped, vo_.late, vo_.dropped);
  }

  // ----------------------------------------------------------------------- outputs
  void publishEstimate()
  {
    const auto& ekf = fusion_.ekf();
    const auto& x = ekf.x();
    const auto& P = ekf.P();
    if (!x.allFinite() || !P.allFinite()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, 2000, "estimate not finite -- not published");
      return;
    }
    const double yaw = x(loc::YAW);
    nav_msgs::msg::Odometry o;
    o.header.stamp = toStamp(ekf.time());
    o.header.frame_id = map_frame_;
    o.child_frame_id = body_frame_;
    o.pose.pose.position.x = x(loc::X);
    o.pose.pose.position.y = x(loc::Y);
    o.pose.pose.position.z = x(loc::Z);
    o.pose.pose.orientation.z = std::sin(yaw / 2.0);   // roll and pitch are not estimated (0)
    o.pose.pose.orientation.w = std::cos(yaw / 2.0);
    // pose covariance: (x, y, z, roll, pitch, yaw), roll/pitch marked unknown
    auto& pc = o.pose.covariance;
    const int idx[6] = {loc::X, loc::Y, loc::Z, -1, -1, loc::YAW};
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c)
        pc[r * 6 + c] = (idx[r] < 0 || idx[c] < 0) ? (r == c ? 1e6 : 0.0) : P(idx[r], idx[c]);
    // twist in the child frame (base_link), as nav_msgs/Odometry defines it
    const Vec3 vb = loc::worldToBody(x.segment<3>(loc::VX), yaw);
    o.twist.twist.linear.x = vb.x();
    o.twist.twist.linear.y = vb.y();
    o.twist.twist.linear.z = vb.z();
    o.twist.twist.angular.z = ekf.rateHold() - x(loc::BG);
    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0.0, std::sin(yaw), std::cos(yaw), 0.0, 0.0, 0.0, 1.0;
    const Eigen::Matrix3d Pvb = Rz.transpose() * P.block<3, 3>(loc::VX, loc::VX) * Rz;
    auto& tc = o.twist.covariance;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) tc[r * 6 + c] = Pvb(r, c);
    tc[21] = tc[28] = 1e6;
    tc[35] = P(loc::BG, loc::BG);
    odom_pub_->publish(o);

    std_msgs::msg::Float64MultiArray d;
    d.layout.dim.resize(1);
    d.layout.dim[0].label = "t,x,y,z,vx,vy,vz,yaw,b_g,sx,sy,sz,svx,svy,svz,syaw,sb_g";
    d.layout.dim[0].size = 17;
    d.layout.dim[0].stride = 17;
    d.data.reserve(17);
    d.data.push_back(ekf.time());
    for (int i = 0; i < loc::N; ++i) d.data.push_back(x(i));
    for (int i = 0; i < loc::N; ++i) d.data.push_back(std::sqrt(P(i, i)));
    debug_pub_->publish(d);
  }

  void publishInnovation(double t, Sensor s, const UpdateResult& r)
  {
    std_msgs::msg::Float64MultiArray m;
    m.layout.dim.resize(1);
    m.layout.dim[0].label = "t,sensor(0 gps 1 vo_vel 2 vo_yaw),status(0 ok 1 forced 2 gate 3 old 4 numeric),d2,gate,r0,r1,r2";
    m.layout.dim[0].size = 8;
    m.layout.dim[0].stride = 8;
    m.data = {t, static_cast<double>(static_cast<int>(s)), static_cast<double>(static_cast<int>(r.status)), r.d2,
              fusion_.gateThreshold(s), r.innovation[0], r.innovation[1], r.innovation[2]};
    innovation_pub_->publish(m);
  }

  // ----------------------------------------------------------------------- members
  loc::FusionConfig cfg_;
  SensorFusion fusion_;
  Channel gps_, vo_, imu_;
  double time_reset_s_ = 1.0, stats_period_s_ = 5.0, last_stats_ = 0.0;
  bool initialised_logged_ = false;
  std::string map_frame_, vo_frame_, body_frame_, odom_topic_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_, innovation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalizationEstimatorNode>());   // single-threaded on purpose
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_estimator"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
