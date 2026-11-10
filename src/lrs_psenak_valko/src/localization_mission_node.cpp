// localization_mission_node.cpp -- A2 section 4: the A2 route flown on OUR estimate.
//
//   WAIT_ESTIMATE -> SET_MODE -> ARM -> TAKEOFF -> ROUTE (two laps) -> LAND -> DONE
//   any flight state --(abort service / geofence / tracking error / estimate lost)--> ABORTING -> ABORTED
//   any flight state --(RC mode change / MAVROS drop-out / disarm)-----------------> STOPPED (no setpoints!)
//
// FEEDBACK: only /estimator/odometry (our EKF) and /mavros/state.  No MAVROS position, odometry,
// attitude, heading or IMU topic is subscribed.
// OUTPUT:   /mavros/setpoint_raw/local with VELOCITY + YAW RATE, position and yaw fields ignored.
//           The position loop is closed HERE, on the estimate:
//               v_cmd = v_carrot + Kp (carrot - p_est),   yaw_rate = K_yaw wrap(yaw_hold - yaw_est)
//           so if the estimate moves, the drone moves with it.  ArduPilot only tracks velocity.
// DEGRADED ESTIMATE: when sigma_xy of the estimate grows (VO tracking loss: GPS alone cannot carry the
//           velocity), the carrot slows down and stops, and the feedback gain drops, so the drone
//           holds still instead of chasing a noisy estimate.  Full speed again once sigma recovers.
//
// ROUTE: relative ENU offsets from the ORIGIN = the first valid estimated position (estimate
// initialised AND its sigma below origin_max_std_*), flown laps times at flight_altitude_m.
// Takeoff is the MAVROS takeoff command (a service, allowed); landing is LAND mode.
//
// Times: node clock (use_sim_time: true), so speeds are in simulation metres per second even when
// Gazebo runs slower than real time.  Single-threaded executor on purpose (no data races).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

struct Vec3
{
  double x = 0.0, y = 0.0, z = 0.0;
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double k) const { return {x * k, y * k, z * k}; }
  double norm() const { return std::sqrt(x * x + y * y + z * z); }
  double normXY() const { return std::hypot(x, y); }
};

double wrapPi(double a) { return a - 2.0 * kPi * std::floor((a + kPi) / (2.0 * kPi)); }
}  // namespace

class LocalizationMissionNode final : public rclcpp::Node
{
public:
  LocalizationMissionNode() : Node("localization_mission")
  {
    declareParameters();
    createInterfaces();
    RCLCPP_INFO(get_logger(),
      "Route: %zu corners x %d laps at %.2f m, %.2f m/s (accel %.2f m/s^2); feedback = %s (our estimate only)",
      offsets_.size(), p_.laps, p_.flight_altitude, p_.cruise_speed, p_.max_accel, p_.estimate_topic.c_str());
    RCLCPP_INFO(get_logger(), "Control: Kp_xy=%.2f Kp_z=%.2f 1/s, |v_xy|<=%.2f |v_z|<=%.2f m/s, Kp_yaw=%.2f 1/s, "
      "|yaw rate|<=%.0f deg/s", p_.kp_xy, p_.kp_z, p_.max_vel_xy, p_.max_vel_z, p_.kp_yaw, p_.max_yaw_rate_deg);
    RCLCPP_INFO(get_logger(), "Degraded estimate: sigma_xy > %.2f m slows the route, > %.2f m holds it "
      "(feedback gain down to x%.2f)", p_.degraded_std_xy, p_.hold_std_xy, p_.degraded_kp_factor);
    RCLCPP_INFO(get_logger(), "State: %s", stateName(state_));
    publishState();
  }

private:
  // ------------------------------------------------------------------ types
  enum class State { WAIT_ESTIMATE, SET_MODE, ARM, TAKEOFF, ROUTE, LAND, DONE, ABORTING, ABORTED, STOPPED, FAULT };

  static const char* stateName(State s)
  {
    switch (s) {
      case State::WAIT_ESTIMATE: return "WAIT_ESTIMATE";
      case State::SET_MODE: return "SET_MODE";
      case State::ARM: return "ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::ROUTE: return "ROUTE";
      case State::LAND: return "LAND";
      case State::DONE: return "DONE";
      case State::ABORTING: return "ABORTING";
      case State::ABORTED: return "ABORTED";
      case State::STOPPED: return "STOPPED";
      case State::FAULT: return "FAULT";
    }
    return "?";
  }

  struct Params
  {
    std::string estimate_topic = "/estimator/odometry";
    double flight_altitude = 2.0, cruise_speed = 0.5, max_accel = 0.5;
    int laps = 2;
    double control_rate_hz = 30.0;
    double kp_xy = 1.0, kp_z = 1.0, max_vel_xy = 1.0, max_vel_z = 0.5;
    double kp_yaw = 1.0, max_yaw_rate_deg = 30.0;
    double waypoint_tol = 0.3, max_lead = 1.0;
    double degraded_std_xy = 0.2, hold_std_xy = 0.28, degraded_kp_factor = 0.3;
    double takeoff_tol = 0.5, stabilize_time = 2.0, stabilize_speed = 0.15;
    double origin_max_std_xy = 0.15, origin_max_std_z = 0.2;
    double max_horizontal = 6.0, max_altitude = 3.5, max_tracking_error = 2.0, max_estimate_std = 2.0;
    double estimate_timeout = 0.5;
    std::string abort_mode = "LAND";
    double handshake_timeout = 60.0, takeoff_timeout = 40.0, route_timeout = 240.0, landing_timeout = 60.0;
    double service_timeout = 5.0, status_period = 2.0;
  };

  // ----------------------------------------------------------- parameters
  void declareParameters()
  {
    p_.estimate_topic = declare_parameter<std::string>("estimate_topic", p_.estimate_topic);
    p_.flight_altitude = declare_parameter<double>("flight_altitude_m", p_.flight_altitude);
    const auto flat = declare_parameter<std::vector<double>>("route_offsets_xy",
      {0.0, -4.0, 2.5, -4.0, 2.5, 0.0, 0.0, 0.0});
    p_.laps = static_cast<int>(declare_parameter<int64_t>("laps", p_.laps));
    p_.cruise_speed = declare_parameter<double>("cruise_speed_mps", p_.cruise_speed);
    p_.max_accel = declare_parameter<double>("max_accel_mps2", p_.max_accel);
    p_.control_rate_hz = declare_parameter<double>("control_rate_hz", p_.control_rate_hz);
    p_.kp_xy = declare_parameter<double>("kp_xy", p_.kp_xy);
    p_.kp_z = declare_parameter<double>("kp_z", p_.kp_z);
    p_.max_vel_xy = declare_parameter<double>("max_velocity_xy_mps", p_.max_vel_xy);
    p_.max_vel_z = declare_parameter<double>("max_velocity_z_mps", p_.max_vel_z);
    p_.kp_yaw = declare_parameter<double>("kp_yaw", p_.kp_yaw);
    p_.max_yaw_rate_deg = declare_parameter<double>("max_yaw_rate_deg_s", p_.max_yaw_rate_deg);
    p_.waypoint_tol = declare_parameter<double>("waypoint_tolerance_m", p_.waypoint_tol);
    p_.max_lead = declare_parameter<double>("max_lead_m", p_.max_lead);
    p_.degraded_std_xy = declare_parameter<double>("degraded_std_xy_m", p_.degraded_std_xy);
    p_.hold_std_xy = declare_parameter<double>("hold_std_xy_m", p_.hold_std_xy);
    p_.degraded_kp_factor = declare_parameter<double>("degraded_kp_factor", p_.degraded_kp_factor);
    p_.takeoff_tol = declare_parameter<double>("takeoff_tolerance_m", p_.takeoff_tol);
    p_.stabilize_time = declare_parameter<double>("stabilize_time_s", p_.stabilize_time);
    p_.stabilize_speed = declare_parameter<double>("stabilize_speed_mps", p_.stabilize_speed);
    p_.origin_max_std_xy = declare_parameter<double>("origin_max_std_xy_m", p_.origin_max_std_xy);
    p_.origin_max_std_z = declare_parameter<double>("origin_max_std_z_m", p_.origin_max_std_z);
    p_.max_horizontal = declare_parameter<double>("max_horizontal_from_origin_m", p_.max_horizontal);
    p_.max_altitude = declare_parameter<double>("max_altitude_m", p_.max_altitude);
    p_.max_tracking_error = declare_parameter<double>("max_tracking_error_m", p_.max_tracking_error);
    p_.max_estimate_std = declare_parameter<double>("max_estimate_std_m", p_.max_estimate_std);
    p_.estimate_timeout = declare_parameter<double>("estimate_timeout_s", p_.estimate_timeout);
    p_.abort_mode = declare_parameter<std::string>("abort_mode", p_.abort_mode);
    p_.handshake_timeout = declare_parameter<double>("handshake_timeout_s", p_.handshake_timeout);
    p_.takeoff_timeout = declare_parameter<double>("takeoff_timeout_s", p_.takeoff_timeout);
    p_.route_timeout = declare_parameter<double>("route_timeout_s", p_.route_timeout);
    p_.landing_timeout = declare_parameter<double>("landing_timeout_s", p_.landing_timeout);
    p_.service_timeout = declare_parameter<double>("service_timeout_s", p_.service_timeout);
    p_.status_period = declare_parameter<double>("status_period_s", p_.status_period);

    auto bad = [](const std::string& m) { throw std::invalid_argument("invalid parameter: " + m); };
    if (flat.empty() || flat.size() % 2 != 0) bad("route_offsets_xy must hold x,y pairs");
    for (size_t i = 0; i + 1 < flat.size(); i += 2) offsets_.push_back({flat[i], flat[i + 1], 0.0});
    if (p_.laps < 1) bad("laps must be >= 1");
    if (p_.flight_altitude < 0.5 || p_.flight_altitude > p_.max_altitude) bad("0.5 <= flight_altitude_m <= max_altitude_m");
    if (p_.cruise_speed <= 0.0 || p_.cruise_speed > p_.max_vel_xy) bad("0 < cruise_speed_mps <= max_velocity_xy_mps");
    if (p_.max_accel <= 0.0 || p_.kp_xy <= 0.0 || p_.kp_z <= 0.0 || p_.kp_yaw <= 0.0) bad("gains / accel must be > 0");
    if (p_.control_rate_hz < 10.0 || p_.control_rate_hz > 50.0) bad("control_rate_hz must be in [10, 50]");
    if (p_.waypoint_tol <= 0.0 || p_.max_lead <= p_.waypoint_tol) bad("need 0 < waypoint_tolerance < max_lead");
    if (p_.degraded_std_xy <= 0.0 || p_.hold_std_xy <= p_.degraded_std_xy) bad("need 0 < degraded_std_xy_m < hold_std_xy_m");
    if (p_.degraded_kp_factor <= 0.0 || p_.degraded_kp_factor > 1.0) bad("0 < degraded_kp_factor <= 1");
    if (p_.abort_mode != "LAND" && p_.abort_mode != "RTL" && p_.abort_mode != "LOITER" && p_.abort_mode != "BRAKE")
      bad("abort_mode must be one of LAND, RTL, LOITER, BRAKE");
    for (const Vec3& o : offsets_)
      if (o.normXY() > p_.max_horizontal) bad("route corner outside max_horizontal_from_origin_m");
  }

  // ----------------------------------------------------------- interfaces
  void createInterfaces()
  {
    const auto qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>("/mavros/state", qos,
      [this](mavros_msgs::msg::State::ConstSharedPtr m) { vehicle_state_ = *m; });
    est_sub_ = create_subscription<nav_msgs::msg::Odometry>(p_.estimate_topic, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onEstimate(*m); });

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    carrot_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/mission/carrot", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/mission/state", rclcpp::QoS(10).transient_local());
    leg_pub_ = create_publisher<std_msgs::msg::UInt32>("/mission/leg", rclcpp::QoS(10).transient_local());

    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    abort_srv_ = create_service<std_srvs::srv::Trigger>("~/abort",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        abort_requested_ = true;
        res->success = true;
        res->message = "abort requested; mode will be switched to " + p_.abort_mode;
        RCLCPP_WARN(get_logger(), "ABORT requested via service");
      });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / p_.control_rate_hz));
    timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { tick(); });
  }

  void onEstimate(const nav_msgs::msg::Odometry& m)
  {
    const auto& p = m.pose.pose.position;
    const auto& q = m.pose.pose.orientation;
    const auto& c = m.pose.covariance;
    const auto& v = m.twist.twist.linear;     // body frame -> speed is rotation invariant
    const Vec3 pos{p.x, p.y, p.z};
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z) || !std::isfinite(yaw)) return;
    est_pos_ = pos;
    est_yaw_ = yaw;
    est_speed_ = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    est_std_xy_ = std::sqrt(std::max(c[0], c[7]));
    est_std_z_ = std::sqrt(c[14]);
    have_est_ = true;
    est_time_ = now().seconds();
  }

  // ------------------------------------------------------- state plumbing
  double nowS() { return now().seconds(); }
  double secondsInState() { return nowS() - state_started_; }

  void transition(State next)
  {
    RCLCPP_INFO(get_logger(), "State: %s -> %s", stateName(state_), stateName(next));
    state_ = next;
    state_started_ = nowS();
    last_wait_log_ = state_started_;
    stable_valid_ = false;
    publishState();
  }

  void publishState()
  {
    std_msgs::msg::String msg;
    msg.data = stateName(state_);
    state_pub_->publish(msg);
  }

  static bool isStreaming(State s) { return s == State::TAKEOFF || s == State::ROUTE; }
  static bool isTerminal(State s)
  {
    return s == State::DONE || s == State::ABORTED || s == State::STOPPED || s == State::FAULT;
  }
  bool airborne() const { return isStreaming(state_) || state_ == State::LAND; }

  void fault(const std::string& why)
  {
    RCLCPP_ERROR(get_logger(), "FAULT in %s: %s", stateName(state_), why.c_str());
    transition(State::FAULT);
  }
  void beginAbort(const std::string& why)
  {
    RCLCPP_ERROR(get_logger(), "ABORT in %s: %s -> switching to %s", stateName(state_), why.c_str(), p_.abort_mode.c_str());
    if (airborne()) transition(State::ABORTING);
    else transition(State::FAULT);
  }
  /// Do NOT fight the pilot: stop publishing setpoints, change nothing else.
  void stopSilently(const std::string& why)
  {
    RCLCPP_ERROR(get_logger(), "STOPPED in %s: %s. No more setpoints will be sent.", stateName(state_), why.c_str());
    transition(State::STOPPED);
  }
  void logWaiting(const std::string& what)
  {
    if (nowS() - last_wait_log_ < 5.0) return;
    last_wait_log_ = nowS();
    RCLCPP_INFO(get_logger(), "Still waiting in %s: %s", stateName(state_), what.c_str());
  }

  // ---------------------------------------------------- service helpers
  uint64_t beginService()
  {
    service_pending_ = true;
    service_started_ = nowS();
    return ++service_id_;
  }
  bool endService(uint64_t id)
  {
    if (!service_pending_ || id != service_id_) return false;
    service_pending_ = false;
    return true;
  }
  bool canCallService()
  {
    if (service_pending_ || nowS() - last_service_try_ < 1.0) return false;
    last_service_try_ = nowS();
    return true;
  }
  void trySetMode(const std::string& mode)
  {
    if (!mode_client_->service_is_ready() || !canCallService()) return;
    const uint64_t id = beginService();
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    mode_client_->async_send_request(req, [this, id, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture f) {
      if (!endService(id)) return;
      const auto r = f.get();
      if (!r || !r->mode_sent) RCLCPP_ERROR(get_logger(), "MAVROS rejected mode %s, will retry", mode.c_str());
    });
  }
  void tryArm()
  {
    if (!arm_client_->service_is_ready() || !canCallService()) return;
    const uint64_t id = beginService();
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;
    arm_client_->async_send_request(req, [this, id](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f) {
      if (!endService(id)) return;
      const auto r = f.get();
      if (!r || !r->success) RCLCPP_ERROR(get_logger(), "arming rejected (pre-arm checks?), will retry");
    });
  }
  void tryTakeoff()
  {
    if (takeoff_requested_ || !takeoff_client_->service_is_ready() || !canCallService()) return;
    const uint64_t id = beginService();
    takeoff_requested_ = true;
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = static_cast<float>(p_.flight_altitude);   // above the arming point
    req->yaw = std::nanf("");                                  // no yaw request (NED convention there)
    takeoff_client_->async_send_request(req, [this, id](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture f) {
      if (!endService(id)) return;
      const auto r = f.get();
      if (!r || !r->success) {
        RCLCPP_ERROR(get_logger(), "takeoff command rejected, will retry");
        takeoff_requested_ = false;
      } else {
        takeoff_acknowledged_ = true;
        RCLCPP_INFO(get_logger(), "MAVROS accepted takeoff to %.2f m", p_.flight_altitude);
      }
    });
  }

  // ----------------------------------------------------------- control
  bool estimateFresh() { return have_est_ && nowS() - est_time_ <= p_.estimate_timeout; }

  /// Moves the carrot towards `goal` with a trapezoidal speed profile (accel-limited start and
  /// stop, no velocity steps at the corners) and never lets it run more than max_lead ahead of the
  /// ESTIMATED position.  Returns true when the carrot is at the goal and the estimate within
  /// waypoint_tolerance of it.
  bool moveCarrotTo(const Vec3& goal, double dt)
  {
    carrot_vel_ = {};
    const double d = (goal - carrot_).norm();
    if ((est_pos_ - carrot_).norm() >= p_.max_lead) carrot_speed_ = 0.0;   // wait for the drone
    else if (d > 1e-9) {
      // a degraded estimate lowers the speed limit; the carrot brakes towards it with max_accel
      const double v_lim = p_.cruise_speed * quality_;
      carrot_speed_ = std::min({carrot_speed_ + p_.max_accel * dt, std::sqrt(2.0 * p_.max_accel * d),
                                std::max(v_lim, carrot_speed_ - p_.max_accel * dt)});
      const double step = std::min(carrot_speed_ * dt, d);
      const Vec3 u = (goal - carrot_) * (1.0 / d);
      carrot_ = carrot_ + u * step;
      carrot_vel_ = u * (step / dt);
    }
    return (goal - carrot_).norm() < 1e-6 && (est_pos_ - goal).norm() <= p_.waypoint_tol;
  }

  /// Position loop on the estimate -> velocity + yaw-rate setpoint.
  void computeCommand()
  {
    const Vec3 e = carrot_ - est_pos_;
    const double kp_xy = p_.kp_xy * (p_.degraded_kp_factor + (1.0 - p_.degraded_kp_factor) * quality_);
    Vec3 v{carrot_vel_.x + kp_xy * e.x, carrot_vel_.y + kp_xy * e.y, carrot_vel_.z + p_.kp_z * e.z};
    const double vxy = v.normXY();
    if (vxy > p_.max_vel_xy) { v.x *= p_.max_vel_xy / vxy; v.y *= p_.max_vel_xy / vxy; }
    v.z = std::clamp(v.z, -p_.max_vel_z, p_.max_vel_z);
    cmd_vel_ = v;
    const double max_rate = p_.max_yaw_rate_deg / kRad2Deg;
    cmd_yaw_rate_ = std::clamp(p_.kp_yaw * wrapPi(yaw_hold_ - est_yaw_), -max_rate, max_rate);
  }

  void publishSetpoint()
  {
    using PT = mavros_msgs::msg::PositionTarget;
    PT m;
    m.header.stamp = now();
    m.header.frame_id = "map";
    m.coordinate_frame = PT::FRAME_LOCAL_NED;     // MAVROS converts ENU -> NED
    m.type_mask = PT::IGNORE_PX | PT::IGNORE_PY | PT::IGNORE_PZ | PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ |
                  PT::IGNORE_YAW;                 // velocity + yaw rate only
    m.velocity.x = cmd_vel_.x;
    m.velocity.y = cmd_vel_.y;
    m.velocity.z = cmd_vel_.z;
    m.yaw_rate = static_cast<float>(cmd_yaw_rate_);
    setpoint_pub_->publish(m);

    geometry_msgs::msg::PointStamped c;
    c.header = m.header;
    c.point.x = carrot_.x;
    c.point.y = carrot_.y;
    c.point.z = carrot_.z;
    carrot_pub_->publish(c);
  }

  // ------------------------------------------------------ main control tick
  void tick()
  {
    const double t = nowS();
    const double dt = std::clamp(t - last_tick_, 0.001, 0.2);
    last_tick_ = t;
    printStatus();
    if (isTerminal(state_)) return;

    if (service_pending_ && t - service_started_ > p_.service_timeout) {
      RCLCPP_WARN(get_logger(), "MAVROS service reply timed out, will retry");
      service_pending_ = false;
    }
    if (!safetyChecks()) return;

    switch (state_) {
      case State::WAIT_ESTIMATE: tickWaitEstimate(); break;
      case State::SET_MODE:
        if (vehicle_state_.mode == "GUIDED") transition(State::ARM);
        else { trySetMode("GUIDED"); logWaiting("GUIDED mode"); }
        break;
      case State::ARM:
        if (vehicle_state_.armed) transition(State::TAKEOFF);
        else { tryArm(); logWaiting("arming"); }
        break;
      case State::TAKEOFF: tickTakeoff(); break;
      case State::ROUTE: tickRoute(dt); break;
      case State::LAND: tickLand(); break;
      case State::ABORTING: tickAborting(); break;
      default: break;
    }
    // During TAKEOFF ArduPilot climbs on its own; streaming setpoints would cancel the takeoff.
    if (state_ == State::ROUTE) {
      computeCommand();
      publishSetpoint();
    }
  }

  bool safetyChecks()
  {
    if (abort_requested_ && state_ != State::ABORTING) {
      abort_requested_ = false;
      if (airborne()) beginAbort("abort service called");
      else { RCLCPP_WARN(get_logger(), "abort requested on the ground: stopping"); transition(State::FAULT); }
      return false;
    }
    if (origin_valid_ && (airborne() || state_ == State::SET_MODE || state_ == State::ARM) && state_ != State::LAND) {
      if (!estimateFresh()) {
        if (airborne()) beginAbort("estimate stale (no /estimator/odometry for > " + std::to_string(p_.estimate_timeout) + " s)");
        else fault("estimate stale");
        return false;
      }
      if (est_std_xy_ > p_.max_estimate_std) {
        if (airborne()) beginAbort("estimate uncertainty sigma_xy = " + std::to_string(est_std_xy_) + " m too large");
        else fault("estimate uncertainty too large");
        return false;
      }
    }
    if (isStreaming(state_)) {
      if (!vehicle_state_.connected) { stopSilently("MAVROS lost the FCU connection"); return false; }
      if (!vehicle_state_.armed) { stopSilently("vehicle is no longer armed"); return false; }
      if (vehicle_state_.mode != "GUIDED") {
        stopSilently("flight mode is " + vehicle_state_.mode + ", not GUIDED (RC override?) -- not fighting the pilot");
        return false;
      }
      const Vec3 rel = est_pos_ - origin_;
      if (rel.normXY() > p_.max_horizontal) { beginAbort("geofence: " + std::to_string(rel.normXY()) + " m from origin"); return false; }
      if (rel.z > p_.max_altitude) { beginAbort("geofence: altitude " + std::to_string(rel.z) + " m"); return false; }
      if (state_ == State::ROUTE && (est_pos_ - carrot_).norm() > p_.max_tracking_error) {
        beginAbort("tracking error " + std::to_string((est_pos_ - carrot_).norm()) + " m");
        return false;
      }
    }
    double limit = 0.0;
    switch (state_) {
      case State::SET_MODE: case State::ARM: limit = p_.handshake_timeout; break;
      case State::TAKEOFF: limit = p_.takeoff_timeout; break;
      case State::ROUTE: limit = p_.route_timeout; break;
      case State::LAND: case State::ABORTING: limit = p_.landing_timeout; break;
      default: break;
    }
    if (limit > 0.0 && secondsInState() > limit) {
      if (airborne()) beginAbort("state timeout after " + std::to_string(limit) + " s");
      else fault("state timeout after " + std::to_string(limit) + " s");
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------- state ticks
  void tickWaitEstimate()
  {
    if (!vehicle_state_.connected) { logWaiting("MAVROS/FCU not connected"); return; }
    if (!estimateFresh()) { logWaiting("no estimate on " + p_.estimate_topic); return; }
    if (est_std_xy_ > p_.origin_max_std_xy || est_std_z_ > p_.origin_max_std_z) {
      logWaiting("estimate not converged yet: sigma_xy=" + std::to_string(est_std_xy_) + " sigma_z=" +
                 std::to_string(est_std_z_) + " m");
      return;
    }
    origin_ = est_pos_;
    yaw_hold_ = est_yaw_;
    origin_valid_ = true;
    waypoints_.clear();
    for (int lap = 0; lap < p_.laps; ++lap)
      for (const Vec3& o : offsets_) waypoints_.push_back({origin_.x + o.x, origin_.y + o.y, origin_.z + p_.flight_altitude});
    RCLCPP_INFO(get_logger(),
      "ORIGIN = first valid estimate: (%.2f, %.2f, %.2f) m, yaw %.1f deg (sigma_xy %.2f m, sigma_z %.2f m); "
      "%zu waypoints, yaw held at the origin yaw", origin_.x, origin_.y, origin_.z, yaw_hold_ * kRad2Deg,
      est_std_xy_, est_std_z_, waypoints_.size());
    transition(State::SET_MODE);
  }

  void tickTakeoff()
  {
    if (!takeoff_acknowledged_) { tryTakeoff(); return; }
    const double climb = est_pos_.z - origin_.z;
    const bool at_alt = std::fabs(climb - p_.flight_altitude) <= p_.takeoff_tol;
    const bool calm = est_speed_ <= p_.stabilize_speed;
    if (!(at_alt && calm)) { stable_valid_ = false; logWaiting("climb " + std::to_string(climb) + " m"); return; }
    if (!stable_valid_) { stable_since_ = nowS(); stable_valid_ = true; return; }
    if (nowS() - stable_since_ < p_.stabilize_time) return;
    RCLCPP_INFO(get_logger(), "Takeoff confirmed by the estimate: climb %.2f m (target %.2f), speed %.2f m/s",
      climb, p_.flight_altitude, est_speed_);
    carrot_ = {origin_.x, origin_.y, origin_.z + p_.flight_altitude};   // start of the route, above the origin
    carrot_speed_ = 0.0;
    wp_index_ = 0;
    publishLeg();
    transition(State::ROUTE);
  }

  /// Estimate quality from sigma_xy: 1 = normal, 0 = hold.  Logged once per transition.
  void updateQuality()
  {
    quality_ = std::clamp((p_.hold_std_xy - est_std_xy_) / (p_.hold_std_xy - p_.degraded_std_xy), 0.0, 1.0);
    const int mode = quality_ >= 1.0 ? 0 : (quality_ > 0.0 ? 1 : 2);
    // hysteresis: back to a better mode only 0.02 m below its threshold
    const bool better = mode < quality_mode_ &&
      est_std_xy_ < (quality_mode_ == 2 ? p_.hold_std_xy : p_.degraded_std_xy) - 0.02;
    if (mode > quality_mode_ || better) {
      static const char* names[] = {"NORMAL (full speed)", "DEGRADED (slowing down)", "HOLD (carrot stopped)"};
      RCLCPP_WARN(get_logger(), "Estimate sigma_xy = %.2f m -> route %s", est_std_xy_, names[mode]);
      quality_mode_ = mode;
    }
  }

  void tickRoute(double dt)
  {
    updateQuality();
    const Vec3& goal = waypoints_[wp_index_];
    if (!moveCarrotTo(goal, dt)) return;
    const size_t per_lap = offsets_.size();
    RCLCPP_INFO(get_logger(), "Lap %zu/%d: reached corner %zu/%zu, offset (%.2f, %.2f) [estimate (%.2f, %.2f, %.2f)]",
      wp_index_ / per_lap + 1, p_.laps, wp_index_ % per_lap + 1, per_lap, goal.x - origin_.x, goal.y - origin_.y,
      est_pos_.x, est_pos_.y, est_pos_.z);
    carrot_speed_ = 0.0;
    if (++wp_index_ >= waypoints_.size()) {
      RCLCPP_INFO(get_logger(), "Both laps done, back above the origin -> LAND");
      transition(State::LAND);
      return;
    }
    publishLeg();
  }

  void publishLeg()
  {
    std_msgs::msg::UInt32 m;
    m.data = static_cast<uint32_t>(wp_index_);
    leg_pub_->publish(m);
  }

  bool pilotTookOver(const std::string& our_mode)
  {
    if (vehicle_state_.mode == "GUIDED" || vehicle_state_.mode == our_mode) return false;
    stopSilently("flight mode changed to " + vehicle_state_.mode + " (RC override?) -- not fighting the pilot");
    return true;
  }

  void tickLand()
  {
    if (pilotTookOver("LAND")) return;
    if (vehicle_state_.mode != "LAND") { trySetMode("LAND"); return; }
    if (!vehicle_state_.armed) {
      RCLCPP_INFO(get_logger(), "Landed and disarmed at estimate (%.2f, %.2f) = offset (%.2f, %.2f) from the origin. "
        "Mission complete.", est_pos_.x, est_pos_.y, est_pos_.x - origin_.x, est_pos_.y - origin_.y);
      transition(State::DONE);
    } else logWaiting("touchdown / disarm");
  }

  void tickAborting()
  {
    if (pilotTookOver(p_.abort_mode)) return;
    if (vehicle_state_.mode == p_.abort_mode) {
      RCLCPP_WARN(get_logger(), "Vehicle is in %s. Mission node is now passive.", p_.abort_mode.c_str());
      transition(State::ABORTED);
      return;
    }
    trySetMode(p_.abort_mode);
    logWaiting("abort mode change (take over with the RC if this persists!)");
  }

  void printStatus()
  {
    const double t = nowS();
    if (t - last_status_ < p_.status_period || !have_est_) return;
    last_status_ = t;
    if (!origin_valid_) return;
    const Vec3 rel = est_pos_ - origin_;
    if (state_ != State::ROUTE) {
      RCLCPP_INFO(get_logger(), "[%s] estimate rel. origin=(%.2f, %.2f, %.2f) yaw=%.1f deg sigma_xy=%.2f",
        stateName(state_), rel.x, rel.y, rel.z, est_yaw_ * kRad2Deg, est_std_xy_);
      return;
    }
    RCLCPP_INFO(get_logger(),
      "[ROUTE] estimate rel. origin=(%.2f, %.2f, %.2f) yaw=%.1f deg sigma_xy=%.2f | carrot err=%.2f m | "
      "cmd v=(%.2f, %.2f, %.2f) yaw_rate=%.1f deg/s", rel.x, rel.y, rel.z, est_yaw_ * kRad2Deg,
      est_std_xy_, (carrot_ - est_pos_).norm(), cmd_vel_.x, cmd_vel_.y, cmd_vel_.z, cmd_yaw_rate_ * kRad2Deg);
  }

  // ------------------------------------------------------------- members
  Params p_;
  std::vector<Vec3> offsets_, waypoints_;
  State state_ = State::WAIT_ESTIMATE;
  double state_started_ = 0.0, last_wait_log_ = 0.0, last_tick_ = 0.0, last_status_ = 0.0;
  double stable_since_ = 0.0, service_started_ = 0.0, last_service_try_ = 0.0;
  bool stable_valid_ = false;

  mavros_msgs::msg::State vehicle_state_;
  bool have_est_ = false;
  Vec3 est_pos_;
  double est_yaw_ = 0.0, est_speed_ = 0.0, est_std_xy_ = 1e9, est_std_z_ = 1e9, est_time_ = 0.0;

  bool service_pending_ = false, takeoff_requested_ = false, takeoff_acknowledged_ = false, abort_requested_ = false;
  uint64_t service_id_ = 0;

  bool origin_valid_ = false;
  Vec3 origin_, carrot_, carrot_vel_, cmd_vel_;
  double yaw_hold_ = 0.0, carrot_speed_ = 0.0, cmd_yaw_rate_ = 0.0;
  double quality_ = 1.0;
  int quality_mode_ = 0;
  size_t wp_index_ = 0;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr est_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr leg_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LocalizationMissionNode>());   // single-threaded on purpose
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_mission"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
