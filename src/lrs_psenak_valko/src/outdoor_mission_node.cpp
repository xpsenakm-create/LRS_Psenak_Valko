// outdoor_mission_node.cpp -- A1.3 outdoor GPS mission.
//
//   WAIT_GPS -> SET_MODE -> ARM -> TAKEOFF -> TO_START -> SQUARE -> TO_CENTER
//            -> FIGURE8 -> RETURN -> LAND -> DONE
//   any flight state --(abort service / geofence / tracking error)--> ABORTING -> ABORTED
//   any flight state --(RC mode change / MAVROS drop-out)-----------> STOPPED (no setpoints!)
//
// Everything is RELATIVE to the launch point captured before arming (first stable
// GPS fix paired with the local pose at that instant) -- no absolute coordinates.
//
// Frames / angles (see outdoor_math.h):  local ENU, yaw 0 = East, CCW positive.
// Altitude: we COMMAND local z = z_launch + flight_altitude and CHECK both
// (local z - z_launch) and /mavros/global_position/rel_alt.
//
// Single-threaded executor on purpose: all callbacks and the control timer run in
// one thread, so there are no data races and no mutexes to explain.

#include "outdoor_math.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
using outdoor::Vec2;
using outdoor::Vec3;
using SteadyClock = std::chrono::steady_clock;

class OutdoorMissionNode final : public rclcpp::Node
{
public:
  OutdoorMissionNode() : Node("outdoor_mission_node")
  {
    declareParameters();
    validateParameters();
    checkMissionEnvelope();
    createInterfaces();
    openLog();

    RCLCPP_INFO(get_logger(),
      "Parameters: altitude=%.2f m, square=%.2f m, figure-8 A=%.2f m omega=%.3f rad/s (lap %.1f s, "
      "peak speed %.2f m/s), POI offset from launch=(E %.2f, N %.2f) m",
      p_.flight_altitude, p_.square_side, p_.amplitude, p_.omega, 2.0 * outdoor::kPi / p_.omega,
      outdoor::figure8PeakSpeed(p_.amplitude, p_.omega), p_.poi_east, p_.poi_north);
    transition(State::WAIT_GPS);
  }

private:
  // ------------------------------------------------------------------ types
  enum class State
  {
    WAIT_GPS, SET_MODE, ARM, TAKEOFF, TO_START, SQUARE, TO_CENTER, FIGURE8, RETURN, LAND,
    DONE, ABORTING, ABORTED, STOPPED, FAULT
  };

  static const char* stateName(State s)
  {
    switch (s) {
      case State::WAIT_GPS: return "WAIT_GPS";
      case State::SET_MODE: return "SET_MODE";
      case State::ARM: return "ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::TO_START: return "TO_START";
      case State::SQUARE: return "SQUARE";
      case State::TO_CENTER: return "TO_CENTER";
      case State::FIGURE8: return "FIGURE8";
      case State::RETURN: return "RETURN";
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
    // mission geometry (fixed by the assignment; exposed so they are documented, not buried)
    double flight_altitude = 5.0, square_side = 5.0, amplitude = 3.0;
    double omega = 0.30, laps = 1.0, ramp_s = 3.0;
    double poi_east = 0.0, poi_north = 0.0;          // POI offset from the LAUNCH point [m]
    bool close_square = true;
    // motion
    double control_rate_hz = 30.0, max_speed = 1.5, square_speed = 0.5;
    double waypoint_tol = 0.3, max_lead = 1.0, yaw_rate_limit_deg = 45.0;
    // takeoff
    double takeoff_tol = 0.2, stabilize_time = 2.0, stabilize_speed = 0.15;
    // GPS gate
    int min_satellites = 8;
    double max_h_std = 2.0, fix_stable_s = 3.0;
    // safety
    double max_horizontal = 4.0, max_altitude = 5.5, max_tracking_error = 2.0;
    std::string abort_mode = "LAND";
    // timeouts
    double handshake_timeout = 60.0, takeoff_timeout = 40.0, segment_timeout = 90.0;
    double landing_timeout = 90.0, service_timeout = 5.0, telemetry_timeout = 1.0;
    // output
    std::string log_csv;
    double status_period = 1.0;
  };

  // ----------------------------------------------------------- parameters
  void declareParameters()
  {
    p_.flight_altitude = declare_parameter<double>("flight_altitude_m", 5.0);
    p_.square_side = declare_parameter<double>("square_side_m", 5.0);
    p_.amplitude = declare_parameter<double>("figure8_amplitude_m", 3.0);
    p_.omega = declare_parameter<double>("figure8_omega_rad_s", 0.30);
    p_.laps = declare_parameter<double>("figure8_laps", 1.0);
    p_.ramp_s = declare_parameter<double>("figure8_ramp_s", 3.0);
    p_.poi_east = declare_parameter<double>("poi_east_m", 0.0);
    p_.poi_north = declare_parameter<double>("poi_north_m", 0.0);
    p_.close_square = declare_parameter<bool>("close_square", true);

    p_.control_rate_hz = declare_parameter<double>("control_rate_hz", 30.0);
    p_.max_speed = declare_parameter<double>("max_speed_mps", 1.5);
    p_.square_speed = declare_parameter<double>("square_speed_mps", 0.5);
    p_.waypoint_tol = declare_parameter<double>("waypoint_tolerance_m", 0.3);
    p_.max_lead = declare_parameter<double>("max_lead_m", 1.0);
    p_.yaw_rate_limit_deg = declare_parameter<double>("yaw_rate_limit_deg_s", 45.0);

    p_.takeoff_tol = declare_parameter<double>("takeoff_tolerance_m", 0.2);
    p_.stabilize_time = declare_parameter<double>("stabilize_time_s", 2.0);
    p_.stabilize_speed = declare_parameter<double>("stabilize_speed_mps", 0.15);

    p_.min_satellites = static_cast<int>(declare_parameter<int64_t>("min_satellites", 8));
    p_.max_h_std = declare_parameter<double>("max_horizontal_std_m", 2.0);
    p_.fix_stable_s = declare_parameter<double>("fix_stable_s", 3.0);

    p_.max_horizontal = declare_parameter<double>("max_horizontal_from_launch_m", 4.0);
    p_.max_altitude = declare_parameter<double>("max_altitude_m", 5.5);
    p_.max_tracking_error = declare_parameter<double>("max_tracking_error_m", 2.0);
    p_.abort_mode = declare_parameter<std::string>("abort_mode", "LAND");

    p_.handshake_timeout = declare_parameter<double>("handshake_timeout_s", 60.0);
    p_.takeoff_timeout = declare_parameter<double>("takeoff_timeout_s", 40.0);
    p_.segment_timeout = declare_parameter<double>("segment_timeout_s", 90.0);
    p_.landing_timeout = declare_parameter<double>("landing_timeout_s", 90.0);
    p_.service_timeout = declare_parameter<double>("service_timeout_s", 5.0);
    p_.telemetry_timeout = declare_parameter<double>("telemetry_timeout_s", 1.0);

    p_.log_csv = declare_parameter<std::string>("log_csv", "outdoor_mission_log.csv");
    p_.status_period = declare_parameter<double>("status_period_s", 1.0);
  }

  void validateParameters() const
  {
    auto bad = [](const std::string& m) { throw std::invalid_argument("invalid parameter: " + m); };
    if (p_.flight_altitude <= 0.5) bad("flight_altitude_m must be > 0.5");
    if (p_.square_side <= 0.0 || p_.amplitude <= 0.0) bad("square_side_m and figure8_amplitude_m must be > 0");
    if (p_.omega <= 0.0 || p_.laps <= 0.0 || p_.ramp_s < 0.0) bad("figure8_omega/laps must be > 0, ramp >= 0");
    if (p_.control_rate_hz < 10.0 || p_.control_rate_hz > 50.0) bad("control_rate_hz must be in [10, 50]");
    if (p_.max_speed <= 0.0 || p_.square_speed <= 0.0 || p_.square_speed > p_.max_speed)
      bad("0 < square_speed_mps <= max_speed_mps");
    if (p_.waypoint_tol <= 0.0 || p_.max_lead <= p_.waypoint_tol) bad("need 0 < waypoint_tolerance < max_lead");
    if (p_.yaw_rate_limit_deg <= 0.0) bad("yaw_rate_limit_deg_s must be > 0");
    if (p_.takeoff_tol <= 0.0 || p_.stabilize_time <= 0.0 || p_.stabilize_speed <= 0.0) bad("takeoff/stabilize values");
    if (p_.min_satellites < 0 || p_.max_h_std <= 0.0 || p_.fix_stable_s < 0.0) bad("GPS gate values");
    if (p_.max_horizontal <= 0.0 || p_.max_altitude <= 0.0 || p_.max_tracking_error <= 0.0) bad("safety limits");
    if (p_.abort_mode != "LAND" && p_.abort_mode != "RTL" && p_.abort_mode != "LOITER" && p_.abort_mode != "BRAKE")
      bad("abort_mode must be one of LAND, RTL, LOITER, BRAKE");
    const double peak = outdoor::figure8PeakSpeed(p_.amplitude, p_.omega);
    if (peak > p_.max_speed)
      bad("figure8_omega_rad_s=" + std::to_string(p_.omega) + " gives peak speed " + std::to_string(peak) +
          " m/s > max_speed_mps; maximum omega is " +
          std::to_string(outdoor::figure8MaxOmega(p_.amplitude, p_.max_speed)));
  }

  /// Pre-flight geofence arithmetic: done in the constructor, i.e. BEFORE anything can arm.
  void checkMissionEnvelope() const
  {
    const double r = outdoor::missionMaxRadius({p_.poi_east, p_.poi_north}, p_.square_side, p_.amplitude);
    RCLCPP_INFO(get_logger(),
      "Mission envelope: max horizontal distance from launch = %.2f m (limit %.2f m), "
      "max altitude = %.2f m (limit %.2f m)", r, p_.max_horizontal, p_.flight_altitude, p_.max_altitude);
    if (r > p_.max_horizontal)
      throw std::invalid_argument("mission reaches " + std::to_string(r) + " m from launch, limit is " +
                                  std::to_string(p_.max_horizontal) + " m (move the POI or raise the limit)");
    if (p_.flight_altitude > p_.max_altitude)
      throw std::invalid_argument("flight_altitude_m exceeds max_altitude_m");
  }

  // ----------------------------------------------------------- interfaces
  void createInterfaces()
  {
    // Best-effort sensor QoS: compatible with both reliable and best-effort MAVROS publishers.
    const auto qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>("/mavros/state", qos,
      [this](mavros_msgs::msg::State::ConstSharedPtr m) { vehicle_state_ = *m; });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        pose_ = *m; have_pose_ = true; pose_time_ = SteadyClock::now();
      });
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/mavros/local_position/velocity_local", qos,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { vel_ = *m; have_vel_ = true; vel_time_ = SteadyClock::now(); });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) { fix_ = *m; have_fix_ = true; fix_time_ = SteadyClock::now(); });
    sats_sub_ = create_subscription<std_msgs::msg::UInt32>("/mavros/global_position/raw/satellites", qos,
      [this](std_msgs::msg::UInt32::ConstSharedPtr m) { satellites_ = m->data; have_sats_ = true; });
    hdg_sub_ = create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr m) { compass_hdg_deg_ = m->data; have_hdg_ = true; });
    rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>("/mavros/global_position/rel_alt", qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr m) { rel_alt_ = m->data; have_rel_alt_ = true; });
    home_sub_ = create_subscription<mavros_msgs::msg::HomePosition>("/mavros/home_position/home", qos,
      [this](mavros_msgs::msg::HomePosition::ConstSharedPtr m) { home_ = *m; have_home_ = true; });

    setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
    intended_path_pub_ = create_publisher<nav_msgs::msg::Path>("mission/intended_path", rclcpp::QoS(1).transient_local());
    flown_path_pub_ = create_publisher<nav_msgs::msg::Path>("mission/flown_path", 10);

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
    timer_ = create_wall_timer(period, [this]() { tick(); });
  }

  void openLog()
  {
    if (p_.log_csv.empty()) return;
    log_.open(p_.log_csv);
    if (!log_) { RCLCPP_WARN(get_logger(), "cannot open %s, CSV log disabled", p_.log_csv.c_str()); return; }
    log_ << "t_s,state,cmd_x,cmd_y,cmd_z,cmd_yaw_deg,x,y,z,yaw_deg,yaw_err_deg,poi_bearing_err_deg,"
            "track_err_m,rel_alt,compass_hdg_deg\n";
    log_start_ = SteadyClock::now();
  }

  // ------------------------------------------------------- state plumbing
  void transition(State next)
  {
    RCLCPP_INFO(get_logger(), "State: %s -> %s", stateName(state_), stateName(next));
    state_ = next;
    state_started_ = SteadyClock::now();
    last_wait_log_ = state_started_;
    stable_since_valid_ = false;
    if (next == State::FIGURE8) f8_started_ = state_started_;
  }

  static bool isStreaming(State s)
  {
    return s == State::TAKEOFF || s == State::TO_START || s == State::SQUARE || s == State::TO_CENTER ||
           s == State::FIGURE8 || s == State::RETURN;
  }
  static bool isTerminal(State s)
  {
    return s == State::DONE || s == State::ABORTED || s == State::STOPPED || s == State::FAULT;
  }
  bool airborne() const { return isStreaming(state_) || state_ == State::LAND; }

  double secondsIn(State) const
  {
    return std::chrono::duration<double>(SteadyClock::now() - state_started_).count();
  }

  void fault(const std::string& why)
  {
    RCLCPP_ERROR(get_logger(), "FAULT in %s: %s", stateName(state_), why.c_str());
    transition(State::FAULT);
  }

  /// Leave the mission and put the vehicle in a safe mode (airborne), or just stop (on ground).
  void beginAbort(const std::string& why)
  {
    RCLCPP_ERROR(get_logger(), "ABORT in %s: %s -> switching to %s", stateName(state_), why.c_str(),
      p_.abort_mode.c_str());
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
    const auto t = SteadyClock::now();
    if (std::chrono::duration<double>(t - last_wait_log_).count() < 5.0) return;
    last_wait_log_ = t;
    RCLCPP_INFO(get_logger(), "Still waiting in %s: %s", stateName(state_), what.c_str());
  }

  // ---------------------------------------------------- service helpers
  uint64_t beginService()
  {
    service_pending_ = true;
    service_started_ = SteadyClock::now();
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
    const auto t = SteadyClock::now();
    if (service_pending_) return false;
    if (std::chrono::duration<double>(t - last_service_try_).count() < 1.0) return false;
    last_service_try_ = t;
    return true;
  }

  void trySetMode(const std::string& mode)
  {
    if (!mode_client_->service_is_ready() || !canCallService()) return;
    const uint64_t id = beginService();
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    mode_client_->async_send_request(req,
      [this, id, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture f) {
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
    arm_client_->async_send_request(req,
      [this, id](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture f) {
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
    req->altitude = static_cast<float>(p_.flight_altitude);   // metres ABOVE the launch point
    if (origin_valid_ && have_fix_) {
      req->latitude = static_cast<float>(fix_.latitude);
      req->longitude = static_cast<float>(fix_.longitude);
    }
    req->yaw = static_cast<float>(poseYaw() * outdoor::kRad2Deg);
    takeoff_client_->async_send_request(req,
      [this, id](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture f) {
        if (!endService(id)) return;
        const auto r = f.get();
        if (!r || !r->success) {
          RCLCPP_ERROR(get_logger(), "takeoff command rejected, will retry");
          takeoff_requested_ = false;
        } else {
          takeoff_acknowledged_ = true;
          RCLCPP_INFO(get_logger(), "MAVROS accepted takeoff to %.2f m above launch", p_.flight_altitude);
        }
      });
  }

  // ----------------------------------------------------- telemetry views
  double poseYaw() const
  {
    const auto& q = pose_.pose.orientation;
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  Vec3 posePos() const { return {pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z}; }
  double speed() const
  {
    const auto& v = vel_.twist.linear;
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }
  bool fresh(SteadyClock::time_point t, double max_age_s) const
  {
    return std::chrono::duration<double>(SteadyClock::now() - t).count() <= max_age_s;
  }

  /// "Usable fix": status >= FIX, finite lat/lon, enough satellites, small enough covariance.
  bool fixGood(std::string* why = nullptr) const
  {
    auto no = [&](const std::string& s) { if (why) *why = s; return false; };
    if (!have_fix_ || !fresh(fix_time_, 2.0)) return no("no fresh NavSatFix");
    if (fix_.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
      return no("fix status " + std::to_string(fix_.status.status) + " (NO_FIX)");
    if (!std::isfinite(fix_.latitude) || !std::isfinite(fix_.longitude)) return no("lat/lon not finite");
    if (p_.min_satellites > 0) {
      if (!have_sats_) return no("no satellite count received");
      if (static_cast<int>(satellites_) < p_.min_satellites)
        return no("only " + std::to_string(satellites_) + " satellites (need " + std::to_string(p_.min_satellites) + ")");
    }
    if (fix_.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double h_std = std::sqrt(std::max(fix_.position_covariance[0], fix_.position_covariance[4]));
      if (h_std > p_.max_h_std) return no("horizontal std " + std::to_string(h_std) + " m too large");
    }
    return true;
  }

  // ---------------------------------------------------------- the origin
  void captureOrigin()
  {
    ltp_ = outdoor::LocalTangentPlane(fix_.latitude, fix_.longitude, fix_.altitude);
    launch_ = posePos();                 // local pose at the same instant as the fix
    cmd_yaw_ = poseYaw();
    origin_valid_ = true;

    poi_ = {launch_.x + p_.poi_east, launch_.y + p_.poi_north};
    corners_ = outdoor::squareCorners(poi_, p_.square_side);
    z_fly_ = launch_.z + p_.flight_altitude;

    sq_.clear();
    for (const Vec2& c : corners_) sq_.push_back(c);
    if (p_.close_square) sq_.push_back(corners_[0]);

    RCLCPP_INFO(get_logger(), "ORIGIN (launch point): lat=%.7f lon=%.7f alt(AMSL)=%.2f m | local ENU=(%.2f, %.2f, %.2f) | yaw=%.1f deg",
      fix_.latitude, fix_.longitude, fix_.altitude, launch_.x, launch_.y, launch_.z, poseYaw() * outdoor::kRad2Deg);
    if (have_home_)
      RCLCPP_INFO(get_logger(), "MAVROS home: lat=%.7f lon=%.7f alt=%.2f (differs from origin by %.2f m horizontally)",
        home_.geo.latitude, home_.geo.longitude, home_.geo.altitude,
        std::hypot(ltp_.toEnu({home_.geo.latitude, home_.geo.longitude, home_.geo.altitude}).e,
                   ltp_.toEnu({home_.geo.latitude, home_.geo.longitude, home_.geo.altitude}).n));
    reportGeodeticPlan();
    publishIntendedPath();
  }

  Vec3 toLocal(const outdoor::Geodetic& g) const
  {
    const outdoor::Enu e = ltp_.toEnu(g);
    return {launch_.x + e.e, launch_.y + e.n, launch_.z + e.u};
  }
  outdoor::Geodetic toGeodetic(const Vec3& local) const
  {
    return ltp_.toGeodetic({local.x - launch_.x, local.y - launch_.y, local.z - launch_.z});
  }

  /// Both conversion directions exercised at startup: plan in local -> report as lat/lon -> back.
  void reportGeodeticPlan()
  {
    auto line = [&](const char* name, const Vec2& p) {
      const outdoor::Geodetic g = toGeodetic({p.x, p.y, z_fly_});
      const Vec3 back = toLocal(g);
      RCLCPP_INFO(get_logger(), "  %-3s local=(%.2f, %.2f)  lat=%.7f lon=%.7f  round-trip error=%.2e m",
        name, p.x, p.y, g.lat_deg, g.lon_deg, std::hypot(back.x - p.x, back.y - p.y));
    };
    RCLCPP_INFO(get_logger(), "Planned geometry (flat-Earth tangent plane at the launch point):");
    line("POI", poi_);
    line("W1", corners_[0]); line("W2", corners_[1]); line("W3", corners_[2]); line("W4", corners_[3]);
  }

  void publishIntendedPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    auto add = [&](double x, double y, double z) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = x; ps.pose.position.y = y; ps.pose.position.z = z;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    };
    for (const Vec2& c : sq_) add(c.x, c.y, z_fly_);
    for (int i = 0; i <= 200; ++i) {
      const auto s = outdoor::figure8(p_.amplitude, 2.0 * outdoor::kPi * i / 200.0);
      add(poi_.x + s.x, poi_.y + s.y, z_fly_);
    }
    intended_path_pub_->publish(path);
  }

  // ----------------------------------------------------------- motion
  /// Moves the "carrot" (the position we command) towards `goal` at `speed`, but never lets it
  /// run more than max_lead ahead of the real drone.  Returns true when the carrot is at the
  /// goal AND the drone is within waypoint_tol of it.
  bool flyCarrotTo(const Vec3& goal, double speed, double dt)
  {
    const Vec3 pos = posePos();
    if (outdoor::dist3(pos, carrot_) < p_.max_lead) {
      const double d = outdoor::dist3(carrot_, goal);
      if (d > 1e-9) {
        const double step = std::min(speed * dt, d);
        carrot_.x += (goal.x - carrot_.x) / d * step;
        carrot_.y += (goal.y - carrot_.y) / d * step;
        carrot_.z += (goal.z - carrot_.z) / d * step;
      }
    }
    cmd_ = carrot_;
    return outdoor::dist3(carrot_, goal) < 1e-6 && outdoor::dist3(pos, goal) <= p_.waypoint_tol;
  }

  void slewYawTo(double target, double dt)
  {
    cmd_yaw_ = outdoor::slewAngle(cmd_yaw_, target, p_.yaw_rate_limit_deg * outdoor::kDeg2Rad * dt);
  }

  /// Yaw continuously aimed at the POI, recomputed every tick from the COMMANDED position.
  /// (Commanded rather than measured: smooth and deterministic; the measured error is logged.)
  /// Right at the POI the bearing is undefined, so the last yaw is held there.
  void updateYawToPoi(double dt)
  {
    const Vec2 c{carrot_.x, carrot_.y};
    if (outdoor::dist2(c, poi_) > 0.5) slewYawTo(outdoor::bearingYaw(c, poi_), dt);
  }

  void publishSetpoint()
  {
    mavros_msgs::msg::PositionTarget m;
    m.header.stamp = now();
    m.header.frame_id = "map";
    m.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;   // MAVROS converts ENU->NED
    m.type_mask = static_cast<uint16_t>(
      mavros_msgs::msg::PositionTarget::IGNORE_VX | mavros_msgs::msg::PositionTarget::IGNORE_VY |
      mavros_msgs::msg::PositionTarget::IGNORE_VZ | mavros_msgs::msg::PositionTarget::IGNORE_AFX |
      mavros_msgs::msg::PositionTarget::IGNORE_AFY | mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
      mavros_msgs::msg::PositionTarget::FORCE | mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE);
    m.position.x = cmd_.x; m.position.y = cmd_.y; m.position.z = cmd_.z;
    m.yaw = static_cast<float>(cmd_yaw_);                                      // ENU yaw, kept continuous
    setpoint_pub_->publish(m);
  }

  // ------------------------------------------------------ main control tick
  void tick()
  {
    const auto t_now = SteadyClock::now();
    const double dt = std::clamp(std::chrono::duration<double>(t_now - last_tick_).count(), 0.001, 0.2);
    last_tick_ = t_now;

    printStatus();
    if (isTerminal(state_)) return;                       // nothing is published any more

    // ---- service timeout (a lost reply must not wedge the node) ----
    if (service_pending_ &&
        std::chrono::duration<double>(t_now - service_started_).count() > p_.service_timeout) {
      RCLCPP_WARN(get_logger(), "MAVROS service reply timed out, will retry");
      service_pending_ = false;
    }

    if (!safetyChecks()) return;

    switch (state_) {
      case State::WAIT_GPS: tickWaitGps(); break;
      case State::SET_MODE:
        if (vehicle_state_.mode == "GUIDED") transition(State::ARM);
        else { trySetMode("GUIDED"); logWaiting("GUIDED mode"); }
        break;
      case State::ARM:
        if (vehicle_state_.armed) transition(State::TAKEOFF);
        else { tryArm(); logWaiting("arming"); }
        break;
      case State::TAKEOFF: tickTakeoff(dt); break;
      case State::TO_START: tickToStart(dt); break;
      case State::SQUARE: tickSquare(dt); break;
      case State::TO_CENTER: tickToCenter(dt); break;
      case State::FIGURE8: tickFigure8(dt); break;
      case State::RETURN: tickReturn(dt); break;
      case State::LAND: tickLand(); break;
      case State::ABORTING: tickAborting(); break;
      default: break;
    }

    // During TAKEOFF do NOT publish position targets: ArduPilot's Guided Takeoff
    // handles the ascent autonomously. Streaming setpoints cancels takeoff.
    if (isStreaming(state_)) {
      if (state_ != State::TAKEOFF) {
        publishSetpoint();
      }
      appendFlownPath();
    }
    writeLogRow();
  }

  /// Returns false if the tick must stop here (state already changed).
  bool safetyChecks()
  {
    if (origin_valid_ && !fresh(pose_time_, p_.telemetry_timeout) &&
        (airborne() || state_ == State::SET_MODE || state_ == State::ARM)) {
      if (airborne()) stopSilently("local position telemetry stale (MAVROS dropped out?)");
      else fault("local position telemetry stale");
      return false;
    }
    if (abort_requested_ && state_ != State::ABORTING) {
      abort_requested_ = false;
      if (airborne()) beginAbort("abort service called");
      else { RCLCPP_WARN(get_logger(), "abort requested while on the ground: stopping"); transition(State::FAULT); }
      return false;
    }
    if (isStreaming(state_)) {
      // The safety pilot / link state always wins: notice and go quiet.
      if (!vehicle_state_.connected) { stopSilently("MAVROS lost the FCU connection"); return false; }
      if (!vehicle_state_.armed) { stopSilently("vehicle is no longer armed"); return false; }
      if (vehicle_state_.mode != "GUIDED") {
        stopSilently("flight mode is " + vehicle_state_.mode + ", not GUIDED (RC override?) -- not fighting the pilot");
        return false;
      }
      // Our own fence, inside the vehicle's: horizontal radius and altitude relative to launch.
      const Vec3 p = posePos();
      const double h = std::hypot(p.x - launch_.x, p.y - launch_.y);
      const double up = p.z - launch_.z;
      if (h > p_.max_horizontal) { beginAbort("geofence: " + std::to_string(h) + " m from launch"); return false; }
      if (up > p_.max_altitude) { beginAbort("geofence: altitude " + std::to_string(up) + " m"); return false; }
      if (state_ != State::TAKEOFF &&
          outdoor::dist3(p, cmd_) > p_.max_tracking_error) {
        beginAbort("tracking error " + std::to_string(outdoor::dist3(p, cmd_)) + " m");
        return false;
      }
    }
    // Per-state timeouts
    const double in_state = secondsIn(state_);
    double limit = 0.0;
    switch (state_) {
      case State::SET_MODE: case State::ARM: limit = p_.handshake_timeout; break;
      case State::TAKEOFF: limit = p_.takeoff_timeout; break;
      case State::TO_START: case State::SQUARE: case State::TO_CENTER: case State::RETURN:
        limit = p_.segment_timeout; break;
      case State::FIGURE8:
        limit = p_.laps * 2.0 * outdoor::kPi / p_.omega + p_.ramp_s + 30.0; break;
      case State::LAND: case State::ABORTING: limit = p_.landing_timeout; break;
      default: break;
    }
    if (limit > 0.0 && in_state > limit) {
      if (airborne()) beginAbort("state timeout after " + std::to_string(limit) + " s");
      else fault("state timeout after " + std::to_string(limit) + " s");
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------- state ticks
  void tickWaitGps()
  {
    std::string why;
    if (!vehicle_state_.connected) { stable_since_valid_ = false; logWaiting("MAVROS/FCU not connected"); return; }
    if (!have_pose_) { logWaiting("no local pose yet"); return; }
    if (!fixGood(&why)) { stable_since_valid_ = false; logWaiting("GPS not usable: " + why); return; }
    if (!stable_since_valid_) { stable_since_ = SteadyClock::now(); stable_since_valid_ = true; return; }
    if (std::chrono::duration<double>(SteadyClock::now() - stable_since_).count() < p_.fix_stable_s) return;
    RCLCPP_INFO(get_logger(), "GPS fix usable and stable for %.1f s (%u satellites)", p_.fix_stable_s, satellites_);
    captureOrigin();
    transition(State::SET_MODE);
  }

  void tickTakeoff(double)
  {
    cmd_ = {launch_.x, launch_.y, z_fly_};        // consistent with the takeoff command; yaw held
    if (!takeoff_acknowledged_) { tryTakeoff(); return; }
    const double climb = posePos().z - launch_.z;
    const bool at_alt = std::fabs(climb - p_.flight_altitude) <= p_.takeoff_tol;
    const bool calm = have_vel_ && fresh(vel_time_, 0.5) && speed() <= p_.stabilize_speed;
    if (!(at_alt && calm)) { stable_since_valid_ = false; return; }
    if (!stable_since_valid_) { stable_since_ = SteadyClock::now(); stable_since_valid_ = true; return; }
    if (std::chrono::duration<double>(SteadyClock::now() - stable_since_).count() < p_.stabilize_time) return;
    RCLCPP_INFO(get_logger(), "Takeoff confirmed: local climb=%.2f m, rel_alt=%.2f m (target %.2f m), speed=%.2f m/s",
      climb, have_rel_alt_ ? rel_alt_ : std::nan(""), p_.flight_altitude, speed());
    carrot_ = {posePos().x, posePos().y, z_fly_};
    transition(State::TO_START);
  }

  void tickToStart(double dt)
  {
    updateYawToPoi(dt);
    if (flyCarrotTo({corners_[0].x, corners_[0].y, z_fly_}, p_.square_speed, dt)) {
      square_index_ = 1;
      RCLCPP_INFO(get_logger(), "At W1");
      transition(State::SQUARE);
    }
  }

  void tickSquare(double dt)
  {
    updateYawToPoi(dt);
    const Vec2& g = sq_[square_index_];
    if (flyCarrotTo({g.x, g.y, z_fly_}, p_.square_speed, dt)) {
      RCLCPP_INFO(get_logger(), "Reached square corner %zu/%zu", square_index_ + 1, sq_.size());
      if (++square_index_ >= sq_.size()) transition(State::TO_CENTER);
    }
  }

  void tickToCenter(double dt)
  {
    // At theta = 0 the figure-8 is at the POI heading along atan2(cos 0, cos 0) = 45 deg.
    const double yaw_target = outdoor::figure8Yaw(0.0);
    slewYawTo(yaw_target, dt);
    const bool there = flyCarrotTo({poi_.x, poi_.y, z_fly_}, p_.square_speed, dt);
    const bool yaw_ok = std::fabs(outdoor::wrapPi(poseYaw() - yaw_target)) < 10.0 * outdoor::kDeg2Rad;
    const bool calm = have_vel_ && fresh(vel_time_, 0.5) && speed() <= p_.stabilize_speed;
    if (there && yaw_ok && calm) {
      RCLCPP_INFO(get_logger(), "Centred on POI with tangent heading -- starting figure-8 timer");
      transition(State::FIGURE8);
    }
  }

  void tickFigure8(double)
  {
    const double s = std::chrono::duration<double>(SteadyClock::now() - f8_started_).count();
    const double theta = outdoor::figure8Phase(p_.omega, p_.ramp_s, s);   // theta = omega * t (with soft start)
    if (theta >= 2.0 * outdoor::kPi * p_.laps) {
      RCLCPP_INFO(get_logger(), "Figure-8 complete (%.2f laps in %.1f s)", p_.laps, s);
      carrot_ = cmd_;
      transition(State::RETURN);
      return;
    }
    const auto f = outdoor::figure8(p_.amplitude, theta);
    cmd_ = {poi_.x + f.x, poi_.y + f.y, z_fly_};
    cmd_yaw_ = outdoor::unwrapNear(cmd_yaw_, outdoor::figure8Yaw(theta));   // tangent heading, unwrapped
    carrot_ = cmd_;
  }

  void tickReturn(double dt)
  {
    if (flyCarrotTo({launch_.x, launch_.y, z_fly_}, p_.square_speed, dt)) {
      RCLCPP_INFO(get_logger(), "Back above the launch point -> landing");
      transition(State::LAND);
    }
  }

  void tickLand()
  {
    if (vehicle_state_.mode != "LAND") { trySetMode("LAND"); return; }
    if (!vehicle_state_.armed) {
      RCLCPP_INFO(get_logger(), "Landed and disarmed. Mission complete.");
      transition(State::DONE);
    } else logWaiting("touchdown / disarm");
  }

  void tickAborting()
  {
    if (vehicle_state_.mode == p_.abort_mode) {
      RCLCPP_WARN(get_logger(), "Vehicle is in %s. Mission node is now passive.", p_.abort_mode.c_str());
      transition(State::ABORTED);
      return;
    }
    trySetMode(p_.abort_mode);
    logWaiting("abort mode change (take over with the RC if this persists!)");
  }

  // ------------------------------------------------ status / logging output
  void printStatus()
  {
    const auto t = SteadyClock::now();
    if (std::chrono::duration<double>(t - last_status_).count() < p_.status_period) return;
    last_status_ = t;
    if (!have_pose_ || !have_fix_) { return; }
    const double yaw_pose_deg = poseYaw() * outdoor::kRad2Deg;
    std::ostringstream o;
    o << std::fixed << std::setprecision(2);
    o << "[" << stateName(state_) << "] GPS lat/lon/alt=" << std::setprecision(7) << fix_.latitude << "/"
      << fix_.longitude << std::setprecision(2) << "/" << fix_.altitude << " m(AMSL)";
    if (origin_valid_) {
      const Vec3 p = posePos();
      const outdoor::Enu g = ltp_.toEnu({fix_.latitude, fix_.longitude, fix_.altitude});
      o << " | ENU from launch: pose=(" << p.x - launch_.x << ", " << p.y - launch_.y << ", " << p.z - launch_.z
        << ") gps=(" << g.e << ", " << g.n << ", " << g.u << ")";
    }
    if (have_rel_alt_) o << " | rel_alt=" << rel_alt_;
    o << " | yaw_ENU(pose)=" << yaw_pose_deg << " deg";
    if (have_hdg_)
      o << " heading(compass)=" << compass_hdg_deg_ << " deg -> yaw_ENU(compass)="
        << outdoor::headingDegToEnuYawDeg(compass_hdg_deg_) << " deg (diff "
        << outdoor::wrapDeg180(outdoor::headingDegToEnuYawDeg(compass_hdg_deg_) - yaw_pose_deg) << ")";
    if (have_sats_) o << " | sats=" << satellites_;
    RCLCPP_INFO(get_logger(), "%s", o.str().c_str());
  }

  void appendFlownPath()
  {
    const auto t = SteadyClock::now();
    if (std::chrono::duration<double>(t - last_path_).count() < 0.2) return;
    last_path_ = t;
    if (flown_.poses.size() > 20000) return;
    flown_.header.frame_id = "map";
    flown_.header.stamp = now();
    geometry_msgs::msg::PoseStamped ps = pose_;
    ps.header.frame_id = "map";
    flown_.poses.push_back(ps);
    flown_path_pub_->publish(flown_);
  }

  void writeLogRow()
  {
    if (!log_ || !origin_valid_ || !have_pose_) return;
    const Vec3 p = posePos();
    const double yaw = poseYaw();
    double poi_err = std::nan("");
    if (state_ == State::TO_START || state_ == State::SQUARE) {
      const Vec2 a{p.x, p.y};
      if (outdoor::dist2(a, poi_) > 0.5)
        poi_err = outdoor::wrapDeg180((yaw - outdoor::bearingYaw(a, poi_)) * outdoor::kRad2Deg);
    }
    const double t = std::chrono::duration<double>(SteadyClock::now() - log_start_).count();
    log_ << std::fixed << std::setprecision(4) << t << "," << stateName(state_) << "," << cmd_.x << "," << cmd_.y
         << "," << cmd_.z << "," << outdoor::wrapDeg180(cmd_yaw_ * outdoor::kRad2Deg) << "," << p.x << "," << p.y
         << "," << p.z << "," << yaw * outdoor::kRad2Deg << ","
         << outdoor::wrapDeg180((yaw - cmd_yaw_) * outdoor::kRad2Deg) << "," << poi_err << ","
         << outdoor::dist3(p, cmd_) << "," << (have_rel_alt_ ? rel_alt_ : std::nan("")) << ","
         << (have_hdg_ ? compass_hdg_deg_ : std::nan("")) << "\n";
  }

  // ------------------------------------------------------------- members
  Params p_;
  State state_ = State::WAIT_GPS;
  SteadyClock::time_point state_started_ = SteadyClock::now(), last_wait_log_ = state_started_;
  SteadyClock::time_point last_tick_ = state_started_, last_status_ = state_started_, last_path_ = state_started_;
  SteadyClock::time_point stable_since_ = state_started_, f8_started_ = state_started_;
  SteadyClock::time_point service_started_ = state_started_, last_service_try_ = state_started_;
  SteadyClock::time_point pose_time_ = state_started_, vel_time_ = state_started_, fix_time_ = state_started_;
  SteadyClock::time_point log_start_ = state_started_;
  bool stable_since_valid_ = false;

  // telemetry
  mavros_msgs::msg::State vehicle_state_;
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::TwistStamped vel_;
  sensor_msgs::msg::NavSatFix fix_;
  mavros_msgs::msg::HomePosition home_;
  uint32_t satellites_ = 0;
  double compass_hdg_deg_ = 0.0, rel_alt_ = 0.0;
  bool have_pose_ = false, have_vel_ = false, have_fix_ = false, have_sats_ = false;
  bool have_hdg_ = false, have_rel_alt_ = false, have_home_ = false;

  // services
  bool service_pending_ = false, takeoff_requested_ = false, takeoff_acknowledged_ = false;
  uint64_t service_id_ = 0;
  bool abort_requested_ = false;

  // mission geometry (local ENU)
  bool origin_valid_ = false;
  outdoor::LocalTangentPlane ltp_;
  Vec3 launch_, cmd_, carrot_;
  double z_fly_ = 0.0, cmd_yaw_ = 0.0;
  Vec2 poi_;
  std::array<Vec2, 4> corners_;
  std::vector<Vec2> sq_;
  size_t square_index_ = 1;

  // output
  std::ofstream log_;
  nav_msgs::msg::Path flown_;

  // ROS handles
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sats_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hdg_sub_, rel_alt_sub_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr intended_path_pub_, flown_path_pub_;
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
    rclcpp::spin(std::make_shared<OutdoorMissionNode>());    // single-threaded on purpose
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("outdoor_mission_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
