#include "lrs_psenak_valko/mission_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/estimator_status.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/status_text.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MissionExecutorNode final : public rclcpp::Node
{
public:
  MissionExecutorNode() : Node("mission_executor_node")
  {
    const std::string mission_file = declare_parameter<std::string>("mission_file", "");
    if (mission_file.empty()) throw std::invalid_argument("set the mission_file parameter");
    mission_ = mission::loadCsv(mission_file);

    hard_radius_ = declare_parameter<double>("hard_radius", 0.15);
    soft_radius_ = declare_parameter<double>("soft_radius", 0.55);
    pass_radius_ = declare_parameter<double>("pass_radius", 0.45);
    settle_speed_ = declare_parameter<double>("settle_speed", 0.20);
    settle_time_ = declare_parameter<double>("settle_time", 1.0);
    pose_stale_timeout_sec_ = declare_parameter<double>("pose_stale_timeout_sec", 5.0);
    velocity_stale_timeout_sec_ = declare_parameter<double>("velocity_stale_timeout_sec", 3.0);
    local_position_rate_hz_ = declare_parameter<double>("local_position_rate_hz", 10.0);
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
    handshake_timeout_sec_ = declare_parameter<double>("handshake_timeout_sec", 180.0);
    fcu_ready_timeout_sec_ = declare_parameter<double>("fcu_ready_timeout_sec", 600.0);
    gps_stable_sec_ = declare_parameter<double>("gps_stable_sec", 5.0);
    ground_wait_sec_ = declare_parameter<double>("ground_wait_sec", 3.0);
    disarm_grace_sec_ = declare_parameter<double>("disarm_grace_sec", 5.0);
    cruise_speed_ = declare_parameter<double>("cruise_speed", 1.0);
    max_lead_ = declare_parameter<double>("max_lead", 1.0);
    log_dir_ = declare_parameter<std::string>("log_dir", "");
    exit_on_done_ = declare_parameter<bool>("exit_on_done", true);
    mode_retry_interval_sec_ = declare_parameter<double>("mode_retry_interval_sec", 2.0);
    arm_retry_interval_sec_ = declare_parameter<double>("arm_retry_interval_sec", 2.0);
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 5.0);
    planner_timeout_sec_ = declare_parameter<double>("planner_timeout_sec", 10.0);
    navigation_timeout_sec_ = declare_parameter<double>("navigation_timeout_sec", 120.0);
    takeoff_timeout_sec_ = declare_parameter<double>("takeoff_timeout_sec", 60.0);
    takeoff_climb_timeout_sec_ = declare_parameter<double>("takeoff_climb_timeout_sec", 12.0);
    max_takeoff_attempts_ = static_cast<uint32_t>(
      std::max<int64_t>(0, declare_parameter<int64_t>("max_takeoff_attempts", 3)));
    landing_timeout_sec_ = declare_parameter<double>("landing_timeout_sec", 90.0);
    yaw_tolerance_deg_ = declare_parameter<double>("yaw_tolerance_deg", 5.0);
    yaw_hold_time_sec_ = declare_parameter<double>("yaw_hold_time_sec", 0.5);
    yaw_timeout_sec_ = declare_parameter<double>("yaw_timeout_sec", 15.0);
    turn_alignment_threshold_deg_ = declare_parameter<double>("turn_alignment_threshold_deg", 20.0);
    map_to_local_yaw_offset_deg_ = declare_parameter<double>("map_to_local_yaw_offset_deg", 90.0);
    if (hard_radius_ <= 0.0 || soft_radius_ <= hard_radius_ || pass_radius_ <= 0.0 ||
      settle_speed_ <= 0.0 || settle_time_ <= 0.0 || setpoint_rate_hz_ < 10.0 ||
      setpoint_rate_hz_ > 20.0 || handshake_timeout_sec_ <= 0.0 ||
      fcu_ready_timeout_sec_ <= 0.0 || gps_stable_sec_ < 0.0 || ground_wait_sec_ < 0.0 ||
      cruise_speed_ <= 0.0 || max_lead_ <= pass_radius_ ||
      pose_stale_timeout_sec_ <= 1.0 || velocity_stale_timeout_sec_ <= 0.5 ||
      local_position_rate_hz_ <= 0.0 ||
      mode_retry_interval_sec_ <= 0.0 || arm_retry_interval_sec_ <= 0.0 ||
      service_timeout_sec_ <= 0.0 || planner_timeout_sec_ <= 0.0 ||
      navigation_timeout_sec_ <= 0.0 || takeoff_timeout_sec_ <= 0.0 || landing_timeout_sec_ <= 0.0 ||
      takeoff_climb_timeout_sec_ <= 0.0 ||
      yaw_tolerance_deg_ <= 0.0 || yaw_tolerance_deg_ >= 45.0 ||
      yaw_hold_time_sec_ <= 0.0 || yaw_timeout_sec_ <= 0.0 ||
      turn_alignment_threshold_deg_ <= 0.0 || turn_alignment_threshold_deg_ >= 180.0 ||
      !std::isfinite(map_to_local_yaw_offset_deg_))
      throw std::invalid_argument("invalid waypoint precision/controller parameters");

    state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_group_;
    state_subscription_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10,
      std::bind(&MissionExecutorNode::onState, this, std::placeholders::_1), state_options);
    const rclcpp::QoS telemetry_qos = rclcpp::SensorDataQoS();
    const rclcpp::QoS latched_qos = rclcpp::QoS(1).reliable().transient_local();
    rclcpp::SubscriptionOptions telemetry_options;
    telemetry_options.callback_group = state_group_;
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", telemetry_qos,
      std::bind(&MissionExecutorNode::onPose, this, std::placeholders::_1), telemetry_options);
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", telemetry_qos,
      std::bind(&MissionExecutorNode::onVelocity, this, std::placeholders::_1), telemetry_options);
    gps_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/mavros/global_position/global", telemetry_qos,
      std::bind(&MissionExecutorNode::onGps, this, std::placeholders::_1), telemetry_options);
    home_subscription_ = create_subscription<mavros_msgs::msg::HomePosition>(
      "/mavros/home_position/home", latched_qos,
      std::bind(&MissionExecutorNode::onHome, this, std::placeholders::_1), telemetry_options);
    estimator_subscription_ = create_subscription<mavros_msgs::msg::EstimatorStatus>(
      "/mavros/estimator_status", telemetry_qos,
      std::bind(&MissionExecutorNode::onEstimatorStatus, this, std::placeholders::_1), telemetry_options);
    // FCU text messages (PreArm reasons, "EKF3 IMU0 is using GPS", mode change failures) are
    // otherwise only visible in the MAVProxy console window.
    statustext_subscription_ = create_subscription<mavros_msgs::msg::StatusText>(
      "/mavros/statustext/recv", telemetry_qos,
      std::bind(&MissionExecutorNode::onStatusText, this, std::placeholders::_1), telemetry_options);
    path_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "planned_path", 10,
      std::bind(&MissionExecutorNode::onPath, this, std::placeholders::_1), telemetry_options);
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      "planning_status", 10,
      std::bind(&MissionExecutorNode::onPlanningStatus, this, std::placeholders::_1), telemetry_options);

    setpoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    plan_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("plan_request", 10);
    flown_path_publisher_ = create_publisher<nav_msgs::msg::Path>("mission/flown_path", 10);
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    message_interval_client_ = create_client<mavros_msgs::srv::MessageInterval>(
      "/mavros/set_message_interval");

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / setpoint_rate_hz_));
    timer_ = create_wall_timer(timer_period, std::bind(&MissionExecutorNode::tick, this), timer_group_);

    openLogs();
    RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", mission_.size(), mission_file.c_str());
    for (size_t i = 0; i < mission_.size(); ++i) {
      const auto& point = mission_[i];
      RCLCPP_INFO(get_logger(), "Waypoint %zu: (%.2f, %.2f, %.2f), %s, task=%s",
        i + 1, point.x, point.y, point.z, mission::toString(point.precision),
        mission::toString(point.task.type));
    }
    transition(State::IDLE);
  }

private:
  enum class State
  {
    IDLE, WAIT_FCU, SET_MODE, ARM, TAKEOFF, PLAN, WAIT_PLAN, WAIT_PREFETCH,
    ALIGN_PATH, NAVIGATE, TASK, LAND, WAIT_LANDED, WAIT_DISARMED, GROUND_WAIT, RETAKEOFF, DONE, ERROR
  };

  static const char* stateName(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::WAIT_FCU: return "WAIT_FCU";
      case State::SET_MODE: return "SET_MODE";
      case State::ARM: return "ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::PLAN: return "PLAN";
      case State::WAIT_PLAN: return "WAIT_PLAN";
      case State::WAIT_PREFETCH: return "WAIT_PREFETCH";
      case State::ALIGN_PATH: return "ALIGN_PATH";
      case State::NAVIGATE: return "NAVIGATE";
      case State::TASK: return "TASK";
      case State::LAND: return "LAND";
      case State::WAIT_LANDED: return "WAIT_LANDED";
      case State::WAIT_DISARMED: return "WAIT_DISARMED";
      case State::GROUND_WAIT: return "GROUND_WAIT";
      case State::RETAKEOFF: return "RETAKEOFF";
      case State::DONE: return "DONE";
      case State::ERROR: return "ERROR";
    }
    return "UNKNOWN";
  }

  void transition(State next)
  {
    state_ = next;
    state_started_at_ = std::chrono::steady_clock::now();
    last_wait_log_at_ = state_started_at_;
    if (next == State::TAKEOFF || next == State::RETAKEOFF) {
      takeoff_target_set_ = false;
      takeoff_request_sent_ = false;
      takeoff_command_accepted_ = false;
      next_takeoff_attempt_at_ = std::chrono::steady_clock::now();
      takeoff_start_altitude_ = have_pose_ ? pose_.pose.position.z : 0.0;
    }
    if (next == State::SET_MODE) {
      next_mode_attempt_at_ = std::chrono::steady_clock::now();
      mode_rejection_count_ = 0;
    }
    if (next == State::ARM) {
      next_arm_attempt_at_ = std::chrono::steady_clock::now();
      arm_rejection_count_ = 0;
    }
    if (next == State::NAVIGATE) carrot_ = target_.pose.position;
    if (next == State::TASK || next == State::NAVIGATE || next == State::ALIGN_PATH) {
      resetHardArrivalSamples();
    }
    RCLCPP_INFO(get_logger(), "Mission state: %s", stateName(state_));
    service_pending_ = false;
    ++service_request_id_;
    stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void fail(const std::string& reason)
  {
    RCLCPP_ERROR(get_logger(), "Mission failed in %s: %s", stateName(state_), reason.c_str());
    if (have_pose_) {
      target_.pose = localToMapPose(pose_.pose);
      target_yaw_ = yawFromQuaternion(target_.pose.orientation);
      target_initialized_ = true;
    }
    transition(State::ERROR);
  }

  uint64_t beginServiceCall()
  {
    service_pending_ = true;
    service_started_at_ = std::chrono::steady_clock::now();
    return ++service_request_id_;
  }

  bool finishServiceCall(uint64_t request_id)
  {
    if (!service_pending_ || request_id != service_request_id_ || state_ == State::ERROR) return false;
    service_pending_ = false;
    return true;
  }

  double timeoutForState() const
  {
    switch (state_) {
      case State::WAIT_FCU:
        return fcu_ready_timeout_sec_;
      case State::SET_MODE:
      case State::ARM:
        return handshake_timeout_sec_;
      case State::WAIT_PLAN:
      case State::WAIT_PREFETCH:
        return planner_timeout_sec_;
      case State::NAVIGATE:
        return navigation_timeout_sec_;
      case State::ALIGN_PATH:
      case State::TASK:
        return yaw_timeout_sec_;
      case State::TAKEOFF:
      case State::RETAKEOFF:
        return takeoff_timeout_sec_;
      case State::LAND:
      case State::WAIT_LANDED:
      case State::WAIT_DISARMED:
        return landing_timeout_sec_;
      default:
        return 0.0;
    }
  }

  void logWaiting()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_steady - last_wait_log_at_).count() < 5.0) return;
    RCLCPP_INFO(get_logger(), "Still waiting in state %s", stateName(state_));
    last_wait_log_at_ = now_steady;
  }

  void onState(const mavros_msgs::msg::State::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_state_ = *message;
  }
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pose_ = *message;
    have_pose_ = true;
    pose_received_at_ = std::chrono::steady_clock::now();
    if (!target_initialized_) {
      target_.pose = localToMapPose(pose_.pose);
      target_initialized_ = true;
      target_yaw_ = yawFromQuaternion(target_.pose.orientation);
      RCLCPP_INFO(get_logger(),
        "Coordinate frames: MAVROS local=(%.2f, %.2f, %.2f), planner map=(%.2f, %.2f, %.2f), yaw offset=%.1f deg",
        pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z,
        target_.pose.position.x, target_.pose.position.y, target_.pose.position.z,
        map_to_local_yaw_offset_deg_);
    }
  }
  void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    velocity_ = *message;
    have_velocity_ = true;
    velocity_received_at_ = std::chrono::steady_clock::now();
    ++velocity_sample_sequence_;
  }
  void onGps(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool fix = message->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    if (!fix) gps_fix_since_valid_ = false;
    else if (!gps_fix_since_valid_) {
      gps_fix_since_ = std::chrono::steady_clock::now();
      gps_fix_since_valid_ = true;
    }
    gps_fix_status_ = message->status.status;
    gps_received_at_ = std::chrono::steady_clock::now();
    have_gps_ = true;
  }
  void onHome(const mavros_msgs::msg::HomePosition::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    have_home_ = std::isfinite(message->geo.latitude) && std::isfinite(message->geo.longitude) &&
      (message->geo.latitude != 0.0 || message->geo.longitude != 0.0);
  }
  void onStatusText(const mavros_msgs::msg::StatusText::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (message->text.find("is using GPS") != std::string::npos && !ekf_using_gps_) {
      ekf_using_gps_ = true;
      RCLCPP_INFO(get_logger(), "FCU: %s -> EKF has a GPS position", message->text.c_str());
      return;
    }
    if (message->severity <= mavros_msgs::msg::StatusText::WARNING) {
      RCLCPP_WARN(get_logger(), "FCU: %s", message->text.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "FCU: %s", message->text.c_str());
    }
  }
  void onEstimatorStatus(const mavros_msgs::msg::EstimatorStatus::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    estimator_status_ = *message;
    estimator_received_at_ = std::chrono::steady_clock::now();
    have_estimator_status_ = true;
  }
  void onPath(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (prefetch_request_pending_) {
      prefetched_path_ = message->poses;
      prefetched_path_received_ = true;
    } else if (state_ == State::WAIT_PLAN) {
      path_ = message->poses;
      got_path_ = true;
    }
  }
  void onPlanningStatus(const std_msgs::msg::String::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool success = message->data.find("[OK] TRASA NÁJDENÁ") != std::string::npos;
    if (prefetch_request_pending_) {
      prefetched_status_received_ = true;
      prefetched_plan_ok_ = success;
      if (!success) prefetched_plan_error_ = message->data;
    } else if (state_ == State::WAIT_PLAN) {
      got_status_ = true;
      plan_ok_ = success;
      if (!success) plan_error_ = message->data;
    }
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double unwrapNear(double reference, double angle)
  {
    return reference + std::remainder(angle - reference, 2.0 * M_PI);
  }

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  geometry_msgs::msg::Pose localToMapPose(const geometry_msgs::msg::Pose& local_pose) const
  {
    const double offset = map_to_local_yaw_offset_deg_ * M_PI / 180.0;
    const double c = std::cos(offset), s = std::sin(offset);
    geometry_msgs::msg::Pose map_pose = local_pose;
    map_pose.position.x = c * local_pose.position.x + s * local_pose.position.y;
    map_pose.position.y = -s * local_pose.position.x + c * local_pose.position.y;
    map_pose.orientation = quaternionFromYaw(yawFromQuaternion(local_pose.orientation) - offset);
    return map_pose;
  }

  geometry_msgs::msg::Pose mapToLocalPose(const geometry_msgs::msg::Pose& map_pose) const
  {
    const double offset = map_to_local_yaw_offset_deg_ * M_PI / 180.0;
    const double c = std::cos(offset), s = std::sin(offset);
    geometry_msgs::msg::Pose local_pose = map_pose;
    local_pose.position.x = c * map_pose.position.x - s * map_pose.position.y;
    local_pose.position.y = s * map_pose.position.x + c * map_pose.position.y;
    local_pose.orientation = quaternionFromYaw(yawFromQuaternion(map_pose.orientation) + offset);
    return local_pose;
  }

  double currentMapYaw() const
  {
    return yawFromQuaternion(pose_.pose.orientation) - map_to_local_yaw_offset_deg_ * M_PI / 180.0;
  }

  double distanceTo(const mission::Waypoint& point) const
  {
    const geometry_msgs::msg::Pose map_pose = localToMapPose(pose_.pose);
    const double dx = map_pose.position.x - point.x;
    const double dy = map_pose.position.y - point.y;
    const double dz = map_pose.position.z - point.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double speed() const
  {
    const auto& v = velocity_.twist.linear;
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }

  bool velocityFresh() const
  {
    return have_velocity_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - velocity_received_at_).count() <= velocity_stale_timeout_sec_;
  }

  bool poseFresh() const
  {
    return have_pose_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - pose_received_at_).count() <= pose_stale_timeout_sec_;
  }


  // Arming readiness: the EKF announced it uses GPS, or (when the executor was started after that
  // message) the GPS fix has been stable for gps_stable_sec.  Without this, ArduCopter refuses
  // GUIDED ("requires position") and arming, and the mission would burn its handshake timeout.
  bool fcuReady(std::string* why) const
  {
    if (!vehicle_state_.connected) { *why = "FCU not connected"; return false; }
    if (!poseFresh()) { *why = "no fresh local position"; return false; }
    if (ekf_using_gps_) return true;
    const bool gps_fresh = have_gps_ && std::chrono::duration<double>(
      std::chrono::steady_clock::now() - gps_received_at_).count() <= 2.0;
    if (!gps_fresh || !gps_fix_since_valid_) { *why = "no GPS fix yet"; return false; }
    const double stable = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - gps_fix_since_).count();
    if (stable < gps_stable_sec_) { *why = "GPS fix not stable yet"; return false; }
    return true;
  }

  void resetHardArrivalSamples()
  {
    stable_velocity_samples_ = 0;
    last_stable_velocity_sequence_ = 0;
    stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  // ArduCopter's GUIDED mode drops a running takeoff as soon as it receives a position setpoint
  // (ModeGuided::set_destination switches the submode to Pos), and the Pos controller does nothing
  // while the vehicle is still landed: rotors idle, no climb, then DISARM_DELAY disarms it.
  // So no setpoints are streamed while arming/taking off/landing, only during flight (and in ERROR,
  // where the target is the current pose and acts as a hold).
  bool setpointsAllowed() const
  {
    switch (state_) {
      case State::PLAN:
      case State::WAIT_PLAN:
      case State::WAIT_PREFETCH:
      case State::ALIGN_PATH:
      case State::NAVIGATE:
      case State::TASK:
      case State::ERROR:
        return true;
      default:
        return false;
    }
  }

  void publishSetpoint()
  {
    if (!have_pose_ || !target_initialized_ || !setpointsAllowed()) return;
    target_.header.stamp = now();
    target_.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped local_target = target_;
    local_target.pose = mapToLocalPose(target_.pose);
    local_target.pose.orientation = quaternionFromYaw(target_yaw_ + map_to_local_yaw_offset_deg_ * M_PI / 180.0);
    setpoint_publisher_->publish(local_target);
  }

  void setTarget(const geometry_msgs::msg::Pose& pose, double yaw)
  {
    target_.pose = pose;
    target_yaw_ = unwrapNear(target_yaw_, yaw);
  }

  void requestPlan()
  {
    if (!have_pose_) return;
    const auto& goal = mission_[waypoint_index_];
    geometry_msgs::msg::PoseArray request;
    request.header.frame_id = "map";
    request.header.stamp = now();
    geometry_msgs::msg::Pose start = localToMapPose(pose_.pose);
    geometry_msgs::msg::Pose finish;
    finish.position.x = goal.x;
    finish.position.y = goal.y;
    finish.position.z = goal.z;
    finish.orientation.w = 1.0;
    request.poses = {start, finish};
    got_path_ = false;
    got_status_ = false;
    plan_ok_ = false;
    plan_error_.clear();
    prefetch_request_pending_ = false;
    transition(State::WAIT_PLAN);
    plan_publisher_->publish(request);
  }

  void requestPrefetchPlan()
  {
    if (waypoint_index_ + 1 >= mission_.size()) return;
    const auto& start_waypoint = mission_[waypoint_index_];
    const auto& goal = mission_[waypoint_index_ + 1];
    geometry_msgs::msg::PoseArray request;
    request.header.frame_id = "map";
    request.header.stamp = now();
    geometry_msgs::msg::Pose start;
    start.position.x = start_waypoint.x;
    start.position.y = start_waypoint.y;
    start.position.z = start_waypoint.z;
    start.orientation.w = 1.0;
    geometry_msgs::msg::Pose finish;
    finish.position.x = goal.x;
    finish.position.y = goal.y;
    finish.position.z = goal.z;
    finish.orientation.w = 1.0;
    request.poses = {start, finish};

    prefetched_waypoint_index_ = waypoint_index_ + 1;
    prefetched_path_.clear();
    prefetched_path_received_ = false;
    prefetched_status_received_ = false;
    prefetched_plan_ok_ = false;
    prefetched_plan_error_.clear();
    prefetch_request_pending_ = true;
    plan_publisher_->publish(request);
  }

  bool activatePrefetchedPath()
  {
    if (!prefetched_status_received_) return false;
    if (prefetched_waypoint_index_ != waypoint_index_) {
      fail("prefetched route does not match the current mission waypoint");
      return false;
    }
    if (!prefetched_plan_ok_) {
      fail("planner failed for next waypoint: " + prefetched_plan_error_);
      return false;
    }
    if (!prefetched_path_received_) return false;
    if (prefetched_path_.empty()) {
      fail("planner returned an empty prefetched path");
      return false;
    }
    const auto& goal = mission_[waypoint_index_];
    const auto& endpoint = prefetched_path_.back().position;
    const double dx = endpoint.x - goal.x;
    const double dy = endpoint.y - goal.y;
    const double dz = endpoint.z - goal.z;
    const double endpoint_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double acceptance = goal.precision == mission::Precision::HARD
      ? hard_radius_ : soft_radius_;
    if (endpoint_error > acceptance) {
      fail("prefetched route endpoint is " + std::to_string(endpoint_error) +
        " m from the mission goal, outside its " + std::string(mission::toString(goal.precision)) +
        " tolerance");
      return false;
    }
    path_ = std::move(prefetched_path_);
    prefetch_request_pending_ = false;
    beginPathNavigation();
    return true;
  }

  void beginPathNavigation()
  {
    logPlannedPath();
    if (path_.size() > 1) {
      path_index_ = 1;
      setTarget(localToMapPose(pose_.pose), segmentYaw(path_index_));
      transition(State::ALIGN_PATH);
    } else {
      path_index_ = 0;
      setTarget(path_.back(), target_yaw_);
      transition(State::NAVIGATE);
    }
    requestPrefetchPlan();
  }

  // ------------------------------------------------------------------ flight logging
  // <log_dir>/flight_log.csv: one row per control tick (20 Hz), map frame, read by plot_a1_2.py.
  // <log_dir>/planned_paths.csv: every planner route that was flown, one row per path point.
  void openLogs()
  {
    if (log_dir_.empty()) return;
    std::error_code error;
    std::filesystem::create_directories(log_dir_, error);
    flight_log_.open(log_dir_ + "/flight_log.csv");
    path_log_.open(log_dir_ + "/planned_paths.csv");
    if (!flight_log_ || !path_log_) {
      RCLCPP_WARN(get_logger(), "Cannot write logs to %s; flight logging disabled", log_dir_.c_str());
      flight_log_.close();
      path_log_.close();
      return;
    }
    flight_log_ << "t_s,state,waypoint,armed,mode,cmd_valid,cmd_x,cmd_y,cmd_z,cmd_yaw_deg,"
                   "x,y,z,yaw_deg,speed\n";
    path_log_ << "leg,waypoint,point,x,y,z\n";
    log_started_at_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Logging flight data to %s", log_dir_.c_str());
  }

  void logSample()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (have_pose_ && std::chrono::duration<double>(steady_now - last_flown_publish_at_).count() >= 0.2 &&
      vehicle_state_.armed)
    {
      last_flown_publish_at_ = steady_now;
      geometry_msgs::msg::PoseStamped sample;
      sample.header.frame_id = "map";
      sample.header.stamp = now();
      sample.pose = localToMapPose(pose_.pose);
      flown_path_.header = sample.header;
      if (flown_path_.poses.size() < 20000) flown_path_.poses.push_back(sample);
      flown_path_publisher_->publish(flown_path_);
    }
    if (!flight_log_ || !have_pose_) return;
    const geometry_msgs::msg::Pose map_pose = localToMapPose(pose_.pose);
    const bool cmd_valid = target_initialized_ && setpointsAllowed();
    const auto deg = [](double rad) { return std::remainder(rad, 2.0 * M_PI) * 180.0 / M_PI; };
    flight_log_ << std::fixed << std::setprecision(3)
      << std::chrono::duration<double>(steady_now - log_started_at_).count() << ','
      << stateName(state_) << ',' << waypoint_index_ + 1 << ','
      << (vehicle_state_.armed ? 1 : 0) << ',' << vehicle_state_.mode << ','
      << (cmd_valid ? 1 : 0) << ','
      << target_.pose.position.x << ',' << target_.pose.position.y << ',' << target_.pose.position.z << ','
      << deg(target_yaw_) << ','
      << map_pose.position.x << ',' << map_pose.position.y << ',' << map_pose.position.z << ','
      << deg(currentMapYaw()) << ',' << (have_velocity_ ? speed() : 0.0) << '\n';
    if (std::chrono::duration<double>(steady_now - last_log_flush_at_).count() >= 1.0) {
      flight_log_.flush();
      last_log_flush_at_ = steady_now;
    }
  }

  void logPlannedPath()
  {
    ++leg_counter_;
    if (!path_log_) return;
    path_log_ << std::fixed << std::setprecision(3);
    for (size_t i = 0; i < path_.size(); ++i) {
      const auto& p = path_[i].position;
      path_log_ << leg_counter_ << ',' << waypoint_index_ + 1 << ',' << i << ','
        << p.x << ',' << p.y << ',' << p.z << '\n';
    }
    path_log_.flush();
  }

  void tick()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    publishSetpoint();
    logSample();
    if (state_ == State::DONE && exit_on_done_ && !shutdown_requested_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - state_started_at_).count() > 1.0)
    {
      shutdown_requested_ = true;
      if (flight_log_) flight_log_.flush();
      if (path_log_) path_log_.flush();
      RCLCPP_INFO(get_logger(), "Mission DONE; executor exits (exit_on_done=true)");
      rclcpp::shutdown();
      return;
    }
    if (state_ == State::DONE || state_ == State::ERROR) return;
    const auto steady_now = std::chrono::steady_clock::now();
    if (service_pending_ &&
      std::chrono::duration<double>(steady_now - service_started_at_).count() > service_timeout_sec_)
    {
      fail("MAVROS service response timed out in " + std::string(stateName(state_)));
      return;
    }
    const double state_timeout = timeoutForState();
    if (state_timeout > 0.0 &&
      std::chrono::duration<double>(steady_now - state_started_at_).count() > state_timeout)
    {
      if (state_ == State::ARM) {
        fail("arming timed out after " + std::to_string(state_timeout) + " s, rejected attempts=" +
          std::to_string(arm_rejection_count_) + "; inspect SITL pre-arm messages");
      } else {
        fail("state timed out after " + std::to_string(state_timeout) + " s");
      }
      return;
    }
    if (vehicle_state_.armed && have_pose_ &&
      std::chrono::duration<double>(steady_now - pose_received_at_).count() > pose_stale_timeout_sec_)
    {
      fail("local position telemetry is stale (MAVROS lost the FCU: is Gazebo paused, or SITL closed?)");
      return;
    }
    const bool requires_guided_flight = state_ == State::TAKEOFF || state_ == State::RETAKEOFF ||
      state_ == State::PLAN || state_ == State::WAIT_PLAN ||
      state_ == State::WAIT_PREFETCH || state_ == State::ALIGN_PATH || state_ == State::NAVIGATE ||
      state_ == State::TASK;
    if (requires_guided_flight && !vehicle_state_.connected) {
      fail("MAVROS FCU connection was lost during the mission");
      return;
    }
    if (requires_guided_flight && !vehicle_state_.armed) {
      // ArduCopter auto-disarms (DISARM_DELAY) when it is armed but never leaves the ground,
      // e.g. when the EKF was not ready at arming time.  Retry mode/arm/takeoff a few times.
      const bool on_ground = !have_pose_ || pose_.pose.position.z < takeoff_start_altitude_ + 0.3;
      if ((state_ == State::TAKEOFF || state_ == State::RETAKEOFF) && on_ground &&
        takeoff_disarm_retries_ < max_takeoff_attempts_)
      {
        ++takeoff_disarm_retries_;
        RCLCPP_WARN(get_logger(),
          "Vehicle disarmed before leaving the ground (attempt %u/%u); retrying GUIDED, arm and takeoff",
          takeoff_disarm_retries_, max_takeoff_attempts_);
        transition(State::SET_MODE);
        return;
      }
      fail("vehicle disarmed unexpectedly during the mission");
      return;
    }
    if ((state_ == State::TAKEOFF || state_ == State::RETAKEOFF || state_ == State::ALIGN_PATH ||
      state_ == State::NAVIGATE || state_ == State::TASK) &&
      vehicle_state_.mode != "GUIDED")
    {
      fail("vehicle left GUIDED mode during waypoint execution");
      return;
    }
    if (state_ != State::IDLE && state_ != State::WAIT_FCU && !have_pose_) return;

    switch (state_) {
      case State::IDLE:
        transition(State::WAIT_FCU);
        break;
      case State::WAIT_FCU: {
        std::string why;
        if (fcuReady(&why)) {
          if (!position_rate_requested_ && message_interval_client_->service_is_ready()) {
            position_rate_requested_ = true;
            auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
            request->message_id = 32;
            request->message_rate = static_cast<float>(local_position_rate_hz_);
            message_interval_client_->async_send_request(request, [this](
                std::shared_future<mavros_msgs::srv::MessageInterval::Response::SharedPtr> future) {
              const auto response = future.get();
              if (response && response->success) {
                RCLCPP_INFO(get_logger(), "Requested LOCAL_POSITION_NED at %.1f Hz",
                  local_position_rate_hz_);
              }
            });
          }
          RCLCPP_INFO(get_logger(), "FCU ready (%s)",
            ekf_using_gps_ ? "EKF is using GPS" : "GPS fix stable");
          transition(State::SET_MODE);
        } else if (std::chrono::duration<double>(steady_now - last_wait_log_at_).count() >= 5.0) {
          RCLCPP_INFO(get_logger(), "Waiting for FCU readiness: %s", why.c_str());
          last_wait_log_at_ = steady_now;
        }
        break;
      }
      case State::SET_MODE:
        if (vehicle_state_.mode == "GUIDED") {
          transition(State::ARM);
          break;
        }
        if (!service_pending_ && mode_client_->service_is_ready() &&
          std::chrono::steady_clock::now() >= next_mode_attempt_at_) {
          const uint64_t request_id = beginServiceCall();
          auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          request->custom_mode = "GUIDED";
          mode_client_->async_send_request(request, [this, request_id](
              std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> future) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!finishServiceCall(request_id)) return;
            const auto response = future.get();
            if (!response || !response->mode_sent) {
              ++mode_rejection_count_;
              next_mode_attempt_at_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(mode_retry_interval_sec_));
              RCLCPP_WARN(get_logger(),
                "Set GUIDED mode attempt %u rejected; retrying in %.1f s",
                mode_rejection_count_, mode_retry_interval_sec_);
            } else {
              // mode_sent only means MAVROS sent it; the FCU may still refuse (e.g. no position
              // yet), so give /mavros/state time to report GUIDED before resending.
              next_mode_attempt_at_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(mode_retry_interval_sec_));
              RCLCPP_INFO(get_logger(), "GUIDED mode request sent to FCU");
            }
          });
        } else logWaiting();
        break;
      case State::ARM:
        if (vehicle_state_.armed) {
          transition(landtakeoff_pending_ ? State::RETAKEOFF : State::TAKEOFF);
          break;
        }
        if (!poseFresh()) {
          logWaiting();
          break;
        }
        if (!service_pending_ && arm_client_->service_is_ready() &&
          std::chrono::steady_clock::now() >= next_arm_attempt_at_) {
          const uint64_t request_id = beginServiceCall();
          auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          request->value = true;
          arm_client_->async_send_request(request, [this, request_id](
              std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> future) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!finishServiceCall(request_id)) return;
            const auto response = future.get();
            if (!response || !response->success) {
              const uint8_t result = response ? response->result : 255;
              ++arm_rejection_count_;
              next_arm_attempt_at_ = std::chrono::steady_clock::now() +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(arm_retry_interval_sec_));
              RCLCPP_WARN(get_logger(),
                "Arm attempt %u rejected (MAV_RESULT=%u); retrying in %.1f s. "
                "Check SITL console for pre-arm reason.",
                arm_rejection_count_, result, arm_retry_interval_sec_);
            }
          });
        } else logWaiting();
        break;
      case State::TAKEOFF:
        if (!takeoffService(mission_[waypoint_index_].z)) transition(State::PLAN);
        break;
      case State::PLAN:
        requestPlan();
        break;
      case State::WAIT_PLAN:
        if (got_status_ && !plan_ok_) { fail("planner failed: " + plan_error_); break; }
        if (got_status_ && got_path_) {
          if (path_.empty()) { fail("planner returned an empty path"); break; }
          const auto& goal = mission_[waypoint_index_];
          const auto& endpoint = path_.back().position;
          const double dx = endpoint.x - goal.x;
          const double dy = endpoint.y - goal.y;
          const double dz = endpoint.z - goal.z;
          const double endpoint_error = std::sqrt(dx * dx + dy * dy + dz * dz);
          const double acceptance = goal.precision == mission::Precision::HARD
            ? hard_radius_ : soft_radius_;
          if (endpoint_error > acceptance) {
            fail("planner snapped the mission goal " + std::to_string(endpoint_error) +
              " m away, outside its " + std::string(mission::toString(goal.precision)) +
              " tolerance");
            break;
          }
          beginPathNavigation();
        } else logWaiting();
        break;
      case State::WAIT_PREFETCH:
        if (!activatePrefetchedPath()) logWaiting();
        break;
      case State::ALIGN_PATH:
        alignPathTick();
        break;
      case State::NAVIGATE:
        navigateTick();
        break;
      case State::TASK:
        taskTick();
        break;
      case State::LAND:
        if (!service_pending_ && land_client_->service_is_ready()) {
          const uint64_t request_id = beginServiceCall();
          auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
          request->yaw = static_cast<float>(target_yaw_);
          land_client_->async_send_request(request, [this, request_id](
              std::shared_future<mavros_msgs::srv::CommandTOL::Response::SharedPtr> future) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!finishServiceCall(request_id)) return;
            const auto response = future.get();
            if (!response || !response->success) fail("MAVROS rejected land request");
            else transition(State::WAIT_LANDED);
          });
        } else logWaiting();
        break;
      case State::WAIT_LANDED:
        if (!vehicle_state_.armed ||
          (velocityFresh() && pose_.pose.position.z <= 0.15 && speed() <= 0.15))
        {
          RCLCPP_INFO(get_logger(), "Touchdown: z=%.2f m; waiting for LAND mode to disarm",
            pose_.pose.position.z);
          next_disarm_attempt_at_ = steady_now +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(disarm_grace_sec_));
          transition(State::WAIT_DISARMED);
        }
        break;
      case State::WAIT_DISARMED:
        // ArduCopter's LAND mode disarms by itself once its own land detector agrees (about a
        // second after touchdown).  Before that, a disarm command is refused, so it is only sent
        // as a fallback after disarm_grace_sec, and a refusal is retried rather than fatal.
        if (vehicle_state_.armed) {
          if (steady_now >= next_disarm_attempt_at_) disarmOnGround();
          else logWaiting();
        } else {
          RCLCPP_INFO(get_logger(), "Disarm confirmed on ground");
          if (landtakeoff_pending_) transition(State::GROUND_WAIT);
          else {
            logTaskComplete(mission_[waypoint_index_].task.type);
            transition(State::DONE);
          }
        }
        break;
      case State::GROUND_WAIT:
        if (std::chrono::duration<double>(steady_now - state_started_at_).count() >= ground_wait_sec_) {
          RCLCPP_INFO(get_logger(), "Waited %.1f s on the ground; re-entering GUIDED for takeoff",
            ground_wait_sec_);
          transition(State::SET_MODE);
        }
        break;
      case State::RETAKEOFF:
        if (!takeoffService(mission_[waypoint_index_].z)) {
          landtakeoff_pending_ = false;
          logTaskComplete(mission::TaskType::LAND_TAKEOFF);
          advanceWaypoint();
        }
        break;
      case State::DONE:
      case State::ERROR:
        break;
    }
  }

  bool takeoffService(double altitude)
  {
    if (!takeoff_target_set_ && have_pose_) {
      geometry_msgs::msg::Pose takeoff_target = localToMapPose(pose_.pose);
      takeoff_target.position.z = altitude;
      setTarget(takeoff_target, target_yaw_);
      takeoff_target_set_ = true;
      RCLCPP_INFO(get_logger(),
        "Takeoff setpoint: holding map x=%.2f y=%.2f and commanding z=%.2f m",
        takeoff_target.position.x, takeoff_target.position.y, altitude);
    }
    if (!takeoff_target_set_) {
      logWaiting();
      return true;
    }

    if (!takeoff_command_accepted_) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (!service_pending_ && !takeoff_request_sent_ && steady_now >= next_takeoff_attempt_at_ &&
        takeoff_client_->service_is_ready())
      {
        const uint64_t request_id = beginServiceCall();
        takeoff_request_sent_ = true;
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = static_cast<float>(altitude);
        takeoff_client_->async_send_request(request, [this, request_id, altitude](
            std::shared_future<mavros_msgs::srv::CommandTOL::Response::SharedPtr> future) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (!finishServiceCall(request_id)) return;
          const auto response = future.get();
          if (!response || !response->success) {
            takeoff_request_sent_ = false;
            next_takeoff_attempt_at_ = std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(takeoff_retry_interval_sec_));
            RCLCPP_WARN(get_logger(),
              "MAVROS rejected NAV_TAKEOFF (result=%u); retrying in %.1f s",
              response ? response->result : 255, takeoff_retry_interval_sec_);
          } else {
            takeoff_command_accepted_ = true;
            takeoff_accepted_at_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(get_logger(), "MAVROS accepted NAV_TAKEOFF to %.2f m", altitude);
          }
        });
      } else if (!service_pending_ && !takeoff_client_->service_is_ready()) {
        logWaiting();
      }
      if (!takeoff_command_accepted_) return true;
    }

    const double confirmed_altitude = std::max(takeoff_start_altitude_ + 0.5, altitude - hard_radius_);
    if (!vehicle_state_.armed || pose_.pose.position.z < confirmed_altitude) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (vehicle_state_.armed && pose_.pose.position.z < takeoff_start_altitude_ + 0.3 &&
        std::chrono::duration<double>(steady_now - takeoff_accepted_at_).count() >
          takeoff_climb_timeout_sec_)
      {
        if (++takeoff_resends_ > max_takeoff_attempts_) {
          fail("vehicle accepted NAV_TAKEOFF but did not climb; check EKF/GPS and SITL messages");
          return true;
        }
        RCLCPP_WARN(get_logger(), "No climb %.0f s after NAV_TAKEOFF; re-sending (attempt %u/%u)",
          takeoff_climb_timeout_sec_, takeoff_resends_, max_takeoff_attempts_);
        takeoff_command_accepted_ = false;
        takeoff_request_sent_ = false;
        next_takeoff_attempt_at_ = steady_now;
        return true;
      }
      if (std::chrono::duration<double>(steady_now - last_wait_log_at_).count() >= 5.0) {
        RCLCPP_INFO(get_logger(),
          "Takeoff progress: armed=%s local_z=%.2f m target_z=%.2f m; waiting for climb",
          vehicle_state_.armed ? "true" : "false", pose_.pose.position.z, altitude);
        last_wait_log_at_ = steady_now;
      }
      return true;
    }
    takeoff_disarm_retries_ = 0;
    takeoff_resends_ = 0;
    RCLCPP_INFO(get_logger(), "Takeoff confirmed: z=%.2f m (threshold %.2f m)",
      pose_.pose.position.z, confirmed_altitude);
    return false;
  }

  void navigateTick()
  {
    const auto& goal = mission_[waypoint_index_];
    if (path_index_ < path_.size()) {
      const auto& pose = path_[path_index_];
      const bool carrot_at_vertex = advanceCarrot(pose.position);
      geometry_msgs::msg::Pose carrot_pose = pose;
      carrot_pose.position = carrot_;
      setTarget(carrot_pose, segmentYaw(path_index_));
      const geometry_msgs::msg::Pose map_pose = localToMapPose(pose_.pose);
      const double dx = map_pose.position.x - pose.position.x;
      const double dy = map_pose.position.y - pose.position.y;
      const double dz = map_pose.position.z - pose.position.z;
      if (carrot_at_vertex && std::sqrt(dx * dx + dy * dy + dz * dz) <= pass_radius_) {
        const size_t reached_index = path_index_;
        ++path_index_;
        if (path_index_ < path_.size()) {
          const double incoming_yaw = segmentYaw(reached_index);
          const double outgoing_yaw = segmentYaw(path_index_);
          const double turn = std::abs(std::remainder(outgoing_yaw - incoming_yaw, 2.0 * M_PI));
          if (turn >= turn_alignment_threshold_deg_ * M_PI / 180.0) {
            setTarget(path_[reached_index], outgoing_yaw);
            transition(State::ALIGN_PATH);
          }
        }
      }
      return;
    }

    setTarget(path_.back(), target_yaw_);
    const double radius = goal.precision == mission::Precision::HARD ? hard_radius_ : soft_radius_;
    if (distanceTo(goal) > radius) { resetHardArrivalSamples(); return; }
    if (goal.precision == mission::Precision::SOFT) {
      RCLCPP_INFO(get_logger(), "Reached soft waypoint %zu: error=%.3f m; continuing without settling",
        waypoint_index_ + 1, distanceTo(goal));
      transition(State::TASK);
      return;
    }
    if (!velocityFresh()) {
      resetHardArrivalSamples();
      return;
    }
    if (speed() > settle_speed_) { resetHardArrivalSamples(); return; }
    if (velocity_sample_sequence_ != last_stable_velocity_sequence_) {
      if (stable_velocity_samples_ == 0) stable_since_ = now();
      ++stable_velocity_samples_;
      last_stable_velocity_sequence_ = velocity_sample_sequence_;
    }
    if (stable_velocity_samples_ >= 2 && (now() - stable_since_).seconds() >= settle_time_) {
      RCLCPP_INFO(get_logger(), "Settled at hard waypoint %zu: error=%.3f m, speed=%.3f m/s for %.2f s",
        waypoint_index_ + 1, distanceTo(goal), speed(), settle_time_);
      transition(State::TASK);
    }
  }

  // Carrot following: the setpoint slides along the planned segment at cruise_speed and never runs
  // more than max_lead ahead of the drone.  Jumping the setpoint straight to the segment end lets
  // ArduCopter mix its horizontal and vertical speed limits, so it leaves the straight line (e.g.
  // descends early) and can clip an obstacle the A* route went over.  Returns true at the vertex.
  bool advanceCarrot(const geometry_msgs::msg::Point& vertex)
  {
    const auto drone = localToMapPose(pose_.pose).position;
    const auto dist = [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
      return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    };
    const double remaining = dist(carrot_, vertex);
    if (remaining < 1e-3) { carrot_ = vertex; return true; }
    if (dist(drone, carrot_) < max_lead_) {
      const double step = std::min(cruise_speed_ / setpoint_rate_hz_, remaining);
      carrot_.x += (vertex.x - carrot_.x) / remaining * step;
      carrot_.y += (vertex.y - carrot_.y) / remaining * step;
      carrot_.z += (vertex.z - carrot_.z) / remaining * step;
    }
    return dist(carrot_, vertex) < 1e-3;
  }

  void alignPathTick()
  {
    const double heading_error = std::abs(std::remainder(
      target_yaw_ - currentMapYaw(), 2.0 * M_PI));
    if (heading_error > yaw_tolerance_deg_ * M_PI / 180.0) {
      stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return;
    }
    if (stable_since_.nanoseconds() == 0) stable_since_ = now();
    if ((now() - stable_since_).seconds() >= yaw_hold_time_sec_) {
      RCLCPP_INFO(get_logger(), "Aligned with path heading %.1f deg", target_yaw_ * 180.0 / M_PI);
      transition(State::NAVIGATE);
    }
  }

  double segmentYaw(size_t path_index) const
  {
    if (path_index == 0 || path_index >= path_.size()) return target_yaw_;
    const size_t from = path_index - 1;
    const size_t to = path_index;
    const auto& a = path_[from].position;
    const auto& b = path_[to].position;
    return std::atan2(b.y - a.y, b.x - a.x);
  }

  void taskTick()
  {
    const auto& task = mission_[waypoint_index_].task;
    switch (task.type) {
      case mission::TaskType::NONE:
        logTaskComplete(task.type);
        advanceWaypoint();
        return;
      case mission::TaskType::TAKEOFF:
        logTaskComplete(task.type);
        advanceWaypoint();
        return;
      case mission::TaskType::YAW: {
        const double commanded = task.yaw_degrees * M_PI / 180.0;
        target_yaw_ = unwrapNear(target_yaw_, commanded);
        const double heading_error = std::abs(std::remainder(
          currentMapYaw() - commanded, 2.0 * M_PI));
        if (heading_error > yaw_tolerance_deg_ * M_PI / 180.0) {
          stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          return;
        }
        if (stable_since_.nanoseconds() == 0) stable_since_ = now();
        if ((now() - stable_since_).seconds() >= yaw_hold_time_sec_) {
          RCLCPP_INFO(get_logger(), "Yaw task %.1f deg reached and held for %.2f s",
            task.yaw_degrees, yaw_hold_time_sec_);
          logTaskComplete(task.type);
          advanceWaypoint();
        }
        return;
      }
      case mission::TaskType::LAND:
        landtakeoff_pending_ = false;
        RCLCPP_INFO(get_logger(), "Executing land task at waypoint %zu", waypoint_index_ + 1);
        transition(State::LAND);
        return;
      case mission::TaskType::LAND_TAKEOFF:
        landtakeoff_pending_ = true;
        RCLCPP_INFO(get_logger(), "Executing landtakeoff task at waypoint %zu", waypoint_index_ + 1);
        transition(State::LAND);
        return;
    }
  }

  void logTaskComplete(mission::TaskType task)
  {
    RCLCPP_INFO(get_logger(), "Completed task '%s' at waypoint %zu",
      mission::toString(task), waypoint_index_ + 1);
  }

  void advanceWaypoint()
  {
    ++waypoint_index_;
    if (waypoint_index_ >= mission_.size()) {
      transition(State::DONE);
      RCLCPP_INFO(get_logger(), "Mission complete");
      return;
    }
    if (prefetch_request_pending_) transition(State::WAIT_PREFETCH);
    else transition(State::PLAN);
  }

  void disarmOnGround()
  {
    if (!service_pending_ && arm_client_->service_is_ready()) {
      const uint64_t request_id = beginServiceCall();
      auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      request->value = false;
      arm_client_->async_send_request(request, [this, request_id](
            std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> future) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!finishServiceCall(request_id)) return;
        const auto response = future.get();
        next_disarm_attempt_at_ = std::chrono::steady_clock::now() +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(arm_retry_interval_sec_));
        if (!response || !response->success) {
          RCLCPP_WARN(get_logger(), "Disarm refused by FCU (not landed yet?); retrying in %.1f s",
            arm_retry_interval_sec_);
        }
      });
    } else logWaiting();
  }

  mission::Mission mission_;
  size_t waypoint_index_ = 0;
  size_t path_index_ = 0;
  std::vector<geometry_msgs::msg::Pose> path_;
  State state_ = State::WAIT_FCU;
  mavros_msgs::msg::State vehicle_state_;
  geometry_msgs::msg::PoseStamped pose_, target_;
  geometry_msgs::msg::TwistStamped velocity_;
  bool have_pose_ = false, have_velocity_ = false, target_initialized_ = false;
  bool have_gps_ = false, have_home_ = false, have_estimator_status_ = false;
  bool ekf_using_gps_ = false, gps_fix_since_valid_ = false;
  std::chrono::steady_clock::time_point gps_fix_since_ = std::chrono::steady_clock::now();
  int8_t gps_fix_status_ = sensor_msgs::msg::NavSatStatus::STATUS_UNKNOWN;
  mavros_msgs::msg::EstimatorStatus estimator_status_;
  bool service_pending_ = false;
  bool takeoff_request_sent_ = false, takeoff_command_accepted_ = false;
  bool takeoff_target_set_ = false;
  bool position_rate_requested_ = false;
  uint64_t service_request_id_ = 0;
  uint32_t mode_rejection_count_ = 0;
  uint32_t arm_rejection_count_ = 0;
  uint64_t velocity_sample_sequence_ = 0, last_stable_velocity_sequence_ = 0;
  uint32_t stable_velocity_samples_ = 0;
  bool got_path_ = false, got_status_ = false, plan_ok_ = false;
  bool prefetch_request_pending_ = false, prefetched_path_received_ = false;
  bool prefetched_status_received_ = false, prefetched_plan_ok_ = false;
  bool landtakeoff_pending_ = false;
  std::string plan_error_;
  std::string prefetched_plan_error_;
  size_t prefetched_waypoint_index_ = 0;
  std::vector<geometry_msgs::msg::Pose> prefetched_path_;
  std::mutex state_mutex_;
  double hard_radius_ = 0.15, soft_radius_ = 0.55, pass_radius_ = 0.45;
  double settle_speed_ = 0.20, settle_time_ = 1.0;
  double pose_stale_timeout_sec_ = 5.0, velocity_stale_timeout_sec_ = 3.0;
  double local_position_rate_hz_ = 10.0;
  double yaw_tolerance_deg_ = 5.0, yaw_hold_time_sec_ = 0.5, yaw_timeout_sec_ = 15.0;
  double turn_alignment_threshold_deg_ = 20.0;
  double map_to_local_yaw_offset_deg_ = 90.0;
  double setpoint_rate_hz_ = 20.0, handshake_timeout_sec_ = 180.0;
  double fcu_ready_timeout_sec_ = 600.0, gps_stable_sec_ = 5.0, ground_wait_sec_ = 3.0;
  double disarm_grace_sec_ = 5.0;
  double cruise_speed_ = 1.0, max_lead_ = 1.0;
  geometry_msgs::msg::Point carrot_;
  std::string log_dir_;
  bool exit_on_done_ = true, shutdown_requested_ = false;
  std::ofstream flight_log_, path_log_;
  size_t leg_counter_ = 0;
  nav_msgs::msg::Path flown_path_;
  std::chrono::steady_clock::time_point log_started_at_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_log_flush_at_ = log_started_at_;
  std::chrono::steady_clock::time_point last_flown_publish_at_ = log_started_at_;
  std::chrono::steady_clock::time_point next_disarm_attempt_at_ = std::chrono::steady_clock::now();
  double mode_retry_interval_sec_ = 2.0;
  double arm_retry_interval_sec_ = 2.0;
  double service_timeout_sec_ = 5.0, planner_timeout_sec_ = 10.0;
  double navigation_timeout_sec_ = 120.0;
  double takeoff_timeout_sec_ = 60.0, landing_timeout_sec_ = 90.0;
  double takeoff_retry_interval_sec_ = 2.0;
  double takeoff_climb_timeout_sec_ = 12.0;
  uint32_t max_takeoff_attempts_ = 3, takeoff_disarm_retries_ = 0, takeoff_resends_ = 0;
  double takeoff_start_altitude_ = 0.0;
  double target_yaw_ = 0.0;
  std::chrono::steady_clock::time_point state_started_at_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_wait_log_at_ = state_started_at_;
  std::chrono::steady_clock::time_point service_started_at_ = state_started_at_;
  std::chrono::steady_clock::time_point pose_received_at_ = state_started_at_;
  std::chrono::steady_clock::time_point velocity_received_at_ = state_started_at_;
  std::chrono::steady_clock::time_point gps_received_at_ = state_started_at_;
  std::chrono::steady_clock::time_point estimator_received_at_ = state_started_at_;
  std::chrono::steady_clock::time_point next_mode_attempt_at_ = state_started_at_;
  std::chrono::steady_clock::time_point next_arm_attempt_at_ = state_started_at_;
  std::chrono::steady_clock::time_point next_takeoff_attempt_at_ = state_started_at_;
  std::chrono::steady_clock::time_point takeoff_accepted_at_ = state_started_at_;
  rclcpp::Time stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::CallbackGroup::SharedPtr state_group_, timer_group_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_subscription_;
  rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr statustext_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr flown_path_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr message_interval_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_, land_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MissionExecutorNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_executor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}