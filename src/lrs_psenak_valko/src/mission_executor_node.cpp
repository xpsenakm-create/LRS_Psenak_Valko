#include "mission_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
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
    setpoint_rate_hz_ = declare_parameter<double>("setpoint_rate_hz", 20.0);
    handshake_timeout_sec_ = declare_parameter<double>("handshake_timeout_sec", 60.0);
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 5.0);
    planner_timeout_sec_ = declare_parameter<double>("planner_timeout_sec", 10.0);
    navigation_timeout_sec_ = declare_parameter<double>("navigation_timeout_sec", 120.0);
    takeoff_timeout_sec_ = declare_parameter<double>("takeoff_timeout_sec", 60.0);
    landing_timeout_sec_ = declare_parameter<double>("landing_timeout_sec", 90.0);
    yaw_tolerance_deg_ = declare_parameter<double>("yaw_tolerance_deg", 5.0);
    yaw_hold_time_sec_ = declare_parameter<double>("yaw_hold_time_sec", 0.5);
    yaw_timeout_sec_ = declare_parameter<double>("yaw_timeout_sec", 15.0);
    turn_alignment_threshold_deg_ = declare_parameter<double>("turn_alignment_threshold_deg", 20.0);
    map_to_local_yaw_offset_deg_ = declare_parameter<double>("map_to_local_yaw_offset_deg", 90.0);
    if (hard_radius_ <= 0.0 || soft_radius_ <= hard_radius_ || pass_radius_ <= 0.0 ||
      settle_speed_ <= 0.0 || settle_time_ <= 0.0 || setpoint_rate_hz_ < 10.0 ||
      setpoint_rate_hz_ > 20.0 || handshake_timeout_sec_ <= 0.0 ||
      service_timeout_sec_ <= 0.0 || planner_timeout_sec_ <= 0.0 ||
      navigation_timeout_sec_ <= 0.0 || takeoff_timeout_sec_ <= 0.0 || landing_timeout_sec_ <= 0.0 ||
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
    rclcpp::SubscriptionOptions telemetry_options;
    telemetry_options.callback_group = state_group_;
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", 10,
      std::bind(&MissionExecutorNode::onPose, this, std::placeholders::_1), telemetry_options);
    velocity_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/mavros/local_position/velocity_local", 10,
      std::bind(&MissionExecutorNode::onVelocity, this, std::placeholders::_1), telemetry_options);
    path_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "planned_path", 10,
      std::bind(&MissionExecutorNode::onPath, this, std::placeholders::_1), telemetry_options);
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      "planning_status", 10,
      std::bind(&MissionExecutorNode::onPlanningStatus, this, std::placeholders::_1), telemetry_options);

    setpoint_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
    plan_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("plan_request", 10);
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    takeoff_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / setpoint_rate_hz_));
    timer_ = create_wall_timer(timer_period, std::bind(&MissionExecutorNode::tick, this), timer_group_);

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
    ALIGN_PATH, NAVIGATE, TASK, LAND, WAIT_LANDED, WAIT_DISARMED, RETAKEOFF, DONE, ERROR
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
      takeoff_requested_ = false;
      takeoff_acknowledged_ = false;
      takeoff_start_altitude_ = have_pose_ ? pose_.pose.position.z : 0.0;
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
      std::chrono::steady_clock::now() - velocity_received_at_).count() <= 0.5;
  }

  void publishSetpoint()
  {
    if (!have_pose_ || !target_initialized_) return;
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

  void tick()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    publishSetpoint();
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
      fail("state timed out after " + std::to_string(state_timeout) + " s");
      return;
    }
    if (state_ != State::IDLE && state_ != State::WAIT_FCU && have_pose_ &&
      std::chrono::duration<double>(steady_now - pose_received_at_).count() > 1.0)
    {
      fail("local position telemetry is stale");
      return;
    }
    const bool requires_guided_flight = state_ == State::PLAN || state_ == State::WAIT_PLAN ||
      state_ == State::WAIT_PREFETCH || state_ == State::ALIGN_PATH || state_ == State::NAVIGATE ||
      state_ == State::TASK;
    if (requires_guided_flight && !vehicle_state_.connected) {
      fail("MAVROS FCU connection was lost during the mission");
      return;
    }
    if (requires_guided_flight && !vehicle_state_.armed) {
      fail("vehicle disarmed unexpectedly during the mission");
      return;
    }
    if ((state_ == State::ALIGN_PATH || state_ == State::NAVIGATE || state_ == State::TASK) &&
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
      case State::WAIT_FCU:
        if (vehicle_state_.connected && have_pose_) transition(State::SET_MODE);
        else logWaiting();
        break;
      case State::SET_MODE:
        if (vehicle_state_.mode == "GUIDED") { transition(State::ARM); break; }
        if (!service_pending_ && mode_client_->service_is_ready()) {
          const uint64_t request_id = beginServiceCall();
          auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
          request->custom_mode = "GUIDED";
          mode_client_->async_send_request(request, [this, request_id](
              std::shared_future<mavros_msgs::srv::SetMode::Response::SharedPtr> future) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!finishServiceCall(request_id)) return;
            const auto response = future.get();
            if (!response || !response->mode_sent) fail("MAVROS rejected GUIDED mode request");
          });
        } else logWaiting();
        break;
      case State::ARM:
        if (vehicle_state_.armed) {
          transition(landtakeoff_pending_ ? State::RETAKEOFF : State::TAKEOFF);
          break;
        }
        if (!service_pending_ && arm_client_->service_is_ready()) {
          const uint64_t request_id = beginServiceCall();
          auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          request->value = true;
          arm_client_->async_send_request(request, [this, request_id](
              std::shared_future<mavros_msgs::srv::CommandBool::Response::SharedPtr> future) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!finishServiceCall(request_id)) return;
            const auto response = future.get();
            if (!response || !response->success) fail("MAVROS rejected arm request");
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
        if (velocityFresh() && pose_.pose.position.z <= 0.15 && speed() <= 0.15) {
          if (vehicle_state_.armed) disarmOnGround();
          else transition(State::WAIT_DISARMED);
        }
        break;
      case State::WAIT_DISARMED:
        if (!vehicle_state_.armed) {
          RCLCPP_INFO(get_logger(), "Disarm confirmed on ground");
          if (landtakeoff_pending_) transition(State::SET_MODE);
          else {
            logTaskComplete(mission_[waypoint_index_].task.type);
            transition(State::DONE);
          }
        } else logWaiting();
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
    if (!takeoff_requested_ && takeoff_client_->service_is_ready()) {
      const uint64_t request_id = beginServiceCall();
      takeoff_requested_ = true;
      auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      request->altitude = static_cast<float>(altitude);
      takeoff_client_->async_send_request(request, [this, request_id](
          std::shared_future<mavros_msgs::srv::CommandTOL::Response::SharedPtr> future) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!finishServiceCall(request_id)) return;
        const auto response = future.get();
        if (!response || !response->success) fail("MAVROS rejected takeoff request");
        else {
          takeoff_acknowledged_ = true;
          RCLCPP_INFO(get_logger(), "MAVROS accepted takeoff to %.2f m", mission_[waypoint_index_].z);
        }
      });
    } else if (!takeoff_requested_) {
      logWaiting();
      return true;
    }

    if (!takeoff_acknowledged_) return true;
    const double confirmed_altitude = std::max(takeoff_start_altitude_ + 0.5, altitude - hard_radius_);
    if (!vehicle_state_.armed || pose_.pose.position.z < confirmed_altitude) return true;
    RCLCPP_INFO(get_logger(), "Takeoff confirmed: z=%.2f m (threshold %.2f m)",
      pose_.pose.position.z, confirmed_altitude);
    return false;
  }

  void navigateTick()
  {
    const auto& goal = mission_[waypoint_index_];
    if (path_index_ < path_.size()) {
      const auto& pose = path_[path_index_];
      setTarget(pose, segmentYaw(path_index_));
      const geometry_msgs::msg::Pose map_pose = localToMapPose(pose_.pose);
      const double dx = map_pose.position.x - pose.position.x;
      const double dy = map_pose.position.y - pose.position.y;
      const double dz = map_pose.position.z - pose.position.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) <= pass_radius_) {
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
    if (distanceTo(goal) > radius) { stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type()); return; }
    if (goal.precision == mission::Precision::SOFT) {
      RCLCPP_INFO(get_logger(), "Reached soft waypoint %zu: error=%.3f m; continuing without settling",
        waypoint_index_ + 1, distanceTo(goal));
      transition(State::TASK);
      return;
    }
    if (!velocityFresh()) {
      stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return;
    }
    if (speed() > settle_speed_) { stable_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type()); return; }
    if (stable_since_.nanoseconds() == 0) stable_since_ = now();
    if ((now() - stable_since_).seconds() >= settle_time_) {
      RCLCPP_INFO(get_logger(), "Settled at hard waypoint %zu: error=%.3f m, speed=%.3f m/s for %.2f s",
        waypoint_index_ + 1, distanceTo(goal), speed(), settle_time_);
      transition(State::TASK);
    }
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
        if (!response || !response->success) { fail("MAVROS rejected disarm request"); return; }
        transition(State::WAIT_DISARMED);
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
  bool service_pending_ = false, takeoff_requested_ = false, takeoff_acknowledged_ = false;
  uint64_t service_request_id_ = 0;
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
  double yaw_tolerance_deg_ = 5.0, yaw_hold_time_sec_ = 0.5, yaw_timeout_sec_ = 15.0;
  double turn_alignment_threshold_deg_ = 20.0;
  double map_to_local_yaw_offset_deg_ = 90.0;
  double setpoint_rate_hz_ = 20.0, handshake_timeout_sec_ = 60.0;
  double service_timeout_sec_ = 5.0, planner_timeout_sec_ = 10.0;
  double navigation_timeout_sec_ = 120.0;
  double takeoff_timeout_sec_ = 60.0, landing_timeout_sec_ = 90.0;
  double takeoff_start_altitude_ = 0.0;
  double target_yaw_ = 0.0;
  std::chrono::steady_clock::time_point state_started_at_ = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point last_wait_log_at_ = state_started_at_;
  std::chrono::steady_clock::time_point service_started_at_ = state_started_at_;
  std::chrono::steady_clock::time_point pose_received_at_ = state_started_at_;
  std::chrono::steady_clock::time_point velocity_received_at_ = state_started_at_;
  rclcpp::Time stable_since_{0, 0, RCL_ROS_TIME};
  rclcpp::CallbackGroup::SharedPtr state_group_, timer_group_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr plan_publisher_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
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