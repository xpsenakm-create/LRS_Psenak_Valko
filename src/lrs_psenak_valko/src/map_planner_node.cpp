// 3D A* planner node. Rack geometry, map resolution, and safety inflation are
// loaded from ROS parameters; requests and results use PoseArray topics.

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "a_star_planner.h"
#include "voxel_occupancy_grid.h"

class MapPlannerNode final : public rclcpp::Node
{
public:
  MapPlannerNode()
  : Node("map_planner_node")
  {
    const std::string package_share = ament_index_cpp::get_package_share_directory("lrs_psenak_valko");
    const std::string map_path = declare_parameter<std::string>(
      "map_path", "maps/FEI_LRS_PCD/map.pcd");
    const std::filesystem::path configured_map_path(map_path);
    const std::string resolved_map_path = configured_map_path.is_absolute()
      ? configured_map_path.string()
      : (std::filesystem::path(package_share) / configured_map_path).lexically_normal().string();
    const std::string occupancy_pcd_path =
      declare_parameter<std::string>("occupancy_pcd_path", "map_voxels_with_racks_inflated.pcd");
    const double leaf_size = declare_parameter<double>("voxel_size", 0.15);
    const double safety_radius = declare_parameter<double>("safety_radius", 0.45);
    const std::vector<double> rack_boxes =
      declare_parameter<std::vector<double>>("rack_boxes", std::vector<double>{});
    // Only this specific map (FEI hangar) must never start without its racks filled.
    // Other maps (e.g. an outdoor world) legitimately have none -- set this to true
    // in the hangar's params YAML to keep the old hard-fail safety belt there.
    const bool require_rack_boxes = declare_parameter<bool>("require_rack_boxes", false);

    leaf_size_ = leaf_size;
    safety_radius_ = safety_radius;

    astar_params_.allow_diagonal = declare_parameter<bool>("allow_diagonal", true);
    astar_params_.heuristic_weight = declare_parameter<double>("heuristic_weight", 1.0);
    astar_params_.max_planning_time_s = declare_parameter<double>("max_planning_time_s", 5.0);
    const int64_t max_expansions = declare_parameter<int64_t>("max_expansions", 0);
    astar_params_.snap_radius_cells = declare_parameter<int>("snap_radius_cells", 0);
    astar_params_.simplify_path = declare_parameter<bool>("simplify_path", true);

    if (!std::isfinite(leaf_size) || leaf_size <= 0.0)
      throw std::invalid_argument("voxel_size must be positive and finite");
    if (!std::isfinite(safety_radius) || safety_radius < 0.0)
      throw std::invalid_argument("safety_radius must be non-negative and finite");
    if (rack_boxes.size() % 7 != 0)
      throw std::invalid_argument(
          "rack_boxes must contain whole groups of [cx, cy, cz, sx, sy, sz, yaw] "
          "(size must be a multiple of 7)");
    if (rack_boxes.empty())
    {
      if (require_rack_boxes)
        throw std::invalid_argument(
            "require_rack_boxes is true but no rack_boxes were given -- known solid "
            "obstacles (e.g. shelving racks) would not be filled");
      RCLCPP_WARN(get_logger(),
          "rack_boxes is empty -- no known solid obstacles will be force-filled. "
          "Fine for a map without racks (e.g. an outdoor world); for the FEI hangar "
          "map double-check this is intentional (see A1.1 'you cannot fly through "
          "the shelves'). Set require_rack_boxes:=true to make a missing rack_boxes "
          "a hard startup error again.");
    }
    if (!std::isfinite(astar_params_.heuristic_weight) || astar_params_.heuristic_weight < 1.0 ||
      !std::isfinite(astar_params_.max_planning_time_s) || astar_params_.max_planning_time_s <= 0.0 ||
      max_expansions < 0 || astar_params_.snap_radius_cells < 0)
      throw std::invalid_argument("invalid A* parameters");
    astar_params_.max_expansions = static_cast<size_t>(max_expansions);

    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", rclcpp::QoS(1).transient_local());
    occupancy_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inflated_occupancy", rclcpp::QoS(1).transient_local());
    path_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", 10);
    rack_marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "rack_boxes_marker", rclcpp::QoS(1).transient_local());
    occupancy_result_publisher_ = create_publisher<std_msgs::msg::Bool>("occupancy_result", 10);
        loadMap(resolved_map_path, occupancy_pcd_path, static_cast<float>(leaf_size),
          static_cast<float>(safety_radius), rack_boxes);
    publishRackBoxes(rack_boxes);

    path_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("planned_path", 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>("planning_status", 10);
    // Separate callback groups so a slow plan_request (planning can legitimately
    // take up to max_planning_time_s) never blocks occupancy_query or anything
    // else -- see main() for the MultiThreadedExecutor that lets them actually
    // run concurrently instead of just being declared separately.
    planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    query_callback_group_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions planning_sub_options;
    planning_sub_options.callback_group = planning_callback_group_;
    request_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "plan_request", 10,
        std::bind(&MapPlannerNode::handleRequest, this, std::placeholders::_1),
        planning_sub_options);

    rclcpp::SubscriptionOptions query_sub_options;
    query_sub_options.callback_group = query_callback_group_;
    occupancy_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "occupancy_query", 10,
      std::bind(&MapPlannerNode::handleOccupancyQuery, this, std::placeholders::_1),
      query_sub_options);

    RCLCPP_INFO(get_logger(), "Ready: plan via 'plan_request'; query inflated occupancy via 'occupancy_query'");
    RCLCPP_INFO(get_logger(), "Map: %s; raw voxels: %zu; inflated voxels: %zu; safety radius: %.2f m",
                resolved_map_path.c_str(), grid_->voxelCount(), grid_->inflatedVoxelCount(), safety_radius);
  }

private:
  void loadMap(const std::string& path, const std::string& occupancy_pcd_path,
               float leaf_size, float safety_radius,
               const std::vector<double>& rack_boxes)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *raw) < 0)
      throw std::runtime_error("could not load point cloud: " + path);
    if (raw->empty()) throw std::runtime_error("point cloud is empty: " + path);

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*raw, min_point, max_point);
    RCLCPP_INFO(get_logger(), "Map bounds: x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f]",
                min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(raw);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*downsampled);

    grid_ = std::make_unique<VoxelOccupancyGrid>(leaf_size);
    grid_->build(*downsampled);
    for (size_t i = 0; i < rack_boxes.size(); i += 7)
    {
      const size_t added = grid_->addBox(
          static_cast<float>(rack_boxes[i]), static_cast<float>(rack_boxes[i + 1]),
          static_cast<float>(rack_boxes[i + 2]), static_cast<float>(rack_boxes[i + 3]),
          static_cast<float>(rack_boxes[i + 4]), static_cast<float>(rack_boxes[i + 5]),
          static_cast<float>(rack_boxes[i + 6]));
      RCLCPP_INFO(get_logger(), "Filled rack box %zu (%zu new voxels)", i / 7, added);
    }
    grid_->inflate(safety_radius);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points(new pcl::PointCloud<pcl::PointXYZ>(*downsampled));
    sensor_msgs::msg::PointCloud2 map_message;
    pcl::toROSMsg(*map_points, map_message);
    map_message.header.frame_id = "map";
    map_publisher_->publish(map_message);

    const pcl::PointCloud<pcl::PointXYZ> inflated_points = grid_->inflatedPointCloud();
    if (pcl::io::savePCDFileBinary(occupancy_pcd_path, inflated_points) < 0)
      throw std::runtime_error("could not write inflated occupancy PCD: " + occupancy_pcd_path);
    RCLCPP_INFO(get_logger(), "Saved inflated voxel centers (including racks) to %s",
                occupancy_pcd_path.c_str());
    sensor_msgs::msg::PointCloud2 occupancy_message;
    pcl::toROSMsg(inflated_points, occupancy_message);
    occupancy_message.header.frame_id = "map";
    occupancy_publisher_->publish(occupancy_message);

    const VoxelOccupancyGrid::IndexBounds bounds = grid_->inflatedBounds();
    if (!bounds.valid) throw std::runtime_error("no occupied voxels in planning map");
    constexpr int64_t padding = 1;
    const int64_t sx = bounds.max_x - bounds.min_x + 1 + 2 * padding;
    const int64_t sy = bounds.max_y - bounds.min_y + 1 + 2 * padding;
    const int64_t sz = bounds.max_z - bounds.min_z + 1 + 2 * padding;
    const double cells = static_cast<double>(sx) * sy * sz;
    if (cells > 4e8) throw std::runtime_error("planning grid exceeds 400 million cells");

    grid_info_.resolution = leaf_size;
    grid_info_.origin_x = static_cast<double>(bounds.min_x - padding) * leaf_size;
    grid_info_.origin_y = static_cast<double>(bounds.min_y - padding) * leaf_size;
    grid_info_.origin_z = static_cast<double>(bounds.min_z - padding) * leaf_size;
    grid_info_.size_x = static_cast<int>(sx);
    grid_info_.size_y = static_cast<int>(sy);
    grid_info_.size_z = static_cast<int>(sz);
    const int64_t ox = bounds.min_x - padding;
    const int64_t oy = bounds.min_y - padding;
    const int64_t oz = bounds.min_z - padding;
    planner_ = std::make_unique<astar::AStarPlanner>(
        grid_info_, [this, ox, oy, oz](int x, int y, int z) {
          return grid_->isOccupiedInflatedIndex(ox + x, oy + y, oz + z);
        }, astar_params_);

    RCLCPP_INFO(get_logger(), "Voxelized points: %zu; occupied voxels: %zu; inflated: %zu",
                downsampled->size(), grid_->voxelCount(), grid_->inflatedVoxelCount());
    RCLCPP_INFO(get_logger(), "3D planning grid: %d x %d x %d", grid_info_.size_x,
                grid_info_.size_y, grid_info_.size_z);
  }

  bool pathIsCollisionFree(const std::vector<astar::Vec3>& path) const
  {
    const double step = 0.25 * grid_->leafSize();
    for (size_t i = 0; i < path.size(); ++i)
    {
      if (grid_->isOccupiedInflated(path[i].x, path[i].y, path[i].z)) return false;
      if (i == 0) continue;
      const astar::Vec3& a = path[i - 1];
      const astar::Vec3& b = path[i];
      const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
      const int samples = std::max(1, static_cast<int>(
          std::ceil(std::sqrt(dx * dx + dy * dy + dz * dz) / step)));
      for (int k = 0; k <= samples; ++k)
      {
        const double t = static_cast<double>(k) / samples;
        if (grid_->isOccupiedInflated(a.x + t * dx, a.y + t * dy, a.z + t * dz)) return false;
      }
    }
    return true;
  }

  void publishRackBoxes(const std::vector<double>& boxes)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "rack_collision_boxes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.045;
    marker.color.r = 1.0f;
    marker.color.g = 0.72f;
    marker.color.b = 0.08f;
    marker.color.a = 1.0f;

    constexpr int edges[12][2] = {
      {0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3},
      {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}
    };
    for (size_t offset = 0; offset < boxes.size(); offset += 7)
    {
      const double cx = boxes[offset], cy = boxes[offset + 1], cz = boxes[offset + 2];
      const double hx = 0.5 * boxes[offset + 3];
      const double hy = 0.5 * boxes[offset + 4];
      const double hz = 0.5 * boxes[offset + 5];
      const double c = std::cos(boxes[offset + 6]);
      const double s = std::sin(boxes[offset + 6]);
      geometry_msgs::msg::Point corners[8];
      for (int corner = 0; corner < 8; ++corner)
      {
        const double lx = (corner & 1) ? hx : -hx;
        const double ly = (corner & 2) ? hy : -hy;
        const double lz = (corner & 4) ? hz : -hz;
        corners[corner].x = cx + c * lx - s * ly;
        corners[corner].y = cy + s * lx + c * ly;
        corners[corner].z = cz + lz;
      }
      for (const auto& edge : edges)
      {
        marker.points.push_back(corners[edge[0]]);
        marker.points.push_back(corners[edge[1]]);
      }
    }
    rack_marker_publisher_->publish(marker);
  }

  void handleRequest(const geometry_msgs::msg::PoseArray::SharedPtr request)
  {
    std_msgs::msg::String status;
    if (request->poses.size() != 2)
    {
      status.data = "INVALID_REQUEST: expected exactly two poses (start, goal)";
      status_publisher_->publish(status);
      RCLCPP_WARN(get_logger(), "%s", status.data.c_str());
      return;
    }

    const auto& start = request->poses[0].position;
    const auto& goal = request->poses[1].position;
    const astar::Vec3 s{start.x, start.y, start.z}, g{goal.x, goal.y, goal.z};
    const astar::Result result = planner_->plan(s, g);

    if (!result.success())
    {
      status.data = buildReport(result, s, g, true);
      status_publisher_->publish(status);
      RCLCPP_WARN(get_logger(), "\n%s", status.data.c_str());
      return;
    }
    const bool collision_free = pathIsCollisionFree(result.path);
    status.data = buildReport(result, s, g, collision_free);
    if (!collision_free)
    {
      status_publisher_->publish(status);
      RCLCPP_ERROR(get_logger(), "\n%s", status.data.c_str());
      return;
    }

    geometry_msgs::msg::PoseArray path;
    path.header = request->header;
    for (const astar::Vec3& point : result.path)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = point.x;
      pose.position.y = point.y;
      pose.position.z = point.z;
      pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
    visualization_msgs::msg::Marker line;
    line.header = path.header;
    line.ns = "planned_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.06;
    line.color.r = 0.1f;
    line.color.g = 0.9f;
    line.color.b = 0.2f;
    line.color.a = 1.0f;
    for (const auto& pose : path.poses) line.points.push_back(pose.position);
    path_marker_publisher_->publish(line);
    status_publisher_->publish(status);
    RCLCPP_INFO(get_logger(), "\n%s", status.data.c_str());
  }

  // ------------------------------------------------------------------------
  //  Human readable report (Slovak) -- published on `planning_status` and logged.
  // ------------------------------------------------------------------------
  static std::string fmt(double v, int prec = 2)
  {
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
  }
  static std::string pt(const astar::Vec3& p)
  {
    return "(" + fmt(p.x) + ", " + fmt(p.y) + ", " + fmt(p.z) + ") m";
  }
  static double dist3(const astar::Vec3& a, const astar::Vec3& b)
  {
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  /// Nearest point that is free in the INFLATED map, searched within max_m.
  bool nearestFree(const astar::Vec3& p, double max_m, astar::Vec3& out, double& d) const
  {
    const double step = grid_info_.resolution;
    const int R = static_cast<int>(std::ceil(max_m / step));
    double best = std::numeric_limits<double>::infinity();
    for (int dz = -R; dz <= R; ++dz)
      for (int dy = -R; dy <= R; ++dy)
        for (int dx = -R; dx <= R; ++dx)
        {
          const astar::Vec3 q{p.x + dx * step, p.y + dy * step, p.z + dz * step};
          if (planner_->isBlockedWorld(q)) continue;
          const double dd = dist3(p, q);
          if (dd < best && dd <= max_m) { best = dd; out = q; }
        }
    d = best;
    return std::isfinite(best);
  }

  /// Explains why an endpoint is not usable (out of bounds / inside obstacle / inside safety margin).
  void describeBadEndpoint(std::ostringstream& o, const char* name, const astar::Vec3& p,
                           bool out_of_bounds) const
  {
    const auto& gi = grid_info_;
    if (out_of_bounds)
    {
      const double mn[3] = {gi.origin_x, gi.origin_y, gi.origin_z};
      const double mx[3] = {gi.origin_x + gi.size_x * gi.resolution,
                            gi.origin_y + gi.size_y * gi.resolution,
                            gi.origin_z + gi.size_z * gi.resolution};
      const double v[3] = {p.x, p.y, p.z};
      const char* ax[3] = {"x", "y", "z"};
      o << "  " << name << " " << pt(p) << " leží MIMO plánovacej mriežky.\n";
      for (int k = 0; k < 3; ++k)
        if (v[k] < mn[k] || v[k] >= mx[k])
          o << "    - os " << ax[k] << " = " << fmt(v[k]) << " m, povolený rozsah je ["
            << fmt(mn[k]) << ", " << fmt(mx[k]) << "] m\n";
      o << "  Tip: skontroluj súradnicový systém (origin mapy vs. pozícia dronu) a jednotky (metre).\n";
      return;
    }

    const bool raw = grid_->isOccupied(static_cast<float>(p.x), static_cast<float>(p.y),
                                       static_cast<float>(p.z));
    if (raw)
      o << "  " << name << " " << pt(p) << " leží PRIAMO V PREKÁŽKE (voxel z mapy alebo z vyplneného regálu).\n";
    else
      o << "  " << name << " " << pt(p) << " je voľný, ale leží v BEZPEČNOSTNOM PÁSME okolo prekážky "
        << "(safety_radius = " << fmt(safety_radius_) << " m).\n";

    astar::Vec3 q;
    double d = 0.0;
    if (nearestFree(p, 3.0, q, d))
    {
      const int cells = static_cast<int>(std::ceil(d / grid_info_.resolution));
      o << "    - najbližší voľný bod: " << pt(q) << ", vzdialenosť " << fmt(d) << " m (" << cells
        << " voxelov)\n";
      o << "  Tip: posuň bod do voľného priestoru, alebo nastav snap_radius_cells >= " << cells
        << " (teraz " << astar_params_.snap_radius_cells << "), alebo zníž safety_radius.\n";
    }
    else
      o << "    - do 3 m od bodu nie je žiadny voľný voxel -> bod je hlboko v obstacle/racku.\n";
  }

  std::string buildReport(const astar::Result& r, const astar::Vec3& s, const astar::Vec3& g,
                          bool collision_free) const
  {
    const std::string bar(64, '=');
    const std::string line(64, '-');
    const double res = grid_info_.resolution;
    std::ostringstream o;
    o << bar << "\n";
    o << " Štart : " << pt(s) << "\n";
    o << " Cieľ  : " << pt(g) << "\n";
    o << " Priama vzdialenosť štart-cieľ: " << fmt(dist3(s, g)) << " m"
      << "  (dz = " << fmt(g.z - s.z) << " m)\n";
    o << line << "\n";

    const bool ok = r.success() && collision_free;
    o << " VÝSLEDOK: " << (ok ? "[OK] TRASA NÁJDENÁ" : "[CHYBA] TRASA NEBOLA NÁJDENÁ / NEPLATNÁ") << "\n";
    o << " Stav plánovača: " << astar::toString(r.status) << "\n";
    o << line << "\n";

    if (!r.success())
    {
      o << " DÔVOD:\n";
      switch (r.status)
      {
        case astar::Status::START_OUT_OF_BOUNDS:
          describeBadEndpoint(o, "Štart", s, true);
          break;
        case astar::Status::GOAL_OUT_OF_BOUNDS:
          describeBadEndpoint(o, "Cieľ", g, true);
          break;
        case astar::Status::START_OCCUPIED:
          describeBadEndpoint(o, "Štart", s, false);
          break;
        case astar::Status::GOAL_OCCUPIED:
          describeBadEndpoint(o, "Cieľ", g, false);
          break;
        case astar::Status::NO_PATH:
          o << "  Štart aj cieľ sú voľné, ale cieľ je z neho NEDOSIAHNUTEĽNÝ (prehľadaný celý\n"
            << "  súvislý voľný priestor okolo štartu a cieľ v ňom nie je).\n";
          o << "    - prehľadaných voxelov: " << r.expansions << " (~"
            << fmt(r.expansions * res * res * res, 1) << " m3 voľného priestoru)\n";
          if (std::isfinite(r.closest_dist_m))
            o << "    - najbližšie k cieľu sa trasa dostala na " << fmt(r.closest_dist_m) << " m, bod "
              << pt(r.closest_to_goal) << "\n";
          o << "  Možné príčiny:\n"
            << "    1) safety_radius (" << fmt(safety_radius_) << " m) uzavrel úzky priechod -> skús menší\n"
            << "    2) cieľ je v uzavretom priestore (za regálom / v obstacle)\n"
            << "    3) rack_boxes zablokovali jediný priechod -> skontroluj rozmery a yaw\n";
          break;
        case astar::Status::TIMEOUT:
        {
          const bool by_exp = astar_params_.max_expansions && r.expansions >= astar_params_.max_expansions;
          o << "  Prekročený limit " << (by_exp ? "počtu expanzií" : "času plánovania")
            << " skôr, než sa našla odpoveď (trasa môže existovať).\n";
          o << "    - limit času: " << fmt(astar_params_.max_planning_time_s, 1) << " s, uplynulo "
            << fmt(r.planning_time_ms / 1000.0, 2) << " s\n";
          o << "    - limit expanzií: "
            << (astar_params_.max_expansions ? std::to_string(astar_params_.max_expansions) : "bez limitu")
            << ", vykonaných " << r.expansions << "\n";
          if (std::isfinite(r.closest_dist_m))
            o << "    - najbližšie k cieľu: " << fmt(r.closest_dist_m) << " m, bod "
              << pt(r.closest_to_goal) << "\n";
          o << "  Tip: zvýš max_planning_time_s / max_expansions, alebo heuristic_weight > 1 (rýchlejšie,\n"
            << "  mierne suboptimálne), alebo zväčši voxel_size.\n";
          break;
        }
        default:
          o << "  Neznámy dôvod.\n";
      }
      o << line << "\n";
      o << " Čas plánovania: " << fmt(r.planning_time_ms, 1) << " ms, expanzií: " << r.expansions << "\n";
      o << " Parametre: voxel_size=" << fmt(leaf_size_) << " m, safety_radius=" << fmt(safety_radius_)
        << " m, heuristic_weight=" << fmt(astar_params_.heuristic_weight, 1) << ", "
        << (astar_params_.allow_diagonal ? "26" : "6") << "-susedstvo\n";
      o << bar;
      return o.str();
    }

    // ---- success (path may still fail the independent collision check) ----
    o << " TRASA\n";
    o << "   Body pôvodne (A*)     : " << r.raw_waypoints << "\n";
    o << "   Body po zjednodušení  : " << r.path.size() << "\n";
    o << "   Dĺžka trasy (finálna) : " << fmt(r.path_length_m) << " m\n";
    o << "   Dĺžka pôvodnej trasy  : " << fmt(r.raw_path_length_m) << " m\n";
    o << "   Čas plánovania        : " << fmt(r.planning_time_ms, 1) << " ms\n";
    if (r.start_snapped) o << "   ! Štart posunutý na " << pt(r.start_used) << "\n";
    if (r.goal_snapped)  o << "   ! Cieľ posunutý na " << pt(r.goal_used) << "\n";
    if (!collision_free) o << "   [CHYBA] trasa koliduje s nafúknutou mapou -> nepublikujem\n";
    o << line << "\n";
    o << " BODY TRASY (#, x, y, z, úsek, kumulatívne)\n";
    double cum = 0.0;
    for (size_t i = 0; i < r.path.size(); ++i)
    {
      const double seg = i ? dist3(r.path[i - 1], r.path[i]) : 0.0;
      cum += seg;
      o << "   " << std::setw(2) << i << "  " << pt(r.path[i]) << "   +" << fmt(seg) << " m   "
        << fmt(cum) << " m\n";
    }
    o << bar;
    return o.str();
  }

  void handleOccupancyQuery(const geometry_msgs::msg::PointStamped::SharedPtr query)
  {
    std_msgs::msg::Bool result;
    result.data = planner_->isBlockedWorld({query->point.x, query->point.y, query->point.z});
    occupancy_result_publisher_->publish(result);
  }

  double leaf_size_ = 0.0, safety_radius_ = 0.0;
  astar::Params astar_params_;
  astar::GridInfo grid_info_;
  std::unique_ptr<VoxelOccupancyGrid> grid_;
  std::unique_ptr<astar::AStarPlanner> planner_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rack_marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr occupancy_result_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr request_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr occupancy_subscription_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr query_callback_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<MapPlannerNode>();
    // Needed for the two callback groups above to actually run concurrently --
    // the default single-threaded rclcpp::spin() would serialize them anyway.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_planner_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}