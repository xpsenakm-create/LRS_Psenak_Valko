// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
//
// A1.1 bod 1+2+3 -- Map loading, obstacle inflation, 3D A* planning.
//
// Pipeline:
//   .pcd -> voxel downsample -> sparse occupancy grid -> fill racks (solid boxes)
//        -> inflate (3D sphere) -> dense index grid (bounded) -> A* -> verified path
//
// Usage:
//   map_planner_node [--pcd FILE] [--leaf M] [--radius M]
//                    [--rack cx cy cz sx sy sz yaw]      (repeatable, yaw in radians)
//                    [--start X Y Z] [--goal X Y Z]
//                    [--weight W] [--snap CELLS] [--no-simplify] [--csv OUT.csv]

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/common/common.h>  // pcl::getMinMax3D
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "a_star_planner.h"
#include "voxel_occupancy_grid.h"

struct Rack { float cx, cy, cz, sx, sy, sz, yaw; };

struct Options
{
  std::string pcd = "maps/FEI_LRS_PCD/map.pcd";
  float leaf = 0.15f;
  float radius = 0.3f;  // drone radius + controller tolerance + margin -- justify in docs
  std::vector<Rack> racks;
  bool has_start = false, has_goal = false;
  astar::Vec3 start, goal;
  astar::Params astar_params;
  std::string csv;
};

static bool parseArgs(int argc, char** argv, Options& o)
{
  auto need = [&](int i, int n) {
    if (i + n >= argc) { std::cerr << "ERROR: " << argv[i] << " needs " << n << " value(s)\n"; return false; }
    return true;
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--pcd")            { if (!need(i, 1)) return false; o.pcd = argv[++i]; }
    else if (a == "--leaf")      { if (!need(i, 1)) return false; o.leaf = std::stof(argv[++i]); }
    else if (a == "--radius")    { if (!need(i, 1)) return false; o.radius = std::stof(argv[++i]); }
    else if (a == "--weight")    { if (!need(i, 1)) return false; o.astar_params.heuristic_weight = std::stod(argv[++i]); }
    else if (a == "--snap")      { if (!need(i, 1)) return false; o.astar_params.snap_radius_cells = std::stoi(argv[++i]); }
    else if (a == "--csv")       { if (!need(i, 1)) return false; o.csv = argv[++i]; }
    else if (a == "--no-simplify") { o.astar_params.simplify_path = false; }
    else if (a == "--start")     { if (!need(i, 3)) return false;
                                   o.start = {std::stod(argv[i+1]), std::stod(argv[i+2]), std::stod(argv[i+3])};
                                   o.has_start = true; i += 3; }
    else if (a == "--goal")      { if (!need(i, 3)) return false;
                                   o.goal = {std::stod(argv[i+1]), std::stod(argv[i+2]), std::stod(argv[i+3])};
                                   o.has_goal = true; i += 3; }
    else if (a == "--rack")      { if (!need(i, 7)) return false;
                                   o.racks.push_back({std::stof(argv[i+1]), std::stof(argv[i+2]), std::stof(argv[i+3]),
                                                      std::stof(argv[i+4]), std::stof(argv[i+5]), std::stof(argv[i+6]),
                                                      std::stof(argv[i+7])});
                                   i += 7; }
    else { std::cerr << "ERROR: unknown argument " << a << "\n"; return false; }
  }
  return true;
}

// Independent check against the SPARSE inflated grid (not the dense copy A* used):
// every waypoint and every point along each segment (every leaf/4) must be free.
static bool pathIsCollisionFree(const VoxelOccupancyGrid& grid, const std::vector<astar::Vec3>& path)
{
  const double step = 0.25 * grid.leafSize();
  for (size_t i = 0; i < path.size(); ++i) {
    if (grid.isOccupiedInflated(path[i].x, path[i].y, path[i].z)) return false;
    if (i == 0) continue;
    const astar::Vec3 &a = path[i - 1], &b = path[i];
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const int n = std::max(1, static_cast<int>(std::ceil(std::sqrt(dx*dx + dy*dy + dz*dz) / step)));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      if (grid.isOccupiedInflated(a.x + t*dx, a.y + t*dy, a.z + t*dz)) return false;
    }
  }
  return true;
}

int main(int argc, char** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt)) return 1;

  std::cout << "Loading point cloud from: " << opt.pcd << std::endl;
  std::cout << "Voxel leaf size: " << opt.leaf << " m" << std::endl;
  std::cout << "Inflation radius: " << opt.radius << " m" << std::endl;
  if (opt.radius < opt.leaf) {
    std::cout << "WARNING: inflation radius < voxel size -> inflation has NO effect." << std::endl;
  }

  // ---- 1. load + downsample ---------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(opt.pcd, *raw_cloud) == -1) {
    std::cerr << "ERROR: could not load " << opt.pcd << std::endl;
    return 1;
  }
  std::cout << "Raw cloud points: " << raw_cloud->size() << std::endl;

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
  std::cout << "Bounding box: "
            << "x=[" << min_pt.x << ", " << max_pt.x << "] "
            << "y=[" << min_pt.y << ", " << max_pt.y << "] "
            << "z=[" << min_pt.z << ", " << max_pt.z << "]" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(raw_cloud);
  voxel_filter.setLeafSize(opt.leaf, opt.leaf, opt.leaf);
  voxel_filter.filter(*voxel_cloud);
  std::cout << "Downsampled cloud points: " << voxel_cloud->size() << std::endl;

  // ---- 2. occupancy grid + racks + inflation -----------------------------------
  VoxelOccupancyGrid grid(opt.leaf);
  grid.build(*voxel_cloud);
  std::cout << "Raw occupied voxels (from cloud): " << grid.voxelCount() << std::endl;

  if (opt.racks.empty()) {
    std::cout << "WARNING: no --rack given. The shelving racks are NOT filled, so paths may "
                 "fly through the gaps between shelf levels!" << std::endl;
  }
  for (const Rack& r : opt.racks) {
    const size_t added = grid.addBox(r.cx, r.cy, r.cz, r.sx, r.sy, r.sz, r.yaw);
    std::cout << "Rack filled: centre (" << r.cx << ", " << r.cy << ", " << r.cz << ") size "
              << r.sx << " x " << r.sy << " x " << r.sz << ", yaw " << r.yaw
              << " rad, +" << added << " voxels" << std::endl;
  }
  std::cout << "Raw occupied voxels (with racks): " << grid.voxelCount() << std::endl;

  grid.inflate(opt.radius);
  std::cout << "Inflated occupied voxels: " << grid.inflatedVoxelCount() << std::endl;

  const float wx = min_pt.x + opt.radius * 0.5f;
  const float wy = (min_pt.y + max_pt.y) / 2.0f;
  const float wz = 1.0f;
  std::cout << "Effect of inflation near a wall, at (" << wx << ", " << wy << ", " << wz << "): raw="
            << (grid.isOccupied(wx, wy, wz) ? "YES" : "no") << "  inflated="
            << (grid.isOccupiedInflated(wx, wy, wz) ? "YES" : "no") << std::endl;

  // ---- 3. bounded dense grid for A* ----------------------------------------------
  // The sparse set has no extent, so take the index range of the inflated map
  // (+1 voxel padding).  Everything outside it is treated as blocked by A*.
  const VoxelOccupancyGrid::IndexBounds b = grid.inflatedBounds();
  if (!b.valid) { std::cerr << "ERROR: map is empty" << std::endl; return 1; }
  const int64_t pad = 1;

  astar::GridInfo info;
  info.resolution = static_cast<double>(opt.leaf);
  info.origin_x = static_cast<double>(b.min_x - pad) * info.resolution;
  info.origin_y = static_cast<double>(b.min_y - pad) * info.resolution;
  info.origin_z = static_cast<double>(b.min_z - pad) * info.resolution;
  const int64_t sx = b.max_x - b.min_x + 1 + 2 * pad;
  const int64_t sy = b.max_y - b.min_y + 1 + 2 * pad;
  const int64_t sz = b.max_z - b.min_z + 1 + 2 * pad;

  const double cells = static_cast<double>(sx) * sy * sz;
  std::cout << "Dense planning grid: " << sx << " x " << sy << " x " << sz << " = "
            << cells << " voxels (~" << cells * 11.0 / 1e6 << " MB while planning)" << std::endl;
  if (cells > 4e8) {
    std::cerr << "ERROR: planning grid too large -- increase --leaf or crop outliers from the map." << std::endl;
    return 1;
  }
  info.size_x = static_cast<int>(sx);
  info.size_y = static_cast<int>(sy);
  info.size_z = static_cast<int>(sz);

  const int64_t ox = b.min_x - pad, oy = b.min_y - pad, oz = b.min_z - pad;
  astar::AStarPlanner planner(info,
      [&](int x, int y, int z) { return grid.isOccupiedInflatedIndex(ox + x, oy + y, oz + z); },
      opt.astar_params);

  // ---- 4. plan ---------------------------------------------------------------------
  if (!opt.has_start || !opt.has_goal) {
    std::cout << "No --start/--goal given -> map loaded and inflated, planning skipped." << std::endl;
    return 0;
  }

  std::cout << "Planning (" << opt.start.x << ", " << opt.start.y << ", " << opt.start.z << ") -> ("
            << opt.goal.x << ", " << opt.goal.y << ", " << opt.goal.z << ") ..." << std::endl;
  const astar::Result res = planner.plan(opt.start, opt.goal);

  std::cout << "Status: " << astar::toString(res.status) << std::endl;
  std::cout << "Planning time: " << res.planning_time_ms << " ms, expansions: " << res.expansions << std::endl;
  if (!res.success()) return 2;

  std::cout << "Path: " << res.raw_waypoints << " raw waypoints -> " << res.path.size()
            << " after simplification, length " << res.path_length_m << " m" << std::endl;
  for (const astar::Vec3& p : res.path) {
    std::cout << "  (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
  }
  const bool ok = pathIsCollisionFree(grid, res.path);
  std::cout << "Path collision-free against inflated map: " << (ok ? "YES" : "NO  <-- BUG") << std::endl;

  if (!opt.csv.empty()) {
    std::ofstream f(opt.csv);
    f << "x,y,z\n";
    for (const astar::Vec3& p : res.path) f << p.x << "," << p.y << "," << p.z << "\n";
    std::cout << "Path written to " << opt.csv << std::endl;
  }
  return ok ? 0 : 3;
}
