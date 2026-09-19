// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
//
// A1.1 bod 1 -- Map loading.
// Standalone program (no ROS runtime needed): reads a .pcd point cloud,
// downsamples it into a voxel grid, and builds a fast "is this point free?"
// occupancy query structure. Run without any simulator, per the assignment hint.

#include <iostream>
#include <string>
#include <unordered_set>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>  // pcl::getMinMax3D

// A simple 3D occupancy grid built from a downsampled point cloud.
// "Occupied" means: there exists at least one surface point inside that voxel.
class VoxelOccupancyGrid
{
public:
  explicit VoxelOccupancyGrid(float leaf_size) : leaf_size_(leaf_size) {}

  // Build the occupancy set from an already-downsampled cloud.
  void build(const pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    occupied_.clear();
    occupied_.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      occupied_.insert(voxelKey(point.x, point.y, point.z));
    }
  }

  // Is the voxel containing (x, y, z) occupied?
  bool isOccupied(float x, float y, float z) const
  {
    return occupied_.count(voxelKey(x, y, z)) > 0;
  }

  size_t voxelCount() const { return occupied_.size(); }

private:
  // Map a continuous 3D point to a single integer key identifying its voxel.
  // Voxel indices are computed by flooring (coordinate / leaf_size), then
  // packed into one 64-bit integer so std::unordered_set can hash it in O(1).
  int64_t voxelKey(float x, float y, float z) const
  {
    // Offset by a large constant before casting to int so negative
    // coordinates (the map extends on both sides of the origin) do not
    // collide with positive ones after the cast.
    constexpr int64_t OFFSET = 1 << 20;  // supports +/- ~1,000,000 voxels per axis
    int64_t ix = static_cast<int64_t>(std::floor(x / leaf_size_)) + OFFSET;
    int64_t iy = static_cast<int64_t>(std::floor(y / leaf_size_)) + OFFSET;
    int64_t iz = static_cast<int64_t>(std::floor(z / leaf_size_)) + OFFSET;
    // Pack into one int64: 21 bits per axis is plenty for OFFSET's range.
    return (ix << 42) ^ (iy << 21) ^ iz;
  }

  float leaf_size_;
  std::unordered_set<int64_t> occupied_;
};

int main(int argc, char** argv)
{
  // Path to the .pcd file, and the voxel leaf size in metres, both
  // overridable from the command line -- see A1.1: "not hardcoded".
  std::string pcd_path = "maps/FEI_LRS_PCD/map.pcd";
  float leaf_size = 0.15f;  // 15 cm voxels -- a starting point, tune later

  if (argc >= 2) pcd_path = argv[1];
  if (argc >= 3) leaf_size = std::stof(argv[2]);

  std::cout << "Loading point cloud from: " << pcd_path << std::endl;
  std::cout << "Voxel leaf size: " << leaf_size << " m" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw_cloud) == -1) {
    std::cerr << "ERROR: could not load " << pcd_path << std::endl;
    return 1;
  }
  std::cout << "Raw cloud points: " << raw_cloud->size() << std::endl;

  // Bounding box of the raw map -- tells us where (0,0,0) sits relative to
  // the hangar, and how big the space we plan in actually is.
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*raw_cloud, min_pt, max_pt);
  std::cout << "Bounding box: "
            << "x=[" << min_pt.x << ", " << max_pt.x << "] "
            << "y=[" << min_pt.y << ", " << max_pt.y << "] "
            << "z=[" << min_pt.z << ", " << max_pt.z << "]" << std::endl;

  // Downsample: this is what turns "raw points" into "something you can
  // plan in" per the assignment -- planning over raw points is too slow.
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(raw_cloud);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*voxel_cloud);
  std::cout << "Downsampled cloud points (== voxel count): "
            << voxel_cloud->size() << std::endl;

  // Build the occupancy query structure and try a few sample queries.
  VoxelOccupancyGrid grid(leaf_size);
  grid.build(*voxel_cloud);

  auto report_query = [&grid](float x, float y, float z) {
    std::cout << "  occupied(" << x << ", " << y << ", " << z << ") = "
              << (grid.isOccupied(x, y, z) ? "YES" : "no") << std::endl;
  };

  std::cout << "Sample occupancy queries:" << std::endl;
  report_query(0.0f, 0.0f, 1.0f);            // origin-ish, 1 m up
  report_query(min_pt.x, min_pt.y, min_pt.z); // a corner of the bounding box
  report_query((min_pt.x + max_pt.x) / 2.0f,
               (min_pt.y + max_pt.y) / 2.0f,
               (min_pt.z + max_pt.z) / 2.0f); // dead centre of the map

  return 0;
}
