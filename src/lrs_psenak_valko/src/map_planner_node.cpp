// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
//
// A1.1 bod 1+2 -- Map loading a Obstacle inflation.
// Standalone program: reads a .pcd point cloud, downsamples it into a voxel
// grid, builds a fast occupancy query structure, then inflates every
// occupied voxel by a configurable safety radius.

#include <iostream>
#include <string>
#include <unordered_set>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>  // pcl::getMinMax3D

// A 3D occupancy grid built from a downsampled point cloud, with a second
// "inflated" layer used for planning (inflate() must be called once after
// build(); isOccupiedInflated() then reflects the safety margin).
class VoxelOccupancyGrid
{
public:
  explicit VoxelOccupancyGrid(float leaf_size) : leaf_size_(leaf_size) {}

  void build(const pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    occupied_.clear();
    occupied_.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      occupied_.insert(encodeVoxel(toIndex(point.x), toIndex(point.y), toIndex(point.z)));
    }
    // Until inflate() is called, the inflated layer is just the raw layer.
    inflated_ = occupied_;
  }

  // Grow every occupied voxel into a sphere of the given radius (metres).
  // Must be called after build(). Safe to call again with a different
  // radius -- it always starts fresh from the raw occupied_ set.
  void inflate(float radius_m)
  {
    inflated_ = occupied_;
    if (radius_m <= 0.0f) return;

    // How many voxel-steps the radius spans, rounded up so we do not miss
    // a voxel whose centre is just inside the sphere.
    const int voxel_radius = static_cast<int>(std::ceil(radius_m / leaf_size_));

    for (const int64_t key : occupied_) {
      int64_t ix, iy, iz;
      decodeVoxel(key, ix, iy, iz);

      for (int dx = -voxel_radius; dx <= voxel_radius; ++dx) {
        for (int dy = -voxel_radius; dy <= voxel_radius; ++dy) {
          for (int dz = -voxel_radius; dz <= voxel_radius; ++dz) {
            // Euclidean check -> a sphere, not the cheaper-but-wrong cube.
            const float dist = leaf_size_ * std::sqrt(
                static_cast<float>(dx * dx + dy * dy + dz * dz));
            if (dist <= radius_m) {
              inflated_.insert(encodeVoxel(ix + dx, iy + dy, iz + dz));
            }
          }
        }
      }
    }
  }

  // Raw occupancy (no safety margin) -- mostly useful for debugging/printing.
  bool isOccupied(float x, float y, float z) const
  {
    return occupied_.count(encodeVoxel(toIndex(x), toIndex(y), toIndex(z))) > 0;
  }

  // What the planner must actually use: occupancy including the safety margin.
  bool isOccupiedInflated(float x, float y, float z) const
  {
    return inflated_.count(encodeVoxel(toIndex(x), toIndex(y), toIndex(z))) > 0;
  }

  size_t voxelCount() const { return occupied_.size(); }
  size_t inflatedVoxelCount() const { return inflated_.size(); }

private:
  // Continuous coordinate -> integer voxel index along one axis.
  int64_t toIndex(float coord) const
  {
    return static_cast<int64_t>(std::floor(coord / leaf_size_));
  }

  // Pack three voxel indices into one hashable key. OFFSET keeps every
  // index non-negative (the map spans both sides of the origin) so the
  // three fields cannot bleed into each other after the bit shifts.
  static constexpr int64_t OFFSET = 1 << 20;
  static constexpr int64_t MASK21 = (1 << 21) - 1;  // low 21 bits

  int64_t encodeVoxel(int64_t ix, int64_t iy, int64_t iz) const
  {
    ix += OFFSET; iy += OFFSET; iz += OFFSET;
    return (ix << 42) ^ (iy << 21) ^ iz;
  }

  // Inverse of encodeVoxel: recovers the original (ix, iy, iz), needed to
  // walk the neighbours of a voxel during inflate().
  void decodeVoxel(int64_t key, int64_t& ix, int64_t& iy, int64_t& iz) const
  {
    iz = (key & MASK21) - OFFSET;
    iy = ((key >> 21) & MASK21) - OFFSET;
    ix = (key >> 42) - OFFSET;
  }

  float leaf_size_;
  std::unordered_set<int64_t> occupied_;
  std::unordered_set<int64_t> inflated_;
};

int main(int argc, char** argv)
{
  std::string pcd_path = "maps/FEI_LRS_PCD/map.pcd";
  float leaf_size = 0.15f;
  float inflation_radius = 0.3f;  // drone radius (~0.2 m) + margin -- justify in docs

  if (argc >= 2) pcd_path = argv[1];
  if (argc >= 3) leaf_size = std::stof(argv[2]);
  if (argc >= 4) inflation_radius = std::stof(argv[3]);

  std::cout << "Loading point cloud from: " << pcd_path << std::endl;
  std::cout << "Voxel leaf size: " << leaf_size << " m" << std::endl;
  std::cout << "Inflation radius: " << inflation_radius << " m" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw_cloud) == -1) {
    std::cerr << "ERROR: could not load " << pcd_path << std::endl;
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
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*voxel_cloud);
  std::cout << "Downsampled cloud points (== voxel count): "
            << voxel_cloud->size() << std::endl;

  VoxelOccupancyGrid grid(leaf_size);
  grid.build(*voxel_cloud);
  std::cout << "Raw occupied voxels: " << grid.voxelCount() << std::endl;

  grid.inflate(inflation_radius);
  std::cout << "Inflated occupied voxels: " << grid.inflatedVoxelCount() << std::endl;

  // Pick a point that sits just outside the raw bounding-box wall to show
  // the inflation actually changes the answer near a surface.
  const float near_wall_x = min_pt.x + inflation_radius * 0.5f;
  const float near_wall_y = (min_pt.y + max_pt.y) / 2.0f;
  const float near_wall_z = 1.0f;

  std::cout << "Effect of inflation near a wall, at ("
            << near_wall_x << ", " << near_wall_y << ", " << near_wall_z << "):" << std::endl;
  std::cout << "  raw isOccupied      = "
            << (grid.isOccupied(near_wall_x, near_wall_y, near_wall_z) ? "YES" : "no") << std::endl;
  std::cout << "  inflated isOccupied = "
            << (grid.isOccupiedInflated(near_wall_x, near_wall_y, near_wall_z) ? "YES" : "no") << std::endl;

  return 0;
}
