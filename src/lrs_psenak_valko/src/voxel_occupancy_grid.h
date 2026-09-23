// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
//
// voxel_occupancy_grid.h
//
// Sparse 3D occupancy grid (hash set of occupied voxels) with
//   * build()      : point cloud -> occupied voxels
//   * addBox()     : fill a SOLID box (used for the shelving racks, whose
//                    point-cloud surfaces leave free-looking gaps between levels)
//   * inflate()    : grow every occupied voxel into a sphere of a given radius
//   * queries      : world coordinates OR integer voxel indices
//   * bounds       : index range of the inflated map (needed by the dense A* grid)
//
// Only OCCUPIED voxels are stored.  "Free" simply means "not in the set".
// Typical call order:  build() -> addBox()* -> inflate().

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class VoxelOccupancyGrid
{
public:
  /// Inclusive integer index range of a set of voxels.
  struct IndexBounds
  {
    int64_t min_x = 0, min_y = 0, min_z = 0;
    int64_t max_x = -1, max_y = -1, max_z = -1;
    bool valid = false;
  };

  explicit VoxelOccupancyGrid(float leaf_size) : leaf_size_(leaf_size) {}

  void build(const pcl::PointCloud<pcl::PointXYZ>& cloud)
  {
    occupied_.clear();
    occupied_.reserve(cloud.size());
    for (const auto& point : cloud.points) {
      occupied_.insert(encodeVoxel(toIndex(point.x), toIndex(point.y), toIndex(point.z)));
    }
    inflated_ = occupied_;  // until inflate() is called
  }

  /// Fill a solid box (centre, full sizes, yaw about +Z in radians) with occupied
  /// voxels.  Use it for objects whose collision shape is a known solid box.
  /// Call BEFORE inflate() (inflate() restarts from the raw layer, which now
  /// contains the box).
  size_t addBox(float cx, float cy, float cz, float size_x, float size_y, float size_z, float yaw)
  {
    const float c = std::cos(yaw), s = std::sin(yaw);
    const float hx = 0.5f * size_x, hy = 0.5f * size_y, hz = 0.5f * size_z;
    // Axis-aligned bounding box of the rotated box.
    const float ex = std::fabs(c) * hx + std::fabs(s) * hy;
    const float ey = std::fabs(s) * hx + std::fabs(c) * hy;

    size_t added = 0;
    for (int64_t ix = toIndex(cx - ex); ix <= toIndex(cx + ex); ++ix) {
      for (int64_t iy = toIndex(cy - ey); iy <= toIndex(cy + ey); ++iy) {
        for (int64_t iz = toIndex(cz - hz); iz <= toIndex(cz + hz); ++iz) {
          // Voxel centre relative to the box centre, rotated into the box frame.
          const float dx = (ix + 0.5f) * leaf_size_ - cx;
          const float dy = (iy + 0.5f) * leaf_size_ - cy;
          const float dz = (iz + 0.5f) * leaf_size_ - cz;
          const float lx = c * dx + s * dy;
          const float ly = -s * dx + c * dy;
          if (std::fabs(lx) <= hx && std::fabs(ly) <= hy && std::fabs(dz) <= hz) {
            if (occupied_.insert(encodeVoxel(ix, iy, iz)).second) ++added;
          }
        }
      }
    }
    inflated_ = occupied_;
    return added;
  }

  /// Grow every occupied voxel into a sphere of radius_m (3D -- also up/down).
  /// Safe to call repeatedly with different radii (always restarts from occupied_).
  void inflate(float radius_m)
  {
    inflated_ = occupied_;
    if (radius_m <= 0.0f) return;

    // Work in voxel units and precompute the sphere's offsets ONCE
    // (the old version recomputed sqrt() for every voxel).  The small epsilon
    // avoids float rounding dropping the voxel exactly at the radius.
    const double r_vox = static_cast<double>(radius_m) / static_cast<double>(leaf_size_);
    const double r2 = r_vox * r_vox + 1e-6;
    const int R = static_cast<int>(std::floor(std::sqrt(r2)));

    std::vector<std::array<int, 3>> offsets;
    for (int dx = -R; dx <= R; ++dx)
      for (int dy = -R; dy <= R; ++dy)
        for (int dz = -R; dz <= R; ++dz)
          if (dx * dx + dy * dy + dz * dz <= r2) offsets.push_back({dx, dy, dz});

    for (const int64_t key : occupied_) {
      int64_t ix, iy, iz;
      decodeVoxel(key, ix, iy, iz);
      for (const auto& o : offsets) {
        inflated_.insert(encodeVoxel(ix + o[0], iy + o[1], iz + o[2]));
      }
    }
  }

  // ---- queries in world coordinates ----------------------------------------
  bool isOccupied(float x, float y, float z) const
  {
    return occupied_.count(encodeVoxel(toIndex(x), toIndex(y), toIndex(z))) > 0;
  }
  bool isOccupiedInflated(float x, float y, float z) const
  {
    return inflated_.count(encodeVoxel(toIndex(x), toIndex(y), toIndex(z))) > 0;
  }

  // ---- queries in voxel indices (used to fill the dense grid for A*) --------
  bool isOccupiedInflatedIndex(int64_t ix, int64_t iy, int64_t iz) const
  {
    return inflated_.count(encodeVoxel(ix, iy, iz)) > 0;
  }

  /// Index range covered by the inflated map (O(N), call once).
  IndexBounds inflatedBounds() const
  {
    IndexBounds b;
    for (const int64_t key : inflated_) {
      int64_t ix, iy, iz;
      decodeVoxel(key, ix, iy, iz);
      if (!b.valid) {
        b = {ix, iy, iz, ix, iy, iz, true};
      } else {
        b.min_x = std::min(b.min_x, ix); b.max_x = std::max(b.max_x, ix);
        b.min_y = std::min(b.min_y, iy); b.max_y = std::max(b.max_y, iy);
        b.min_z = std::min(b.min_z, iz); b.max_z = std::max(b.max_z, iz);
      }
    }
    return b;
  }

  float  leafSize() const { return leaf_size_; }
  size_t voxelCount() const { return occupied_.size(); }
  size_t inflatedVoxelCount() const { return inflated_.size(); }

private:
  int64_t toIndex(float coord) const
  {
    return static_cast<int64_t>(std::floor(coord / leaf_size_));
  }

  // Pack three voxel indices into one hashable key (OFFSET keeps them positive).
  static constexpr int64_t OFFSET = 1 << 20;
  static constexpr int64_t MASK21 = (1 << 21) - 1;

  static int64_t encodeVoxel(int64_t ix, int64_t iy, int64_t iz)
  {
    ix += OFFSET; iy += OFFSET; iz += OFFSET;
    return (ix << 42) ^ (iy << 21) ^ iz;
  }

  static void decodeVoxel(int64_t key, int64_t& ix, int64_t& iy, int64_t& iz)
  {
    iz = (key & MASK21) - OFFSET;
    iy = ((key >> 21) & MASK21) - OFFSET;
    ix = (key >> 42) - OFFSET;
  }

  float leaf_size_;
  std::unordered_set<int64_t> occupied_;   // raw obstacle voxels (+ filled boxes)
  std::unordered_set<int64_t> inflated_;   // occupied_ grown by the safety radius
};
