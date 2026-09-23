// a_star_planner.h -- 3D A* over a voxel grid (26-connected) for a quadrotor.
// Independent of ROS/PCL: it only needs the grid geometry and an occupancy
// callback for the INFLATED map.  Implementation: a_star_planner.cpp
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <vector>

namespace astar
{

struct Vec3
{
  double x = 0.0, y = 0.0, z = 0.0;
};

/// Geometry of the (already inflated) voxel grid.
struct GridInfo
{
  double resolution = 0.1;                 ///< voxel edge length [m]
  double origin_x = 0, origin_y = 0, origin_z = 0;  ///< world coords of the MIN corner of voxel (0,0,0)
  int size_x = 0, size_y = 0, size_z = 0;  ///< number of voxels per axis
};

/// Returns true if voxel (ix,iy,iz) is occupied in the INFLATED map.
/// Only called with indices inside [0,size).
using OccupancyFn = std::function<bool(int, int, int)>;

struct Params
{
  bool   allow_diagonal      = true;   ///< true: 26-connected, false: 6-connected
  double heuristic_weight    = 1.0;    ///< 1.0 = optimal; >1 = faster, bounded-suboptimal (weighted A*)
  double max_planning_time_s = 5.0;    ///< hard wall-clock limit
  size_t max_expansions      = 0;      ///< 0 = unlimited
  int    snap_radius_cells   = 0;      ///< if start/goal is blocked, search this far for the nearest free voxel (0 = off)
  bool   simplify_path       = true;   ///< line-of-sight shortcutting (checked against the inflated map)
};

enum class Status
{
  SUCCESS,
  START_OUT_OF_BOUNDS,
  GOAL_OUT_OF_BOUNDS,
  START_OCCUPIED,
  GOAL_OCCUPIED,
  NO_PATH,   ///< search space exhausted: goal provably unreachable on this grid
  TIMEOUT    ///< time / expansion limit hit before an answer was found
};

const char* toString(Status s);

struct Result
{
  Status            status = Status::NO_PATH;
  std::vector<Vec3> path;                 ///< start -> goal, world coordinates [m]
  double            planning_time_ms = 0; ///< wall-clock time of plan()
  size_t            expansions = 0;       ///< nodes popped from the open list
  double            path_length_m = 0;    ///< sum of segment lengths of `path`
  size_t            raw_waypoints = 0;    ///< waypoints before simplification

  bool success() const { return status == Status::SUCCESS; }
};

class AStarPlanner
{
public:
  /// The occupancy function is sampled ONCE here into a flat byte array so that
  /// every query during the search is O(1).  Call refresh() if the map changes.
  AStarPlanner(const GridInfo& info, OccupancyFn occupied, const Params& params = Params());

  void refresh();                         ///< re-sample the occupancy function
  void setParams(const Params& p) { params_ = p; }
  const Params&   params() const { return params_; }
  const GridInfo& info()   const { return info_; }

  Result plan(const Vec3& start, const Vec3& goal) const;

  // --- helpers, also handy from map_planner / RViz publishing code -----------
  bool inBounds(int x, int y, int z) const;
  bool isBlocked(int x, int y, int z) const;       ///< out of bounds counts as blocked
  bool isBlockedWorld(const Vec3& p) const;        ///< the "is this point free?" query
  bool worldToIndex(const Vec3& p, int& x, int& y, int& z) const;  ///< false if outside grid
  Vec3 indexToWorld(int x, int y, int z) const;    ///< voxel CENTRE

private:
  size_t linear(int x, int y, int z) const
  {
    return (static_cast<size_t>(z) * info_.size_y + y) * info_.size_x + x;
  }

  bool segmentFree(const Vec3& a, const Vec3& b) const;
  bool moveIsFree(int x, int y, int z, int dx, int dy, int dz) const;
  bool findNearestFree(int& x, int& y, int& z, int radius) const;
  double heuristic(int x, int y, int z, int gx, int gy, int gz) const;
  std::vector<Vec3> simplify(const std::vector<Vec3>& in) const;

  struct Move { int dx, dy, dz; float cost; };

  GridInfo               info_;
  OccupancyFn            occupied_fn_;
  Params                 params_;
  std::vector<uint8_t>   blocked_;   ///< flat copy of the inflated grid
  std::vector<Move>      moves26_;   ///< precomputed neighbour offsets + step cost [m]
  std::vector<Move>      moves6_;
};

}  // namespace astar

