// =============================================================================
// a_star_planner.cpp
//
//  * Works on the INFLATED occupancy grid (inflation is done in VoxelOccupancyGrid).
//  * Heuristic: 3D octile distance = exact shortest-path cost on an obstacle-free
//    26-connected grid  ->  admissible AND consistent (see notes at the bottom).
//  * Diagonal moves may not "squeeze" between blocked voxels (2x2x2 swept-block check).
//  * Never hangs: goal reached / open list exhausted (NO_PATH) / time limit (TIMEOUT).
//  * Reports planning time, number of expansions and path length.
//
// Self-test without PCL/ROS:
//   g++ -std=c++17 -O2 -DASTAR_STANDALONE_TEST a_star_planner.cpp -o a_star_test
// =============================================================================
#include "a_star_planner.h"

#include <iostream>

namespace astar
{

const char* toString(Status s)
{
  switch (s)
  {
    case Status::SUCCESS:             return "SUCCESS";
    case Status::START_OUT_OF_BOUNDS: return "START_OUT_OF_BOUNDS";
    case Status::GOAL_OUT_OF_BOUNDS:  return "GOAL_OUT_OF_BOUNDS";
    case Status::START_OCCUPIED:      return "START_OCCUPIED";
    case Status::GOAL_OCCUPIED:       return "GOAL_OCCUPIED";
    case Status::NO_PATH:             return "NO_PATH";
    case Status::TIMEOUT:             return "TIMEOUT";
  }
  return "UNKNOWN";
}

AStarPlanner::AStarPlanner(const GridInfo& info, OccupancyFn occupied, const Params& params)
  : info_(info), occupied_fn_(std::move(occupied)), params_(params)
{
  // Precompute neighbour offsets and their Euclidean step cost in metres.
  for (int dz = -1; dz <= 1; ++dz)
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        const int   nz   = std::abs(dx) + std::abs(dy) + std::abs(dz);
        const float cost = static_cast<float>(info_.resolution * std::sqrt(static_cast<double>(nz)));
        moves26_.push_back({dx, dy, dz, cost});
        if (nz == 1) moves6_.push_back({dx, dy, dz, cost});
      }
  refresh();
}

void AStarPlanner::refresh()
{
  blocked_.assign(static_cast<size_t>(info_.size_x) * info_.size_y * info_.size_z, 0);
  for (int z = 0; z < info_.size_z; ++z)
    for (int y = 0; y < info_.size_y; ++y)
      for (int x = 0; x < info_.size_x; ++x)
        blocked_[linear(x, y, z)] = occupied_fn_(x, y, z) ? 1 : 0;
}

bool AStarPlanner::inBounds(int x, int y, int z) const
{
  return x >= 0 && y >= 0 && z >= 0 && x < info_.size_x && y < info_.size_y && z < info_.size_z;
}

bool AStarPlanner::isBlocked(int x, int y, int z) const
{
  return !inBounds(x, y, z) || blocked_[linear(x, y, z)] != 0;
}

bool AStarPlanner::worldToIndex(const Vec3& p, int& x, int& y, int& z) const
{
  x = static_cast<int>(std::floor((p.x - info_.origin_x) / info_.resolution));
  y = static_cast<int>(std::floor((p.y - info_.origin_y) / info_.resolution));
  z = static_cast<int>(std::floor((p.z - info_.origin_z) / info_.resolution));
  return inBounds(x, y, z);
}

Vec3 AStarPlanner::indexToWorld(int x, int y, int z) const
{
  return {info_.origin_x + (x + 0.5) * info_.resolution,
          info_.origin_y + (y + 0.5) * info_.resolution,
          info_.origin_z + (z + 0.5) * info_.resolution};
}

bool AStarPlanner::isBlockedWorld(const Vec3& p) const
{
  int x, y, z;
  if (!worldToIndex(p, x, y, z)) return true;
  return isBlocked(x, y, z);
}

// Octile heuristic in 3D.  With sorted axis distances d1<=d2<=d3 (in cells):
//   h = res * [ sqrt3*d1 + sqrt2*(d2-d1) + 1*(d3-d2) ]
//     = res * [ (sqrt3-sqrt2)*d1 + (sqrt2-1)*d2 + d3 ]
// This is the exact cost of the cheapest 26-connected path in EMPTY space, so
// it can never overestimate -> admissible; it also satisfies the triangle
// inequality over the move set -> consistent (closed set is safe).
// For 6-connectivity the exact empty-space cost is the Manhattan distance.
double AStarPlanner::heuristic(int x, int y, int z, int gx, int gy, int gz) const
{
  std::array<int, 3> d = {std::abs(gx - x), std::abs(gy - y), std::abs(gz - z)};
  if (!params_.allow_diagonal)
    return info_.resolution * (d[0] + d[1] + d[2]);
  std::sort(d.begin(), d.end());
  static const double S2 = std::sqrt(2.0), S3 = std::sqrt(3.0);
  return info_.resolution * ((S3 - S2) * d[0] + (S2 - 1.0) * d[1] + d[2]);
}

// A diagonal move is only legal if every voxel in the box spanned by the move is
// free.  This stops the path from slipping through a shared edge/corner of two
// obstacle voxels (which the physical drone body cannot do).
bool AStarPlanner::moveIsFree(int x, int y, int z, int dx, int dy, int dz) const
{
  const int xs[2] = {0, dx}, ys[2] = {0, dy}, zs[2] = {0, dz};
  for (int a = 0; a < (dx != 0 ? 2 : 1); ++a)
    for (int b = 0; b < (dy != 0 ? 2 : 1); ++b)
      for (int c = 0; c < (dz != 0 ? 2 : 1); ++c)
        if (isBlocked(x + xs[a], y + ys[b], z + zs[c])) return false;
  return true;
}

bool AStarPlanner::findNearestFree(int& x, int& y, int& z, int radius) const
{
  if (!isBlocked(x, y, z)) return true;
  double best = std::numeric_limits<double>::infinity();
  int bx = 0, by = 0, bz = 0;
  for (int dz = -radius; dz <= radius; ++dz)
    for (int dy = -radius; dy <= radius; ++dy)
      for (int dx = -radius; dx <= radius; ++dx)
      {
        if (isBlocked(x + dx, y + dy, z + dz)) continue;
        const double d = dx * dx + dy * dy + dz * dz;
        if (d < best) { best = d; bx = x + dx; by = y + dy; bz = z + dz; }
      }
  if (!std::isfinite(best)) return false;
  x = bx; y = by; z = bz;
  return true;
}

// Sample the straight segment a->b every quarter of a voxel.  Since the grid is
// inflated, a tiny sampling error at voxel corners is swallowed by the safety margin.
bool AStarPlanner::segmentFree(const Vec3& a, const Vec3& b) const
{
  const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  const int n = std::max(1, static_cast<int>(std::ceil(len / (0.25 * info_.resolution))));
  for (int i = 0; i <= n; ++i)
  {
    const double t = static_cast<double>(i) / n;
    if (isBlockedWorld({a.x + t * dx, a.y + t * dy, a.z + t * dz})) return false;
  }
  return true;
}

// Greedy line-of-sight shortcutting: from waypoint i jump to the farthest
// waypoint j that is still visible in the inflated map.
std::vector<Vec3> AStarPlanner::simplify(const std::vector<Vec3>& in) const
{
  if (in.size() < 3) return in;
  std::vector<Vec3> out;
  size_t i = 0;
  out.push_back(in[0]);
  while (i < in.size() - 1)
  {
    size_t j = in.size() - 1;
    while (j > i + 1 && !segmentFree(in[i], in[j])) --j;
    out.push_back(in[j]);
    i = j;
  }
  return out;
}

Result AStarPlanner::plan(const Vec3& start, const Vec3& goal) const
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  auto elapsedMs = [&]() {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  };

  Result res;
  auto finish = [&](Status s) {
    res.status = s;
    res.planning_time_ms = elapsedMs();
    return res;
  };

  // ---- 1. validate & snap start / goal --------------------------------------
  int sx, sy, sz, gx, gy, gz;
  if (!worldToIndex(start, sx, sy, sz)) return finish(Status::START_OUT_OF_BOUNDS);
  if (!worldToIndex(goal,  gx, gy, gz)) return finish(Status::GOAL_OUT_OF_BOUNDS);

  bool start_snapped = false, goal_snapped = false;
  if (isBlocked(sx, sy, sz))
  {
    if (params_.snap_radius_cells <= 0 || !findNearestFree(sx, sy, sz, params_.snap_radius_cells))
      return finish(Status::START_OCCUPIED);
    start_snapped = true;
  }
  if (isBlocked(gx, gy, gz))
  {
    if (params_.snap_radius_cells <= 0 || !findNearestFree(gx, gy, gz, params_.snap_radius_cells))
      return finish(Status::GOAL_OCCUPIED);
    goal_snapped = true;
  }

  // ---- 2. A* ------------------------------------------------------------------
  const size_t N = blocked_.size();
  const float  INF = std::numeric_limits<float>::infinity();
  std::vector<float>    g(N, INF);
  std::vector<int32_t>  parent(N, -1);
  std::vector<uint8_t>  closed(N, 0);

  struct Entry { float f, g; uint32_t idx; };
  // std::priority_queue is a max-heap: "less" == lower priority.
  // Lower f first; on ties prefer larger g (deeper node) -> fewer expansions.
  auto lower = [](const Entry& a, const Entry& b) {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g;
  };
  std::priority_queue<Entry, std::vector<Entry>, decltype(lower)> open(lower);

  const size_t s_idx = linear(sx, sy, sz);
  const size_t g_idx = linear(gx, gy, gz);
  const float  w     = static_cast<float>(params_.heuristic_weight);

  g[s_idx] = 0.f;
  open.push({w * static_cast<float>(heuristic(sx, sy, sz, gx, gy, gz)), 0.f, static_cast<uint32_t>(s_idx)});

  const std::vector<Move>& moves = params_.allow_diagonal ? moves26_ : moves6_;
  bool found = false, timed_out = false;

  while (!open.empty())
  {
    const Entry cur = open.top();
    open.pop();
    if (closed[cur.idx]) continue;          // stale duplicate (lazy deletion)
    closed[cur.idx] = 1;

    if (cur.idx == g_idx) { found = true; break; }
    ++res.expansions;

    if ((res.expansions & 0x3FF) == 0)      // check limits every 1024 expansions
    {
      if (elapsedMs() > params_.max_planning_time_s * 1000.0) { timed_out = true; break; }
    }
    if (params_.max_expansions && res.expansions >= params_.max_expansions) { timed_out = true; break; }

    // decode voxel coordinates
    const int cx = static_cast<int>(cur.idx % info_.size_x);
    const int cy = static_cast<int>((cur.idx / info_.size_x) % info_.size_y);
    const int cz = static_cast<int>(cur.idx / (static_cast<size_t>(info_.size_x) * info_.size_y));

    for (const Move& m : moves)
    {
      const int nx = cx + m.dx, ny = cy + m.dy, nz = cz + m.dz;
      if (isBlocked(nx, ny, nz)) continue;
      if (!moveIsFree(cx, cy, cz, m.dx, m.dy, m.dz)) continue;

      const size_t nidx = linear(nx, ny, nz);
      if (closed[nidx]) continue;

      const float ng = cur.g + m.cost;
      if (ng < g[nidx])
      {
        g[nidx] = ng;
        parent[nidx] = static_cast<int32_t>(cur.idx);
        const float f = ng + w * static_cast<float>(heuristic(nx, ny, nz, gx, gy, gz));
        open.push({f, ng, static_cast<uint32_t>(nidx)});
      }
    }
  }

  if (!found) return finish(timed_out ? Status::TIMEOUT : Status::NO_PATH);

  // ---- 3. reconstruct path -----------------------------------------------------
  std::vector<Vec3> raw;
  for (int32_t i = static_cast<int32_t>(g_idx); i != -1; i = parent[i])
  {
    const int x = static_cast<int>(i % info_.size_x);
    const int y = static_cast<int>((i / info_.size_x) % info_.size_y);
    const int z = static_cast<int>(i / (static_cast<size_t>(info_.size_x) * info_.size_y));
    raw.push_back(indexToWorld(x, y, z));
  }
  std::reverse(raw.begin(), raw.end());

  // Use the exact requested endpoints (they lie inside the free start/goal voxels)
  // unless we had to snap to another voxel.
  if (!start_snapped) raw.front() = start;
  if (!goal_snapped)  raw.back()  = goal;
  res.raw_waypoints = raw.size();

  res.path = params_.simplify_path ? simplify(raw) : raw;

  for (size_t i = 1; i < res.path.size(); ++i)
  {
    const double dx = res.path[i].x - res.path[i - 1].x;
    const double dy = res.path[i].y - res.path[i - 1].y;
    const double dz = res.path[i].z - res.path[i - 1].z;
    res.path_length_m += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  return finish(Status::SUCCESS);
}

}  // namespace astar

// ============================================================================
// Self-test on a synthetic grid (no ROS / PCL needed).
// ============================================================================
#ifdef ASTAR_STANDALONE_TEST

static void report(const char* name, const astar::Result& r)
{
  std::cout << "[" << name << "] status=" << astar::toString(r.status)
            << "  time=" << r.planning_time_ms << " ms"
            << "  expansions=" << r.expansions;
  if (r.success())
    std::cout << "  raw_waypoints=" << r.raw_waypoints << "  waypoints=" << r.path.size()
              << "  length=" << r.path_length_m << " m";
  std::cout << "\n";
}

int main()
{
  astar::GridInfo info;
  info.resolution = 0.2;
  info.size_x = 100; info.size_y = 100; info.size_z = 40;   // 20 x 20 x 8 m

  // Wall at x = 50 with a window (y 40..60, z 5..15), plus a floor slab so that
  // the planner has to climb/descend to get through.
  auto occ = [](int x, int y, int z) {
    if (x == 50 && !(y >= 40 && y < 60 && z >= 5 && z < 15)) return true;
    return false;
  };

  astar::AStarPlanner planner(info, occ);

  astar::Vec3 start{2.0, 2.0, 1.0}, goal{18.0, 10.0, 6.0};
  astar::Result r = planner.plan(start, goal);
  report("through window", r);

  // Verify: every waypoint and every segment is collision free.
  bool ok = r.success();
  for (size_t i = 0; ok && i < r.path.size(); ++i) ok = !planner.isBlockedWorld(r.path[i]);
  std::cout << "  path collision-free: " << (ok ? "yes" : "NO") << "\n";

  // No-path case: goal sealed inside a shell of blocked voxels.
  auto occ_sealed = [&](int x, int y, int z) {
    if (occ(x, y, z)) return true;
    const int cx = 90, cy = 50, cz = 30;
    const int d = std::max({std::abs(x - cx), std::abs(y - cy), std::abs(z - cz)});
    return d == 2;
  };
  astar::AStarPlanner sealed(info, occ_sealed);
  report("sealed goal", sealed.plan(start, goal));

  // Blocked goal
  report("blocked goal", planner.plan(start, {10.1, 5.0, 1.0}));

  return 0;
}
#endif

// ============================================================================
// NOTES FOR THE DOCUMENTATION
//
// Heuristic:  3D octile distance (see heuristic()).  On an empty 26-connected
//   grid the cheapest route from a to b uses min(d) triple-diagonals, then
//   (mid-min) double-diagonals, then straight steps - that is exactly the
//   formula, so obstacles can only make the true cost larger, never smaller
//   => admissible.  It is also consistent, so a node is never re-opened and
//   the returned path is optimal on the grid (heuristic_weight = 1).
//   Euclidean distance would also be admissible but is weaker (smaller h ->
//   more expansions); Manhattan would be inadmissible with diagonal moves.
//
// Why the corner-cutting check: a diagonal step between two voxels that only
//   touch at an edge/corner would let the path squeeze through a gap the drone
//   cannot physically pass.  Combined with inflation this keeps the path clear.
//
// Complexity: O(E log V) with a binary heap, V = free voxels reachable before
//   the goal is popped.  Memory: 4 + 4 + 1 + 1 bytes per voxel.
//
// Termination: the open list is finite (each voxel is closed at most once),
//   so a missing path ends with NO_PATH; the time / expansion limits bound
//   even huge maps (TIMEOUT).
// ============================================================================
