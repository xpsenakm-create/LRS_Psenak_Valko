// outdoor_math.h -- A1.3 outdoor mission: all the maths, no ROS dependency.
//
// Kept separate from the node so it can be unit-tested with plain g++
// (see outdoor_math_test.cpp) and so every formula has ONE place to defend.
//
// ANGLE CONVENTIONS used throughout this file (state them in the documentation!):
//   * "ENU yaw"  : radians (or deg where the name says Deg), 0 = East, +CCW.
//                  This is what /mavros/local_position/pose and the setpoints use.
//   * "heading"  : degrees, 0 = North, +CLOCKWISE (compass_hdg).
//   * relation   : yaw_ENU_deg = 90 - heading_deg, wrapped into (-180, 180].
//
// FRAME: local ENU, x = East, y = North, z = Up.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace outdoor
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

struct Vec2 { double x = 0.0, y = 0.0; };
struct Vec3 { double x = 0.0, y = 0.0, z = 0.0; };

inline double dist2(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline double dist3(const Vec3& a, const Vec3& b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// ----------------------------------------------------------------------------
// Angles
// ----------------------------------------------------------------------------

/// Wrap into (-pi, pi].
inline double wrapPi(double a)
{
  double r = std::remainder(a, 2.0 * kPi);  // [-pi, pi]
  if (r <= -kPi) r += 2.0 * kPi;
  return r;
}

/// Wrap into (-180, 180].
inline double wrapDeg180(double a)
{
  double r = std::remainder(a, 360.0);
  if (r <= -180.0) r += 360.0;
  return r;
}

/// Wrap into [0, 360).
inline double wrapDeg360(double a)
{
  double r = std::fmod(a, 360.0);
  if (r < 0.0) r += 360.0;
  return r;
}

/// compass heading (0 = N, clockwise, deg) -> ENU yaw (0 = E, CCW, deg) in (-180, 180].
inline double headingDegToEnuYawDeg(double heading_deg) { return wrapDeg180(90.0 - heading_deg); }

/// ENU yaw (deg) -> compass heading in [0, 360).
inline double enuYawDegToHeadingDeg(double yaw_deg) { return wrapDeg360(90.0 - yaw_deg); }

/// Return the angle equivalent to `angle` (mod 2*pi) that lies closest to `reference`.
/// Applying this to every new yaw sample keeps the sequence continuous (no +-2*pi jumps).
inline double unwrapNear(double reference, double angle)
{
  return reference + std::remainder(angle - reference, 2.0 * kPi);
}

/// Rotate `current` towards `target` by at most `max_step` radians (shortest way round).
/// `current` stays "unwrapped" (continuous), so repeated calls never spin the long way.
inline double slewAngle(double current, double target, double max_step)
{
  const double d = std::remainder(target - current, 2.0 * kPi);
  return current + std::clamp(d, -max_step, max_step);
}

/// ENU yaw of the direction from `from` to `to` (0 = East, CCW).
inline double bearingYaw(const Vec2& from, const Vec2& to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

// ----------------------------------------------------------------------------
// Geodetic <-> local ENU  (local tangent plane, WGS-84 radii at the origin)
//
//   east  = dlon * (N + h0) * cos(lat0)
//   north = dlat * (M + h0)
//
// N = prime-vertical radius, M = meridian radius, both evaluated ONCE at the
// origin.  This is a flat-Earth approximation: the error grows ~ d^2 / (2R), i.e.
// well below 1 mm at 50 m (measured against GeographicLib in the documentation).
// ----------------------------------------------------------------------------
struct Geodetic { double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0; };
struct Enu { double e = 0.0, n = 0.0, u = 0.0; };

class LocalTangentPlane
{
public:
  LocalTangentPlane() = default;
  LocalTangentPlane(double lat0_deg, double lon0_deg, double alt0_m)
  : origin_{lat0_deg, lon0_deg, alt0_m}
  {
    constexpr double a = 6378137.0;               // WGS-84 semi-major axis [m]
    constexpr double f = 1.0 / 298.257223563;     // WGS-84 flattening
    constexpr double e2 = f * (2.0 - f);          // eccentricity squared
    const double phi = lat0_deg * kDeg2Rad;
    const double s = std::sin(phi);
    const double w = 1.0 - e2 * s * s;
    const double N = a / std::sqrt(w);
    const double M = a * (1.0 - e2) / (w * std::sqrt(w));
    m_per_rad_lat_ = M + alt0_m;
    m_per_rad_lon_ = (N + alt0_m) * std::cos(phi);
  }

  Enu toEnu(const Geodetic& g) const
  {
    return {wrapDeg180(g.lon_deg - origin_.lon_deg) * kDeg2Rad * m_per_rad_lon_,
            (g.lat_deg - origin_.lat_deg) * kDeg2Rad * m_per_rad_lat_,
            g.alt_m - origin_.alt_m};
  }

  Geodetic toGeodetic(const Enu& p) const
  {
    return {origin_.lat_deg + p.n / m_per_rad_lat_ * kRad2Deg,
            origin_.lon_deg + p.e / m_per_rad_lon_ * kRad2Deg,
            origin_.alt_m + p.u};
  }

  const Geodetic& origin() const { return origin_; }

private:
  Geodetic origin_;
  double m_per_rad_lat_ = 1.0, m_per_rad_lon_ = 1.0;
};

// ----------------------------------------------------------------------------
// Figure-8 (lemniscate of Gerono), centred on the POI
//
//   x(th) = A sin(th)            dx/dth = A cos(th)
//   y(th) = A sin(th) cos(th)    dy/dth = A cos(2 th)
//   yaw   = atan2(dy/dth, dx/dth)      (tangent; independent of the time scaling)
//
// Speed = A * dth/dt * sqrt(cos^2(th) + cos^2(2 th)); the maximum is A*omega*sqrt(2)
// at th = 0 and th = pi (the crossing).
// ----------------------------------------------------------------------------
struct Figure8Sample { double x, y, dx, dy; };

inline Figure8Sample figure8(double A, double th)
{
  return {A * std::sin(th), A * std::sin(th) * std::cos(th), A * std::cos(th), A * std::cos(2.0 * th)};
}

inline double figure8Yaw(double th) { return std::atan2(std::cos(2.0 * th), std::cos(th)); }

inline double figure8Speed(double A, double omega, double th)
{
  return A * omega * std::sqrt(std::cos(th) * std::cos(th) + std::cos(2.0 * th) * std::cos(2.0 * th));
}

inline double figure8PeakSpeed(double A, double omega) { return A * omega * std::sqrt(2.0); }
inline double figure8MaxOmega(double A, double v_max) { return v_max / (A * std::sqrt(2.0)); }

/// Phase theta(s) after s seconds.  With ramp_s > 0 the angular rate rises linearly
/// from 0 to omega during the first ramp_s seconds (theta = omega*s^2 / (2*ramp)),
/// then stays at omega.  This removes the "lunge": at theta = 0 the curve's speed is
/// already its maximum, so a hard start would demand ~A*omega*sqrt(2) from standstill.
inline double figure8Phase(double omega, double ramp_s, double s)
{
  if (s <= 0.0) return 0.0;
  if (ramp_s <= 0.0) return omega * s;
  if (s < ramp_s) return 0.5 * omega * s * s / ramp_s;
  return omega * (s - 0.5 * ramp_s);
}

// ----------------------------------------------------------------------------
// Waypoint square around the POI.  Order (seen from above, North up), clockwise:
//   W1 = NW  ->  W2 = NE  ->  W3 = SE  ->  W4 = SW
// ----------------------------------------------------------------------------
inline std::array<Vec2, 4> squareCorners(const Vec2& poi, double side)
{
  const double h = 0.5 * side;
  return {{{poi.x - h, poi.y + h}, {poi.x + h, poi.y + h}, {poi.x + h, poi.y - h}, {poi.x - h, poi.y - h}}};
}

/// Largest horizontal distance from the LAUNCH point (0,0) of anything the mission
/// visits: the four corners and the whole figure-8.  Used for the geofence check.
inline double missionMaxRadius(const Vec2& poi, double side, double amplitude)
{
  double r = std::hypot(poi.x, poi.y);
  for (const Vec2& c : squareCorners(poi, side)) r = std::max(r, std::hypot(c.x, c.y));
  for (int i = 0; i < 720; ++i) {
    const double th = 2.0 * kPi * i / 720.0;
    const Figure8Sample s = figure8(amplitude, th);
    r = std::max(r, std::hypot(poi.x + s.x, poi.y + s.y));
  }
  return r;
}

}  // namespace outdoor
