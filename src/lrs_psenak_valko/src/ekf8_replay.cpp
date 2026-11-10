// ekf8_replay.cpp -- offline replay of sensor events through the SAME SensorFusion object the
// ROS estimator uses.  Driven by tools/a2_offline_sim.py (which generates the events with the
// teacher's own sensor_model.py), it lets Q, R and the gate be tuned on many seeds in seconds.
//
// Usage:  ekf8_replay events.csv out_dir [key=value ...]
//   events.csv lines (in ARRIVAL order):  imu,t,w_z | gps,t,x,y,z | vo,t,yaw,vbx,vby,vbz
//   writes out_dir/estimate.csv (after every IMU sample) and out_dir/innovation.csv
//   keys: see applyOverride() -- the same names as in config/localization.yaml.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "ekf8.h"

using namespace loc;

static bool applyOverride(FusionConfig& c, const std::string& key, double v)
{
  const std::map<std::string, std::function<void(double)>> setters = {
    {"accel_xy", [&](double x) { c.q.accel_xy = x; }},
    {"accel_z", [&](double x) { c.q.accel_z = x; }},
    {"yaw_rate", [&](double x) { c.q.yaw_rate = x; }},
    {"gyro_bias_rw", [&](double x) { c.q.gyro_bias_rw = x; }},
    {"std0_pos_xy", [&](double x) { c.std0_pos_xy = x; }},
    {"std0_pos_z", [&](double x) { c.std0_pos_z = x; }},
    {"std0_vel", [&](double x) { c.std0_vel = x; }},
    {"std0_yaw", [&](double x) { c.std0_yaw = x; }},
    {"std0_bias", [&](double x) { c.std0_bias = x; }},
    {"gps_std_xy", [&](double x) { c.gps_std.x() = c.gps_std.y() = x; }},
    {"gps_std_z", [&](double x) { c.gps_std.z() = x; }},
    {"vo_velocity_std", [&](double x) { c.vo_velocity_std = x; }},
    {"vo_yaw_std", [&](double x) { c.vo_yaw_std = x; }},
    {"use_vo_velocity", [&](double x) { c.use_vo_velocity = x != 0.0; }},
    {"use_vo_yaw", [&](double x) { c.use_vo_yaw = x != 0.0; }},
    {"vo_velocity_ekf", [&](double x) { c.vo_velocity_ekf = x != 0.0; }},
    {"gps_gate", [&](double x) { c.gps_gate = x; }},
    {"vo_velocity_gate", [&](double x) { c.vo_velocity_gate = x; }},
    {"vo_yaw_gate", [&](double x) { c.vo_yaw_gate = x; }},
    {"gate_max_consecutive", [&](double x) { c.gate_max_consecutive = static_cast<int>(x); }},
    {"max_lag_s", [&](double x) { c.max_lag_s = x; }},
    {"zaru_enabled", [&](double x) { c.zaru_enabled = x != 0.0; }},
    {"zaru_rate_std", [&](double x) { c.zaru_rate_std = x; }},
    {"zaru_motion_speed", [&](double x) { c.zaru_motion_speed = x; }},
    {"zaru_gate", [&](double x) { c.zaru_gate = x; }},
  };
  const auto it = setters.find(key);
  if (it == setters.end()) return false;
  it->second(v);
  return true;
}

static void writeInnovation(std::ofstream& out, double t, Sensor s, const UpdateResult& r)
{
  out << t << "," << static_cast<int>(s) << "," << static_cast<int>(r.status) << "," << r.d2 << ","
      << r.innovation[0] << "," << r.innovation[1] << "," << r.innovation[2] << "," << (r.late ? 1 : 0) << "\n";
}

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s events.csv out_dir [key=value ...]\n", argv[0]);
    return 2;
  }
  FusionConfig cfg;
  for (int i = 3; i < argc; ++i) {
    const std::string kv = argv[i];
    const auto eq = kv.find('=');
    if (eq == std::string::npos || !applyOverride(cfg, kv.substr(0, eq), std::stod(kv.substr(eq + 1)))) {
      std::fprintf(stderr, "unknown override: %s\n", argv[i]);
      return 2;
    }
  }
  std::ifstream in(argv[1]);
  if (!in) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 1; }
  const std::string dir = argv[2];
  std::ofstream est(dir + "/estimate.csv"), inn(dir + "/innovation.csv");
  if (!est || !inn) { std::fprintf(stderr, "cannot write into %s\n", dir.c_str()); return 1; }
  est << "t,x,y,z,vx,vy,vz,yaw,bias,sx,sy,sz,svx,svy,svz,syaw,sbias\n";
  inn << "t,sensor,status,d2,r0,r1,r2,late\n";
  est.precision(9);
  inn.precision(9);

  SensorFusion fusion(cfg);
  std::string line;
  std::vector<double> v;
  long dropped = 0;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string kind, field;
    std::getline(ss, kind, ',');
    v.clear();
    while (std::getline(ss, field, ',')) v.push_back(std::stod(field));
    if (v.empty()) continue;
    const double t = v[0];
    if (kind == "imu" && v.size() >= 2) {
      if (fusion.imu(t, v[1]) != SensorFusion::Input::USED) { ++dropped; continue; }
      const Vec8& x = fusion.ekf().x();
      const Vec8 s = fusion.ekf().P().diagonal().cwiseSqrt();
      est << fusion.ekf().time();
      for (int i = 0; i < N; ++i) est << "," << x(i);
      for (int i = 0; i < N; ++i) est << "," << s(i);
      est << "\n";
    } else if (kind == "gps" && v.size() >= 4) {
      UpdateResult r;
      if (fusion.gps(t, Vec3(v[1], v[2], v[3]), nullptr, &r) == SensorFusion::Input::USED)
        writeInnovation(inn, t, Sensor::GPS, r);
    } else if (kind == "vo" && v.size() >= 5) {
      UpdateResult ry, rv;
      if (fusion.vo(t, v[1], Vec3(v[2], v[3], v[4]), &ry, &rv) == SensorFusion::Input::USED) {
        if (cfg.use_vo_yaw) writeInnovation(inn, t, Sensor::VO_YAW, ry);
        if (cfg.use_vo_velocity) writeInnovation(inn, t, Sensor::VO_VELOCITY, rv);
      }
    }
  }
  std::printf("replay done, %ld IMU samples not used (before init / duplicate / too old)\n", dropped);
  return 0;
}
