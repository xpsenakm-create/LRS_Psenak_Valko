#pragma once

#include <string>
#include <vector>

namespace mission
{

enum class Precision
{
  SOFT,
  HARD
};

enum class TaskType
{
  NONE,
  TAKEOFF,
  LAND,
  LAND_TAKEOFF,
  YAW
};

struct Task
{
  TaskType type = TaskType::NONE;
  double yaw_degrees = 0.0;
};

struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  Precision precision = Precision::SOFT;
  Task task;
};

using Mission = std::vector<Waypoint>;

Mission loadCsv(const std::string& path);
const char* toString(Precision precision);
const char* toString(TaskType task);

}  // namespace mission