#include "mission_model.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace mission
{
namespace
{

std::string trim(const std::string& value)
{
  const size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

double parseNumber(const std::string& text, size_t line, const char* field)
{
  size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(trim(text), &parsed);
  } catch (const std::exception&) {
    throw std::runtime_error("mission line " + std::to_string(line) + ": invalid " + field);
  }
  if (parsed != trim(text).size() || !std::isfinite(value))
    throw std::runtime_error("mission line " + std::to_string(line) + ": invalid " + field);
  return value;
}

}  // namespace

Mission loadCsv(const std::string& path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open mission file: " + path);

  Mission mission;
  std::string row;
  size_t line_number = 0;
  bool header_seen = false;
  while (std::getline(input, row)) {
    ++line_number;
    row = trim(row);
    if (row.empty() || row.front() == '#') continue;

    std::vector<std::string> fields;
    std::stringstream row_stream(row);
    std::string field;
    while (std::getline(row_stream, field, ',')) fields.push_back(trim(field));
    if (fields.size() != 5)
      throw std::runtime_error("mission line " + std::to_string(line_number) +
                               ": expected x,y,z,precision,task");

    if (!header_seen && fields[0] == "x" && fields[1] == "y" && fields[2] == "z") {
      header_seen = true;
      continue;
    }
    header_seen = true;

    Waypoint waypoint;
    waypoint.x = parseNumber(fields[0], line_number, "x");
    waypoint.y = parseNumber(fields[1], line_number, "y");
    waypoint.z = parseNumber(fields[2], line_number, "z");
    if (fields[3] == "soft") waypoint.precision = Precision::SOFT;
    else if (fields[3] == "hard") waypoint.precision = Precision::HARD;
    else throw std::runtime_error("mission line " + std::to_string(line_number) +
                                  ": precision must be soft or hard");

    const std::string& task = fields[4];
    if (task.empty() || task == "-") waypoint.task.type = TaskType::NONE;
    else if (task == "takeoff") waypoint.task.type = TaskType::TAKEOFF;
    else if (task == "land") waypoint.task.type = TaskType::LAND;
    else if (task == "landtakeoff") waypoint.task.type = TaskType::LAND_TAKEOFF;
    else if (task.rfind("yaw", 0) == 0) {
      waypoint.task.type = TaskType::YAW;
      waypoint.task.yaw_degrees = parseNumber(task.substr(3), line_number, "yaw angle");
    } else {
      throw std::runtime_error("mission line " + std::to_string(line_number) +
                               ": unsupported task '" + task + "'");
    }
    mission.push_back(waypoint);
  }

  if (mission.empty()) throw std::runtime_error("mission file contains no waypoints: " + path);
  return mission;
}

const char* toString(Precision precision)
{
  return precision == Precision::HARD ? "hard" : "soft";
}

const char* toString(TaskType task)
{
  switch (task) {
    case TaskType::NONE: return "-";
    case TaskType::TAKEOFF: return "takeoff";
    case TaskType::LAND: return "land";
    case TaskType::LAND_TAKEOFF: return "landtakeoff";
    case TaskType::YAW: return "yaw";
  }
  return "unknown";
}

}  // namespace mission