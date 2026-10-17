cmake_minimum_required(VERSION 3.8)
project(lrs_psenak_valko)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# A* je bez optimalizácie približne 10x pomalší
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# --- A1.1: map + A* planner -------------------------------------------------
add_executable(map_planner_node
  src/map_planner_node.cpp
  src/a_star_planner.cpp)
target_include_directories(map_planner_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS})
target_compile_definitions(map_planner_node PRIVATE ${PCL_DEFINITIONS})
target_link_directories(map_planner_node PRIVATE ${PCL_LIBRARY_DIRS})
target_link_libraries(map_planner_node ${PCL_LIBRARIES})
ament_target_dependencies(map_planner_node
  ament_index_cpp rclcpp geometry_msgs std_msgs sensor_msgs
  pcl_conversions visualization_msgs)

# --- A1.2: mission file model + executor --------------------------------------
add_library(mission_model STATIC src/mission_model.cpp)
target_include_directories(mission_model PUBLIC include)

add_executable(mission_executor_node src/mission_executor_node.cpp)
target_link_libraries(mission_executor_node mission_model)
ament_target_dependencies(mission_executor_node
  rclcpp geometry_msgs sensor_msgs mavros_msgs std_msgs)

install(TARGETS map_planner_node mission_executor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY missions/ DESTINATION share/${PROJECT_NAME}/missions)

ament_package()
