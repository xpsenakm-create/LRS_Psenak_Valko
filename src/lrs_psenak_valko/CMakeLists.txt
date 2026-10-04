cmake_minimum_required(VERSION 3.5)
project(lrs_psenak_valko)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

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
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS} src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_planner_node
  src/map_planner_node.cpp
  src/a_star_planner.cpp)
target_link_libraries(map_planner_node ${PCL_LIBRARIES})
ament_target_dependencies(map_planner_node ament_index_cpp rclcpp geometry_msgs std_msgs sensor_msgs
  pcl_conversions visualization_msgs)

add_library(mission_model src/mission_model.cpp)
target_include_directories(mission_model PUBLIC src)

add_executable(mission_executor_node src/mission_executor_node.cpp)
target_link_libraries(mission_executor_node mission_model)
ament_target_dependencies(mission_executor_node rclcpp geometry_msgs mavros_msgs std_msgs)

install(TARGETS map_planner_node mission_executor_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS mission_model DESTINATION lib)

install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY missions/ DESTINATION share/${PROJECT_NAME}/missions)

ament_package()