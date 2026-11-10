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
find_package(PCL REQUIRED COMPONENTS common io filters)

find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(outdoor_mission_node src/outdoor_mission_node.cpp)
ament_target_dependencies(outdoor_mission_node
  rclcpp mavros_msgs sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs)
install(TARGETS outdoor_mission_node DESTINATION lib/${PROJECT_NAME})

include_directories(${PCL_INCLUDE_DIRS} src)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_planner_node
  src/map_planner_node.cpp
  src/a_star_planner.cpp)
target_link_libraries(map_planner_node ${PCL_LIBRARIES})
ament_target_dependencies(map_planner_node ament_index_cpp rclcpp geometry_msgs std_msgs sensor_msgs
  pcl_conversions visualization_msgs)

install(TARGETS map_planner_node DESTINATION lib/${PROJECT_NAME})

# A2: eight-state EKF (ekf8.h, Eigen only), its ROS node, the mission flown on it,
# and two ROS-free tools: the unit test and the offline replay (tools/a2_offline_sim.py).
add_executable(localization_estimator_node src/localization_estimator_node.cpp)
target_link_libraries(localization_estimator_node Eigen3::Eigen)
ament_target_dependencies(localization_estimator_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

add_executable(localization_mission_node src/localization_mission_node.cpp)
ament_target_dependencies(localization_mission_node
  rclcpp mavros_msgs geometry_msgs nav_msgs std_msgs std_srvs)

add_executable(ekf8_test src/ekf8_test.cpp)
target_link_libraries(ekf8_test Eigen3::Eigen)
add_executable(ekf8_replay src/ekf8_replay.cpp)
target_link_libraries(ekf8_replay Eigen3::Eigen)

install(TARGETS localization_estimator_node localization_mission_node ekf8_test ekf8_replay
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()