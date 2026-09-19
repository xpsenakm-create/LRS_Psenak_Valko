cmake_minimum_required(VERSION 3.5)
project(lrs_psenak_valko)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_planner_node src/map_planner_node.cpp)
target_link_libraries(map_planner_node ${PCL_LIBRARIES})

install(TARGETS map_planner_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
