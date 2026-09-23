cmake_minimum_required(VERSION 3.10)
project(map_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # A* is ~10x slower without optimisation
endif()

find_package(PCL REQUIRED COMPONENTS common io filters)

add_executable(map_planner_node map_planner_node.cpp a_star_planner.cpp)
target_include_directories(map_planner_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(map_planner_node ${PCL_LIBRARIES})
