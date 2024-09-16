cmake_minimum_required(VERSION 3.0.2)
project(yaw_planner)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# cs_add_library(${PROJECT_NAME}_grid src/frontier_grid.cpp src/Viewpoint_manager.cpp)
cs_add_library(${PROJECT_NAME} src/yaw_planner.cpp)

##########
# EXPORT #
##########
cs_install()
cs_export()

