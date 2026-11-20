cmake_minimum_required(VERSION 3.0.2)
project(bw_statistic)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_executable(bw_statistic_node src/bw_statistic_node.cpp)

cs_install()
cs_export()
