cmake_minimum_required(VERSION 3.0.2)
project(block_map)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME}_raycast src/raycast.cpp)
cs_add_library(${PROJECT_NAME}_grid src/block_map.cpp)
cs_add_library(${PROJECT_NAME}_color src/color_manager.cpp)

target_link_libraries(${PROJECT_NAME}_grid ${PROJECT_NAME}_raycast)
##########
# EXPORT #
##########
cs_install()
cs_export()

