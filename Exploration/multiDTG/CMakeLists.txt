cmake_minimum_required(VERSION 3.0.2)
project(multiDTG)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME} src/multiDTG.cpp src/multiDTG_path_search.cpp)
##########
# EXPORT #
##########
cs_install()
cs_export()