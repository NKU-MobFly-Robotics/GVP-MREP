cmake_minimum_required(VERSION 3.0.2)
project(data_statistics)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
) 

cs_add_library(computation_statistician src/computation_statistician.cpp)

# cs_add_library(volum_listener src/volum_listener.cpp)
# cs_add_executable(volum_logger src/volum_node.cpp)
# target_link_libraries(volum_logger volum_listener)

##########
# EXPORT #
##########
cs_install()
cs_export()
