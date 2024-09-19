cmake_minimum_required(VERSION 3.0.2)
project(multisim_gazebo)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(multi_rotors src/multi_rotors.cpp)

cs_add_executable(multi_rotors_node src/multi_rotors_node.cpp)

target_link_libraries(multi_rotors_node
    ${catkin_LIBRARIES}
    multi_rotors)

##########
# EXPORT #
##########
cs_install()
cs_export()