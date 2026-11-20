cmake_minimum_required(VERSION 2.8.3)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")

# 修复(永久):系统 PCL 库导出的 Eigen::internal::aligned_malloc(朴素 malloc 协议)经
# 符号插入与自编代码的 handmade_aligned_free(记账协议,读 ptr-8)错配——ASan 实证,
# 是规划期堆破坏总源头。EIGEN_MALLOC_ALREADY_ALIGNED=1 让自编代码的分配/释放也统一
# 走朴素 malloc/free,与 PCL 副本协议一致,插入到谁头上都无害。
# (注:EIGEN_DONT_ALIGN 在 Eigen 3.3 对动态存储不生效,勿用)
add_definitions(-DEIGEN_MALLOC_ALREADY_ALIGNED=1)



set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
# find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_opt
#  DEPENDS system_lib
)

add_library(traj_opt src/traj_opt.cpp)

target_link_libraries(traj_opt
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
