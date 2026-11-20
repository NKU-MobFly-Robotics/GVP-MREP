cmake_minimum_required(VERSION 3.0.2)
project(swarm_data)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# 修复(永久):系统 PCL 库导出的 Eigen::internal::aligned_malloc(朴素 malloc 协议)经
# 符号插入与自编代码的 handmade_aligned_free(记账协议,读 ptr-8)错配——ASan 实证,
# 是规划期堆破坏总源头。EIGEN_MALLOC_ALREADY_ALIGNED=1 让自编代码的分配/释放也统一
# 走朴素 malloc/free,与 PCL 副本协议一致,插入到谁头上都无害。
# (注:EIGEN_DONT_ALIGN 在 Eigen 3.3 对动态存储不生效,勿用)
add_definitions(-DEIGEN_MALLOC_ALREADY_ALIGNED=1)



find_package(Eigen3 REQUIRED)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME} src/swarm_data.cpp)

# target_link_libraries(${PROJECT_NAME}
#     ${catkin_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export()
