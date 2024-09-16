cmake_minimum_required(VERSION 2.8.3)
project(swarm_exp_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

# requires zmq, zmqpp 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs 
  swarm_exp_msgs
  sensor_msgs
  geometry_msgs
  exp_comm_msgs
  multisim_gazebo
  gflags_catkin
  glog_catkin
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)



catkin_package(
 CATKIN_DEPENDS
)


# add_executable(bridge_ros_udp
#   src/bridge_ros_udp.cpp 
#   )
# target_link_libraries(bridge_ros_udp 
#   ${catkin_LIBRARIES}
#   )

add_executable(swarm_exp_bridge_tcp
  src/bridge_ros_tcp.cpp
)
target_link_libraries(swarm_exp_bridge_tcp
  ${catkin_LIBRARIES}
  zmq
  zmqpp
)    



add_executable(swarm_exp_bridge_tcp_sim
  src/bridge_ros_tcp_sim.cpp
)
target_link_libraries(swarm_exp_bridge_tcp_sim
  ${catkin_LIBRARIES}
  zmq
  zmqpp
)    