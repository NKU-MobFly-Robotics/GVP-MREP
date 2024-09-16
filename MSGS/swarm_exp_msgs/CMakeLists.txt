cmake_minimum_required(VERSION 3.0.2)
project(swarm_exp_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs 
  sensor_msgs
  geometry_msgs
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SwarmTraj.msg
  LocalTraj.msg
  DtgFFEdge.msg
  DtgHFEdge.msg
  DtgHHEdge.msg
  DtgHNode.msg
  DtgFNode.msg
  DtgBag.msg
  IdOdom.msg
  IdPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

