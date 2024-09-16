cmake_minimum_required(VERSION 3.0.2)
project(exp_comm_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs 
  sensor_msgs
  geometry_msgs
  swarm_exp_msgs
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DtgBagAnswer.msg
  DtgBagC.msg
  SwarmTrajC.msg
  IdPoseC.msg
  SwarmJobC.msg
  SwarmStateC.msg
  MapC.msg
  MapReqC.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  swarm_exp_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

