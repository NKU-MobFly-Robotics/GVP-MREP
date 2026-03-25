cmake_minimum_required(VERSION 3.0.2)
project(murder_swarm)

add_definitions(-std=c++14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_LIBRARY_PATH /usr/lib/x86_64-linux-gnu)
find_library(LIBFFI_LIBRARIES NAMES ffi HINTS ${CMAKE_LIBRARY_PATH})
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(${PROJECT_NAME} src/murder.cpp)

cs_add_library(${PROJECT_NAME}_fsm src/murderFSM.cpp)
target_link_libraries(${PROJECT_NAME}_fsm
    ${catkin_LIBRARIES} ${PROJECT_NAME} ${LIBFFI_LIBRARIES})

cs_add_library(ground src/ground.cpp)


cs_add_executable(murder_swarm_node src/murder_node.cpp)

target_link_libraries(murder_swarm_node
    ${catkin_LIBRARIES} ${PROJECT_NAME}_fsm ${LIBFFI_LIBRARIES})

cs_add_executable(swarm_trigger src/swarm_trigger.cpp)

target_link_libraries(swarm_trigger
    ${catkin_LIBRARIES} ${LIBFFI_LIBRARIES})



cs_add_executable(ground_node src/ground_node.cpp)

target_link_libraries(ground_node
        ${catkin_LIBRARIES} ground ${LIBFFI_LIBRARIES})
##########
# EXPORT #
##########
cs_install()
cs_export()