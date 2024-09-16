cmake_minimum_required(VERSION 2.8.3)
project(ato_test)

find_package(catkin_simple REQUIRED)
add_definitions(-std=c++14)

catkin_simple()

cs_add_executable(fov_ray_test src/fov_ray_test.cpp)

target_link_libraries(fov_ray_test
    ${catkin_LIBRARIES})


cs_add_executable(swarm_data_test src/swarm_data_test.cpp)

target_link_libraries(swarm_data_test
    ${catkin_LIBRARIES})

cs_add_executable(traj_swarm src/traj_swarm.cpp)

target_link_libraries(traj_swarm
        ${catkin_LIBRARIES})

cs_add_executable(cmd_test src/cmd_test.cpp)

target_link_libraries(cmd_test
        ${catkin_LIBRARIES})

cs_add_executable(cmd_repub src/cmd_repub.cpp)

target_link_libraries(cmd_repub
        ${catkin_LIBRARIES})

cs_add_executable(GMM_test src/GMM_test.cpp)

target_link_libraries(cmd_repub
        ${catkin_LIBRARIES})
cs_install()
cs_export()