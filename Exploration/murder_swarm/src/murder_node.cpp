#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include<murder_swarm/murderFSM.h>

#include <glog/logging.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "murder_node");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    MurderFSM M_FSM;
    M_FSM.init(nh, nh_private);
    ros::spin();
    return 0;
}