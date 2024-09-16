#include<murder_swarm/ground.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ground_node");
    ros::NodeHandle nh, nh_private("~");

    Ground GD;
    GD.init(nh, nh_private);

    ros::spin();
    return 0;
}