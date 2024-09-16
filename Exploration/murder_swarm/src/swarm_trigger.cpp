#include <ros/ros.h>
#include <stdio.h>
#include <string>

#include <std_msgs/Empty.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "trigger");
    ros::NodeHandle nh, nh_private("~");

    ros::Publisher trigger_pub;

    trigger_pub = nh.advertise<std_msgs::Empty>("/takeoff", 10);

    ros::Duration(0.5).sleep();
    std_msgs::Empty msg;
    trigger_pub.publish(msg);
    ROS_WARN("go go go!!!");
    ros::Duration(0.5).sleep();
    return 0;
}