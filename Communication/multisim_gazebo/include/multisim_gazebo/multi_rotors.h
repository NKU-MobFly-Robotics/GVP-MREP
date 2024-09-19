#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>
#include <multisim_gazebo/states.h>

using namespace std;
class MultiRotors{
public:
    MultiRotors(){};
    ~MultiRotors(){};
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
private:
    void StatesCallback(const multisim_gazebo::statesConstPtr& msg);
    void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void StatesTimerCallback(const ros::TimerEvent& e);

    // bool center_gazebo_;
    int robot_num_;
    int local_num_;
    double set_freq_;
    //local base link names
    vector<string> local_blks_;
    //all model names
    vector<string> all_mns_;
    vector<bool> have_odom_;

    vector<int> local_ids_;
    vector<nav_msgs::Odometry> odoms_;

    ros::NodeHandle nh_, nh_private_;

    vector<ros::Subscriber> odoms_sub_;
    vector<ros::Publisher> odoms_show_pub_;
    ros::Publisher states_pub_;
    ros::Subscriber states_sub_;
    ros::Timer states_timer_;



};
