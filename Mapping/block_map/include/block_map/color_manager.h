#ifndef COLOR_MANAGER_H_
#define COLOR_MANAGER_H_

#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class ColorManager{
public:
    ColorManager(){};
    ~ColorManager(){};
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    inline std_msgs::ColorRGBA Id2Color(int idx, double a);

    vector<std_msgs::ColorRGBA> c_l_;
};


inline std_msgs::ColorRGBA ColorManager::Id2Color(int idx, double a){
    std_msgs::ColorRGBA color;
    idx = idx % int(c_l_.size());
    color = c_l_[idx];
    color.a = a;
    return color;
}

#endif