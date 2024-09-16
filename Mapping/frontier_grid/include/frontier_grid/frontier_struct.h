#ifndef FRONTIER_STRUCT_H_
#define FRONTIER_STRUCT_H_
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <deque>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <list>
#include <bitset>
#include <memory>
#include <math.h>
#include <std_msgs/ColorRGBA.h>
using namespace std;
using namespace Eigen;

namespace FrontierGridStruct{
enum SensorType{LIVOX, CAMERA};

struct CoarseFrontier{
    uint8_t f_state_;                    //0: unexplored; 1:exploring; 2: explored; 
    uint8_t owner_;
    float owner_dist_;

    // vector<uint8_t> dirs_state_;         //0: unexplored; 1:exploring; 2: explored; 
    vector<uint16_t> dirs_free_num_;      
    Eigen::Vector3d up_, down_, center_;

    vector<uint8_t> local_vps_;         //0: unsampled; 1:alive; 2: dead; 
    // vector<uint8_t> public_vps_;        //0: unsampled; 1:alive; 2: dead; 
    double last_sample_;
    double last_strong_check_;
    int unknown_num_, thresh_num_;
    bitset<4> flags_;                     //(local new)(show)(sample)(temp)
};
}

#endif