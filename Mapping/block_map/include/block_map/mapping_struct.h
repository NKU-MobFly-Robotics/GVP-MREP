#ifndef MAPPING_STRUCT_H_
#define MAPPING_STRUCT_H_
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

namespace BlockMapStruct{

enum GBSTATE{
    UNKNOWN,
    MIXED,
    OCCUPIED,
    FREE
};

enum VoxelState{
    unknown, 
    free, 
    occupied,
    out
};

struct SwarmBlock{
    uint16_t id_;
    Eigen::Vector3d up_, down_;
    vector<double> exploration_rate_;
    vector<double> last_pub_rate_;
    vector<bool> to_pub_;
    uint8_t sub_num_;
};

struct Grid_Block{
    Grid_Block() {state_ = UNKNOWN;};
    ~Grid_Block() {};
    void Awake(float occ, float free){       //if state == UNKNOWN/OCCUPIED/FREE, init odds_log_ of this block 
        if(state_ == UNKNOWN){
            odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), free - 999.0);
            flags_.resize(odds_log_.size(), 0);
        }
        else if(state_ == OCCUPIED){
            odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), occ);
            flags_.resize(odds_log_.size(), 0);
        }
        else if(state_ == FREE){
            odds_log_.resize(block_size_.x() * block_size_.y() * block_size_.z(), free);
            flags_.resize(odds_log_.size(), 0);
        }
        state_ = MIXED;
    }
    Vector3i origin_;
    Vector3i block_size_;
    unsigned state_;
    vector<float> odds_log_;
    vector<uint8_t> flags_;  //0000 0_(need to be show)_(is ray end occupied flag)_(casted)
    bool show_;
    int free_num_, occ_num_, unk_num_;
    int free_max_num_, occ_max_num_;
};

struct FFD_Grid{
    double far_depth_;
    double close_depth_;
    double max_depth_;

    double dist2depth_;
    bool is_frontier_;
    bool new_iter_;
};
}

#endif