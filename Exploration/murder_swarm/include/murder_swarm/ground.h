#ifndef GROUND_H_
#define GROUND_H_

#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <tr1/unordered_map>

#include <multiDTG/dtg_structures.h>
#include <multiDTG/multiDTG.h>
#include <block_map/color_manager.h>
#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <swarm_data/swarm_data.h>
#include <gcopter/traj_opt.h>
#include <frontier_grid/frontier_grid.h>
#include <yaw_planner/yaw_planner.h>
#include <swarm_data/swarm_data.h>
#include <graph_partition/graph_partition.h>

#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/LocalTraj.h>

using namespace std;

class Ground{
public: 
    Ground(){};
    ~Ground(){};
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
private:
    void TrajVisCallback(const ros::TimerEvent &e);
    void PoseVisCallback(const ros::TimerEvent &e);
    void CreateVisModels();
    
    inline void LoadVisModels();

    ros::NodeHandle nh_, nh_private_;
    BlockMap BM_;
    DTG::MultiDTG MDTG_;
    AtoTraj TrajOpt_;
    FrontierGrid FG_;
    ColorManager CM_;
    lowres::LowResMap LRM_;
    SwarmDataManager SDM_;
    DTG::GraphVoronoiPartition GVP_;

    list<pair<uint16_t, uint32_t>> GVP_hn_;  // self global partition, not used
    list<pair<uint16_t, uint32_t>> local_hn_;  // self local partition, not used
    list<uint16_t> local_fn_;   // self exploring fn, not used

    ros::Timer traj_timer_, pose_timer_;
    ros::Publisher pose_vis_pub_, traj_vis_pub_;

    vector<Trajectory<5>> traj_;  
    vector<geometry_msgs::Pose> poses_;
    visualization_msgs::MarkerArray vis_models_;
};


inline void Ground::LoadVisModels(){
    double cur_t = ros::WallTime::now().toSec();
    for(int i = 0; i < SDM_.drone_num_; i++){
        if(abs(cur_t - SDM_.Pose_t_[i]) < 0.2) poses_[i] = SDM_.Poses_[i];
        vis_models_.markers[i*5 + 0].header.stamp = ros::Time::now();
        vis_models_.markers[i*5 + 1].header.stamp = ros::Time::now();
        vis_models_.markers[i*5 + 2].header.stamp = ros::Time::now();
        vis_models_.markers[i*5 + 3].header.stamp = ros::Time::now();
        vis_models_.markers[i*5 + 4].header.stamp = ros::Time::now();

        vis_models_.markers[i*5 + 0].pose.orientation = poses_[i].orientation;
        vis_models_.markers[i*5 + 1].pose.orientation = poses_[i].orientation;
        vis_models_.markers[i*5 + 2].pose.orientation = poses_[i].orientation;
        vis_models_.markers[i*5 + 3].pose.orientation = poses_[i].orientation;
        vis_models_.markers[i*5 + 4].pose = poses_[i];

        Eigen::Quaterniond rot;
        rot.x() = poses_[i].orientation.x;
        rot.y() = poses_[i].orientation.y;
        rot.z() = poses_[i].orientation.z;
        rot.w() = poses_[i].orientation.w;
        Eigen::Vector3d pos;
        pos(0) = poses_[i].position.x;
        pos(1) = poses_[i].position.y;
        pos(2) = poses_[i].position.z;

        Eigen::Vector3d p(0.225, 0.225, -0.02);
        vector<Eigen::Vector3d> pl;
        pl.emplace_back(p);
        p(0) = -p(0);
        pl.emplace_back(p);
        p(1) = -p(1);
        pl.emplace_back(p);
        p(0) = -p(0);
        pl.emplace_back(p);

        for(int j = 0; j < 4; j++){
            p = rot.toRotationMatrix() * pl[j] + pos;
            vis_models_.markers[i*5 + j].pose.position.x = p(0);
            vis_models_.markers[i*5 + j].pose.position.y = p(1);
            vis_models_.markers[i*5 + j].pose.position.z = p(2);
        }
    }
}
#endif