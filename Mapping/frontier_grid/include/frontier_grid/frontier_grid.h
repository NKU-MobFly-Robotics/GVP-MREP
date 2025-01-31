#ifndef FRONTIER_GRID_H_
#define FRONTIER_GRID_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <math.h>
#include <random>
// #include <octomap_world/octomap_manager.h>
#include <tr1/unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <frontier_grid/frontier_struct.h>
#include <block_map/color_manager.h>
#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <swarm_data/swarm_data.h>


using namespace std;
using namespace Eigen;
using namespace FrontierGridStruct;

class FrontierGrid{
public: 
    FrontierGrid(){};
    ~FrontierGrid(){};
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    
    void SetColorManager(ColorManager &CM){CM_ = &CM;}
    void SetLowresMap(lowres::LowResMap &LRM){LRM_ = &LRM;}
    void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;}
    void SetMap(BlockMap &BM){BM_ = &BM;}

    /**
     * @brief sample viewpoints of frontiers in exploring 
     * 
     */
    bool SampleVps();

    /**
     * @brief remove a viewpoint 
     * 
     * @param f_id frontier id  
     * @param v_id viewpoint id
     */
    inline void RemoveVp(const int &f_id, const int &v_id, bool broad_cast = false);

    /**
     * @brief pos to idx
     * 
     * @param pos 
     * @return -1: invalid; else: valid idx
     */
    inline int Pos2Idx(Eigen::Vector3d &pos);

    /**
     * @brief idx to pos
     * 
     * @param idx  
     * @param pos the center position corresponding to the f_grid
     * @return true valid idx
     * @return false invalid idx
     */
    inline bool Idx2Pos(const int &idx, Eigen::Vector3d &pos);

    /**
     * @brief pos to idx
     * 
     * @param pos 
     * @return -1: invalid; else: valid idx
     */
    inline int Posi2Idx(Eigen::Vector3i &pos);

    /**
     * @brief idx to pos3i
     * 
     * @param idx 
     * @param pos 
     * @return true valid idx
     * @return false invalid idx
     */
    inline bool Idx2Posi(const int &idx, Eigen::Vector3i &pos);

    /**
     * @brief Get the viewpoint position
     * 
     * @param f_idx frontier index
     * @param v_id  viewpoint id 
     * @param v_pos  viewpoint pos (xyz)
     * @return true 
     * @return false 
     */
    inline bool GetVpPos(const int &f_idx, const int &v_id, Eigen::Vector3d &v_pos);

    /**
     * @brief Get the viewpoint
     * 
     * @param f_idx frontier index
     * @param v_id  viewpoint id 
     * @param v_pose  viewpoint pose (xyz_yaw)
     * 
     * @return true alive viewpoint
     * @return false dead viewpoint
     */
    inline bool GetVp(const int &f_idx, const int &v_id, Eigen::Vector4d &v_pose);

    /**
     * @brief Get the Vp object
     * 
     * @param f_center frontier center
     * @param v_id      viewpoint id 
     * @param v_pose    viewpoint pose (xyz_yaw)
     * @return true 
     * @return false 
     */
    inline bool GetVp(const Eigen::Vector3d &f_center, const int &v_id, Eigen::Vector4d &v_pose);

    /**
     * @brief sample viewpoints for exploration
     * 
     * @param poses the poses of tagets to be covered by the viewpoints
     */
    bool SampleVps(list<Eigen::Vector3d> &poses);

    /**
     * @brief Set the corresponding frontier explored, dont broadcast
     * 
     * @param id id of the target frontier
     */
    inline void SetExplored(const int &id);

    /**
     * @brief Set the exploring frontier
     * 
     * @param dirs_state samplable dirs 
     * @param id         frontier id
     */
    inline void SetExploring(const vector<uint8_t> &dirs_states, const int &id);

    /**
     * @brief update frontier and viewpoints
     * 
     * @param pts newly registered points
     */
    void UpdateFrontier(const vector<Eigen::Vector3d> &pts);
    
    /**
     * @brief Get the grids inside a bounding box 
     * 
     * @param center 
     * @param box_scale 
     * @param f_list     
     */
    void GetWildGridsBBX(const Eigen::Vector3d &center, const Eigen::Vector3d &box_scale, list<pair<int, list<pair<int, Eigen::Vector3d>>>> &f_list);

    /**
     * @brief check if a viewpoint is blocked or has little gain. This func is time-consuming, only for exploring viewpoint.
     * 
     * @param f_id          frontier id
     * @param v_id          viewpoint id
     * @param allow_unknown allow the viewpoint in unknown space
     * @return true         not blocked, has enough gain
     * @return false        blocked or has little gain
     */
    bool StrongCheckViewpoint(const int &f_id, const int &v_id, const bool &allow_unknown);

    void DebugViewpoint();
    
    inline int Pos2Idx(const Eigen::Vector3d &pos);

    inline int IdCompress(const int &f_id, const int &v_id); 
    inline void AddShow(const int &f_id); 
    inline bool IdDecompress(const int &c_id, int &f_id, int &v_id); 
    inline bool ChangeOwner(const uint16_t &f_id, const uint8_t &n_owner, const double &n_dist, uint8_t &o_owner); 
    inline bool ClearOwner(const uint16_t &f_id); 
    inline double GetSensorRange(){return sensor_range_;}; 
    inline bool HaveAliveNeighbour(uint16_t f_id, Eigen::Vector3d dir);
    inline void YawNorm(double &yaw);
    inline double YawDiff(const double &yaw1, const double &yaw2);

    void ShowGainDebug();
    void DebugShowAll();
    Eigen::Vector3d Robot_pos_; //for lazy sample
    bool sample_flag_;          
    int min_vp_num_;
    SensorType sensor_type_;

    vector<CoarseFrontier> f_grid_;
private:
    inline bool InsideMap(Eigen::Vector3i &pos);
    inline bool InsideMap(Eigen::Vector3d &pos);
    inline bool InsideMap(const Eigen::Vector3d &pos);
    void ExpandFrontier(const int &idx, const bool &local_exp);
    void InitGainRays();
    bool SampleVps(list<int> &idxs);
    bool SampleVps(list<Eigen::Vector3i> &posis);
    double GetGain(const int &f_id, const int &vp_id);

    void LoadVpLines(visualization_msgs::Marker &mk, Eigen::Vector4d &vp);
    inline int Pos2DirIdx(const Eigen::Vector3d &pos, const Eigen::Vector3d &center);
    void SampleVpsCallback(const ros::TimerEvent &e);
    void LazySampleCallback(const ros::TimerEvent &e);
    void ShowVpsCallback(const ros::TimerEvent &e);
    void Debug(list<Eigen::Vector3d> &pts);
    void Debug(list<int> &v_ids);

    Eigen::Vector3d Robot_size_;
    Eigen::Vector3i node_num_;
    Eigen::Vector3d origin_, up_bd_;
    double resolution_, node_scale_, obs_thresh_;
    double vp_thresh_, resample_duration_, sensor_range_, sample_max_range_;

    int samp_h_dir_num_, samp_v_dir_num_, samp_dist_num_;
    double samp_h_dir_;
    int samp_dir_num_; //samp_h_dir_num_ * samp_dist_num_
    int samp_num_; //samp_h_dir_num_ * samp_v_dir_num_ * samp_dist_num_
    // int samp_free_thresh_;
    
    int scan_count_;
    ros::Publisher show_pub_, debug_pub_;
    ros::Timer sample_timer_, show_timer_, lazy_samp_timer_;
    //FOV down sample
    int FOV_h_num_, FOV_v_num_; 
    double cam_hor_, cam_ver_;
    double livox_ver_low_, livox_ver_up_;
    double ray_samp_dist1_, ray_samp_dist2_;

    vector<double> sample_dists_, sample_h_dirs_, sample_v_dirs_;
    vector<double> sample_vdir_sins_, sample_vdir_coses_;
    vector<double> sample_hdir_sins_, sample_hdir_coses_;
    vector<list<pair<list<Eigen::Vector3d>, list<pair<Eigen::Vector3d, double>>>>> gain_rays_;
    vector<list<pair<Eigen::Vector3d, double>>> gain_dirs_; //<<end pt of rays, raw gain>
    
    //to be shown
    list<int> explored_frontiers_show_;
    list<int> exploring_frontiers_show_;

    //for viewpoints sampling
    list<int> exploring_frontiers_;

    ros::NodeHandle nh_, nh_private_;
    ColorManager *CM_;
    lowres::LowResMap *LRM_;
    BlockMap *BM_;

    //swarm_data
    bool use_swarm_;
    SwarmDataManager *SDM_;

};

inline void FrontierGrid::RemoveVp(const int &f_id, const int &v_id, bool broad_cast){
    if(f_id < 0 || f_id >= f_grid_.size() || v_id < 0 || v_id >= samp_num_) return;
    if(f_grid_[f_id].local_vps_[v_id] == 2) return;
    // cout<<"remove:"<<f_id<<" "<<v_id<<endl;

    f_grid_[f_id].local_vps_[v_id] = 2;
    int alive_num = 0;
    for(auto &v : f_grid_[f_id].local_vps_){
        if(v != 2) {
            alive_num++;
            // kill_frontier = false;
            // break;
        }
    }
    bool kill_frontier = (alive_num < min_vp_num_);
    if(alive_num < min_vp_num_){
        if(f_grid_[f_id].f_state_ != 2 && use_swarm_ && !SDM_->is_ground_) {
            SDM_->SetDTGFn(f_id, f_grid_[f_id].local_vps_, 1, false);
            if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(f_id, false);
        }
        f_grid_[f_id].f_state_ = 2;
    }
    if(!f_grid_[f_id].flags_[2]){
        f_grid_[f_id].flags_.set(2);
        exploring_frontiers_show_.emplace_back(f_id);
    }
    if(!kill_frontier && !f_grid_[f_id].flags_[1]){
        f_grid_[f_id].flags_.set(1);
        exploring_frontiers_.emplace_back(f_id);
    }
    if(kill_frontier)
        ExpandFrontier(f_id, broad_cast);
    if(use_swarm_ && !kill_frontier && broad_cast && !SDM_->is_ground_) {
        SDM_->SetDTGFn(f_id, f_grid_[f_id].local_vps_, 0, true);
    }
}

inline bool FrontierGrid::InsideMap(Eigen::Vector3i &pos){
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  node_num_(0) || pos(1) >= node_num_(1) || pos(2) >= node_num_(2) )
        return false;
    return true;
}

inline bool FrontierGrid::InsideMap(Eigen::Vector3d &pos){
    if(pos(0) < origin_(0) || pos(1) < origin_(1) || pos(2) < origin_(2) ||
        pos(0) >  up_bd_(0) || pos(1) > up_bd_(1) || pos(2) > up_bd_(2) )
        return false;
    return true;
}

inline bool FrontierGrid::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < origin_(0) || pos(1) < origin_(1) || pos(2) < origin_(2) ||
        pos(0) >  up_bd_(0) || pos(1) > up_bd_(1) || pos(2) > up_bd_(2) )
        return false;
    return true;
}

inline int FrontierGrid::Pos2Idx(Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / node_scale_);
        posid.y() = floor(dpos.y() / node_scale_);
        posid.z() = floor(dpos.z() / node_scale_);
        return posid(2)*node_num_(0)*node_num_(1) + posid(1)*node_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}


inline int FrontierGrid::Pos2Idx(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / node_scale_);
        posid.y() = floor(dpos.y() / node_scale_);
        posid.z() = floor(dpos.z() / node_scale_);
        return posid(2)*node_num_(0)*node_num_(1) + posid(1)*node_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int FrontierGrid::Pos2DirIdx(const Eigen::Vector3d &pos, const Eigen::Vector3d &center){
    double dir = atan2(pos(1) - center(1), pos(0) - center(0));
    return int((dir + M_PI + 0.5 * samp_h_dir_) / (samp_h_dir_)) % samp_h_dir_num_;
}


inline bool FrontierGrid::Idx2Pos(const int &idx, Eigen::Vector3d &pos){
    if(idx >= 0 && idx < f_grid_.size()){
        int x = idx % node_num_(0);
        int y = ((idx - x)/node_num_(0)) % node_num_(1);
        int z = ((idx - x) - y*node_num_(0))/node_num_(1)/node_num_(0);
        pos(0) = (double(x)+0.5)*node_scale_;
        pos(1) = (double(y)+0.5)*node_scale_;
        pos(2) = (double(z)+0.5)*node_scale_;
        return true;
    }
    else return false;
}

inline int FrontierGrid::Posi2Idx(Eigen::Vector3i &pos){
    if(InsideMap(pos)){
        return pos(0) + pos(1) * node_num_(0) + pos(2) * node_num_(0) * node_num_(1); 
    }
    else{
        return -1; 
    }
}

inline bool FrontierGrid::Idx2Posi(const int &idx, Eigen::Vector3i &pos){
    if(idx >= 0 && idx < f_grid_.size()){
        int x = idx % node_num_(0);
        int y = ((idx - x)/node_num_(0)) % node_num_(1);
        int z = ((idx - x) - y*node_num_(0))/node_num_(1)/node_num_(0);
        pos(0) = x;
        pos(1) = y;
        pos(2) = z;
        return true;
    }
    else return false;
}

inline bool FrontierGrid::GetVpPos(const int &f_idx, const int &v_id, Eigen::Vector3d &v_pos){
    if(f_idx < 0 || f_idx >= f_grid_.size() || v_id < 0 || v_id >= samp_num_) return false;
    if(f_grid_[f_idx].f_state_ != 1) return false;
    // if(!f_grid_[f_idx].dirs_state_[v_id]) return false;
    int h_idx = (v_id + 0.1) / samp_dir_num_;

    // if((f_grid_[f_idx].dirs_state_[h_idx] == 1)){
        int d_idx = v_id % samp_dist_num_;
        int v_idx = (v_id - samp_dir_num_ * h_idx + 0.1) / samp_dist_num_;
        double length = sample_dists_[d_idx];
        double vdir_sin = sample_vdir_sins_[v_idx];
        double vdir_cos = sample_vdir_coses_[v_idx];
        double hdir_sin = sample_hdir_sins_[h_idx];
        double hdir_cos = sample_hdir_coses_[h_idx];
        v_pos(2) = length * vdir_sin + f_grid_[f_idx].center_(2);
        v_pos(1) = length * vdir_cos * hdir_sin + f_grid_[f_idx].center_(1);
        v_pos(0) = length * vdir_cos * hdir_cos + f_grid_[f_idx].center_(0);
        return true;
}


inline bool FrontierGrid::GetVp(const int &f_idx, const int &v_id, Eigen::Vector4d &v_pose){
    if(f_idx < 0 || f_idx >= f_grid_.size() || v_id < 0 || v_id >= samp_num_) return false;
    if(f_grid_[f_idx].f_state_ != 1) return false;
    // if(!f_grid_[f_idx].dirs_state_[v_id]) return false;
    int h_idx = (v_id + 0.1) / samp_dir_num_;

    // if((f_grid_[f_idx].dirs_state_[h_idx] == 1)){
        int d_idx = v_id % samp_dist_num_;
        int v_idx = (v_id - samp_dir_num_ * h_idx + 0.1) / samp_dist_num_;
        double length = sample_dists_[d_idx];
        double vdir_sin = sample_vdir_sins_[v_idx];
        double vdir_cos = sample_vdir_coses_[v_idx];
        double hdir_sin = sample_hdir_sins_[h_idx];
        double hdir_cos = sample_hdir_coses_[h_idx];
        v_pose(3) = M_PI + sample_h_dirs_[h_idx];
        v_pose(2) = length * vdir_sin + f_grid_[f_idx].center_(2);
        v_pose(1) = length * vdir_cos * hdir_sin + f_grid_[f_idx].center_(1);
        v_pose(0) = length * vdir_cos * hdir_cos + f_grid_[f_idx].center_(0);
        return true;
    // }
    // else return false;
}

inline bool FrontierGrid::GetVp(const Eigen::Vector3d &f_center, const int &v_id, Eigen::Vector4d &v_pose){
    if(v_id < 0 || v_id >= samp_num_) return false;
    int h_idx = (v_id + 0.1) / samp_dir_num_;
    int d_idx = v_id % samp_dist_num_;
    int v_idx = (v_id - samp_dir_num_ * h_idx + 0.1) / samp_dist_num_;

    double length = sample_dists_[d_idx];
    double vdir_sin = sample_vdir_sins_[v_idx];
    double vdir_cos = sample_vdir_coses_[v_idx];
    double hdir_sin = sample_hdir_sins_[h_idx];
    double hdir_cos = sample_hdir_coses_[h_idx];
    v_pose(3) = sample_h_dirs_[h_idx] + M_PI;

    v_pose(2) = length * vdir_sin + f_center(2);
    v_pose(1) = length * vdir_cos * hdir_sin + f_center(1);
    v_pose(0) = length * vdir_cos * hdir_cos + f_center(0);
    return true;
}

inline void FrontierGrid::SetExplored(const int &id){
    f_grid_[id].f_state_ = 2;
    if(!(f_grid_[id].flags_[2])){
        explored_frontiers_show_.emplace_back(id);
        f_grid_[id].flags_.set(2);
    }
}

inline void FrontierGrid::SetExploring(const vector<uint8_t> &dirs_states, const int &id){

}

inline int FrontierGrid::IdCompress(const int &f_id, const int &v_id){
    if(f_id < 0 || f_id >= f_grid_.size() || v_id < 0 || v_id >= samp_num_) return -1;
    return f_id * samp_num_ + v_id;
}

inline void FrontierGrid::AddShow(const int &f_id){
    if(!(f_grid_[f_id].flags_[2])){
        exploring_frontiers_show_.emplace_back(f_id);
        f_grid_[f_id].flags_.set(2);
    }
}

inline bool FrontierGrid::IdDecompress(const int &c_id, int &f_id, int &v_id){
    v_id = c_id % samp_num_;
    f_id = (c_id + 0.1) / samp_num_;
    if(f_id < 0 || f_id >= f_grid_.size()) return false;
    return true;
}

inline bool FrontierGrid::ChangeOwner(const uint16_t &f_id, const uint8_t &n_owner, const double &n_dist, uint8_t &o_owner){
    if(f_id < 0 || f_id >= f_grid_.size()) return false;
    o_owner = f_grid_[f_id].owner_;
    if(f_grid_[f_id].owner_dist_ > n_dist || f_grid_[f_id].owner_ == 0){
        f_grid_[f_id].owner_ = n_owner;
        f_grid_[f_id].owner_dist_ = n_dist;
        if(!f_grid_[f_id].flags_[2]){
            exploring_frontiers_show_.push_back(f_id);
            f_grid_[f_id].flags_.set(2);
        }
        return true;
    }
    return false;
}

inline bool FrontierGrid::ClearOwner(const uint16_t &f_id){
    if(f_id < 0 || f_id >= f_grid_.size()) return false;
    f_grid_[f_id].owner_ = 0;
    if(!f_grid_[f_id].flags_[2]){
        exploring_frontiers_show_.push_back(f_id);
        f_grid_[f_id].flags_.set(2);
    }
    return true;
}

inline bool FrontierGrid::HaveAliveNeighbour(uint16_t f_id, Eigen::Vector3d dir){
    Eigen::Vector3i pi, p_it;
    int f_n;
    if(!Idx2Posi(f_id, pi)) return false;
    for(int dim = 0; dim < 3; dim++){
        p_it = pi;
        if(dir(dim) < 0) p_it(dim) = pi(dim) - 1;
        else p_it(dim) = pi(dim) + 1;
        f_n = Posi2Idx(p_it);
        if(f_n != -1 && f_grid_[f_n].f_state_ != 2){
            return true;
        }
    }
    return false;
}

inline void FrontierGrid::YawNorm(double &yaw){
    double yawn;
    int c = yaw / M_PI / 2;
    yawn = yaw - c * M_PI * 2;
    
    if(yawn < -M_PI) yawn += M_PI * 2;
    if(yawn > M_PI) yawn -= M_PI * 2;
    yaw = yawn;
    return;
}

inline double FrontierGrid::YawDiff(const double &yaw1, const double &yaw2){
    double dy = yaw1 - yaw2;
    YawNorm(dy);
    return dy;
}
#endif