#ifndef BLOCK_MAP_H_
#define BLOCK_MAP_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <math.h>
#include <tr1/unordered_map>
// #include <octomap_world/octomap_manager.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <cv_bridge/cv_bridge.h>

#include <block_map/raycast.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <block_map/mapping_struct.h>
#include <exp_comm_msgs/MapC.h>
#include <exp_comm_msgs/MapReqC.h>

#include <swarm_data/swarm_data.h>

using namespace std;
using namespace BlockMapStruct;



class BlockMap{
public:
    BlockMap() {};
    ~BlockMap() {if(use_swarm_) delete swarm_filter_dict_;};

    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

    /**
     * @brief initialize swarm block
     * 
     * @param id_l  block id list
     * @param bound <up, low> 
     */
    void InitSwarmBlock(vector<uint16_t> &id_l, vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &bound);

    void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;};
    /**
     * @brief check if voxes inside the bbx centered at pos contains any occpied vox 
     * 
     * @param pos center
     * @param bbx boundingbox
     * @return true 
     * @return false 
     */
    bool PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx);

    /**
     * @brief check if voxes inside the bbx are all free
     * 
     * @param pos center
     * @param bbx boudingbox
     * @return true 
     * @return false 
     */
    bool PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx);
    
    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const int &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3i &id);

    /**
     * @brief Get the Vox State: unknown/occupied/free
     * 
     * @param id 
     * @return VoxelState 
     */
    inline VoxelState GetVoxState(const Eigen::Vector3d &id);

    /**
     * @brief update map using pcl
     * 
     * @param pcl 
     */
    void InsertPcl(const sensor_msgs::PointCloud2ConstPtr &pcl);


    /**
     * @brief update map using depth img
     * 
     * @param depth 
     */
    void InsertImg(const sensor_msgs::ImageConstPtr &depth);

    /**
     * @brief set current robot pose
     * 
     * @param odom 
     */
    void OdomCallback(const nav_msgs::OdometryConstPtr &odom);

    void SendSwarmBlockMap(const int &f_id, const bool &send_now);

    /**
     * @brief Get the vox poses on the line
     * 
     * @param start line start pos
     * @param end   line end pos
     * @param line 
     */
    inline void GetCastLine(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &line);
    inline int PostoId(const Eigen::Vector3d &pos);//dont check in side map, carefully use



    //for low resolution map
    vector<Eigen::Vector3d> cur_pcl_;
    vector<Eigen::Vector3d> newly_register_idx_;
    double resolution_;

private:
    //callback func
    void InsertPCLCallback(const sensor_msgs::PointCloud2ConstPtr &pcl);
    void InsertDepthCallback(const sensor_msgs::ImageConstPtr &img);
    void CamParamCallback(const sensor_msgs::CameraInfoConstPtr &param);

    //Timer
    void ShowMapCallback(const ros::TimerEvent &e);
    void SwarmMapCallback(const ros::TimerEvent &e);
    void StatisticV(const ros::TimerEvent &e);

    void ProjectToImg(const sensor_msgs::PointCloud2ConstPtr &pcl, vector<double> &depth_img);
    void AwakeBlocks(const Eigen::Vector3d &center, const double &range);
    void InsertSwarmPts(exp_comm_msgs::MapC &msg);

    inline void Vox2Msg(exp_comm_msgs::MapC &msg, const uint16_t &f_id, const uint8_t &b_id);
    inline void Flags2Vox(vector<uint8_t> &flags, Eigen::Vector3d &up, Eigen::Vector3d &down, uint8_t &block_state, vector<Eigen::Vector3d> &pts, vector<uint8_t> &states);
    inline void InseartVox(vector<Eigen::Vector3d> &pts, vector<uint8_t> &states);// newly_register_idx_ not update
    inline void UpdateSBS(const int &SBS_id, const int &sub_id);
    inline bool GetSBSBound(const uint16_t &SBS_id, const uint8_t &sub_id, Eigen::Vector3d &up, Eigen::Vector3d &down);

    // inline 
    inline bool GetVox(int &block_id, int &vox_id, const Vector3i &pos);   //return true: inside map; false: outside map
    inline bool GetVox(int &block_id, int &vox_id, const Vector3d &pos);

    inline float GetVoxOdds(const Eigen::Vector3d &pos);//check if pos is in the block, dont check if pos is in the block
    inline float GetVoxOdds(const int &id);//don't check, carefully use
    inline float GetVoxOdds(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &FG);//don't check

    inline bool GetBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid);//check
    inline int GetBlockId(const Eigen::Vector3d &pos);//check
    inline int GetBlockId(const Eigen::Vector3i &pos);//check, pos: block id/carefully use 
    
    inline int GetVoxId(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB);//don't check, pos of world
    inline int GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<Grid_Block> &GB);//don't check, pos of world
    inline Eigen::Vector3d Id2LocalPos(const shared_ptr<Grid_Block> &GB, const int &id);


    inline bool InsideMap(const Eigen::Vector3i &pos);//
    inline bool InsideMap(const Eigen::Vector3d &pos);//

    inline Eigen::Vector3d IdtoPos(int id);//
    inline Eigen::Vector3i PostoId3(const Eigen::Vector3d &pos);

    inline void GetRayEndInsideMap(const Eigen::Vector3d &start, Eigen::Vector3d &end, bool &occ);

    inline void SpointToUV(const double &x, const double &y, Eigen::Vector2i &uv); //standard depth to camera vector index
    inline void UVToPoint(const int &u, const int &v, const double &depth, Eigen::Vector3d &point); //depthcamera point to 3d point 

    inline void UpdateSwarmData();
    inline void LoadSwarmFilter();


    inline std_msgs::ColorRGBA Getcolor(const double z);

    void Debug(list<Eigen::Vector3d> &pts, int ddd = 0);


    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber odom_sub_, sensor_sub_, camparam_sub_;
    ros::Publisher vox_pub_, debug_pub_;
    ros::Timer show_timer_, debug_timer_;
    vector<std_msgs::ColorRGBA> color_list_;
    double colorhsize_;
    
    Eigen::Vector3d origin_, blockscale_, edgeblock_scale_, map_upbd_, map_lowbd_;
    Eigen::Vector3i block_size_, voxel_num_, block_num_;       //block_size: size of voxels in a block, block_num_:total number of blocks
    Eigen::Vector3i edgeblock_size_;

    nav_msgs::Odometry robot_odom_;
    Eigen::Matrix4d cam2body_, cam2world_;
    //map updating params
    double pro_hit_;             //log(P(hit|occupied)/P(hit|free))
    double pro_miss_;           //log(P(miss|occupied)/P(miss|free))
    double thr_max_, thr_min_, thr_occ_;

    //sensor params
    bool bline_;
    double max_range_;
    double fx_, fy_, cx_, cy_;    
    int u_max_, v_max_, u_down_max_, v_down_max_, downsample_size_, depth_step_;
    vector<shared_ptr<Grid_Block>> GBS_;
    vector<double> void_img_;
    vector<FFD_Grid> downsampled_img_;  //FFD
    //detect frontier flag
    bool depth_;
    bool have_odom_, have_cam_param_, show_block_;
    double last_update_, update_interval_, show_freq_, last_odom_;
    //blocks to be shown
    vector<int> changed_blocks_;
    //awake blocks 
    list<int> awake_blocks_; 

    //statistic
    bool stat_;
    ros::Publisher statistic_pub_;
    ros::Timer statistic_timer_;
    Eigen::Vector3d stat_upbd_, stat_lowbd_;
    int stat_n_;
    double stat_v_;

    //swarm
    ros::Timer swarm_timer_;
    bool use_swarm_, req_flag_, finish_flag_;
    SwarmDataManager *SDM_;
    
    vector<SwarmBlock> SBS_;

    double min_finish_t_, start_t_;
    list<pair<pair<uint16_t, uint8_t>, double>> swarm_pub_block_; 
    vector<pair<double, Eigen::Matrix4d>> swarm_pose_;
    vector<Eigen::Vector3d> robots_scale_;
    uint8_t self_id_;
    double swarm_send_delay_, swarm_tol_, swarm_pub_thresh_, bline_occ_range_;
    tr1::unordered_map<int, int> *swarm_filter_dict_;
    bool vis_mode_;
    // ofstream debug_l_;

};

inline void BlockMap::Flags2Vox(vector<uint8_t> &flags, Eigen::Vector3d &up, Eigen::Vector3d &down,
                                    uint8_t &block_state, vector<Eigen::Vector3d> &pts, vector<uint8_t> &states){
    Eigen::Vector3i v_n;
    int vox_num = 1;
    int flag_num = flags.size() * 4;
    pts.clear();
    states.clear();
    for(int dim = 0; dim < 3; dim++) {
        vox_num *= int(ceil((up(dim) - down(dim)) / resolution_));
        v_n(dim) = int(ceil((up(dim) - down(dim)) / resolution_));
    }
    if(flag_num < vox_num && block_state == 3){
        ROS_ERROR("error Flags2Vox %d < %d, state: %d", flag_num, vox_num, block_state);
        return;
    }
    else if(block_state == 0){
        ROS_ERROR("error Flags2Vox state: 0");
        return;
    }

    Eigen::Vector3d cur_pos, origin_pos;
    int x_id, y_id, z_id;
    origin_pos = down + Eigen::Vector3d::Ones() * resolution_ / 2;
    // cout<<"pure origin_pos:"<<origin_pos.transpose()<<" down:"<<down.transpose()<<endl;

    if(block_state == 0) return; // all unknown
    else if(block_state == 1 || block_state == 2){
        states.resize(vox_num, block_state);

        for(int i = 0; i < vox_num; i++){
            x_id = i % v_n(0);
            y_id = ((i - x_id)/v_n(0)) % v_n(1);
            z_id = ((i - x_id) - y_id*v_n(0))/v_n(1)/v_n(0);
            cur_pos(0) = x_id * resolution_ + origin_pos(0);
            cur_pos(1) = y_id * resolution_ + origin_pos(1);
            cur_pos(2) = z_id * resolution_ + origin_pos(2);
            pts.push_back(cur_pos);
        }
        return; 
    }

    Eigen::Vector3d vox_pos;
    int id;
    int flag_id = 0;
    uint8_t flag1, flag2;
    pts.resize(vox_num);
    states.resize(vox_num, 0);
    for(int i = 0; i < flags.size(); i++){
        for(int j = 0; j < 4; j++){
            id = i * 4 + j;
            x_id = id % v_n(0);
            y_id = ((id - x_id)/v_n(0)) % v_n(1);
            z_id = ((id - x_id) - y_id*v_n(0))/v_n(1)/v_n(0);
            cur_pos(0) = x_id * resolution_ + origin_pos(0);
            cur_pos(1) = y_id * resolution_ + origin_pos(1);
            cur_pos(2) = z_id * resolution_ + origin_pos(2);
            if(id >= vox_num) break;
            flag1 = 1<<(2*j);
            flag2 = 2<<(2*j);
            pts[id] = cur_pos;
            if(flag1 & flags[i]){ // occ
                states[id] = 1;
            }
            else if(flag2 & flags[i]){ // free
                states[id] = 2;
            }
        }
    }
}

inline void BlockMap::Vox2Msg(exp_comm_msgs::MapC &msg, const uint16_t &f_id, const uint8_t &b_id){
    Eigen::Vector3d up, down, it;
    if(!GetSBSBound(f_id, b_id, up, down)) return;
    double unknown_num = 0;
    double sub_num = 1.0;
    for(int dim = 0; dim < 3; dim++) sub_num *= ceil((up(dim) - down(dim)) / resolution_);

    msg.block_state = 0;
    msg.f_id = f_id;
    msg.block_id = b_id;
    msg.flags.clear();
    bool unknown_block = true;
    bool f = false;
    bool o = false;
    int idx = 0;
    int p;
    uint8_t flag1, flag2;
    VoxelState cur_vs, last_vs;
    for(it(2) = down(2); it(2) < up(2); it(2) += resolution_)
        for(it(1) = down(1); it(1) < up(1); it(1) += resolution_)
            for(it(0) = down(0); it(0) < up(0); it(0) += resolution_){
        cur_vs = GetVoxState(it);
        p = idx % 4;
        flag1 = 1<<(2*p);
        flag2 = 2<<(2*p);
        if(idx == 0) last_vs = cur_vs;
        if(p == 0) msg.flags.push_back(0);

        if(cur_vs == VoxelState::out){
            ROS_ERROR("Vox2Msg how out?");
            ros::shutdown();
            return;
        }
        else if(cur_vs == VoxelState::unknown){
            unknown_num += 1.0;
        }
        else if(cur_vs == VoxelState::free){
            unknown_block = false;
            f = true;
            msg.flags.back() |= flag2;
        }
        else if(cur_vs == VoxelState::occupied){
            unknown_block = false;
            o = true;
            msg.flags.back() |= flag1;
        }

        if(last_vs != cur_vs){
            msg.block_state = 3;
        }
        last_vs = cur_vs;
        idx++;
    }

    if(!msg.flags.empty() && msg.block_state == 0 && !unknown_block){
        if(msg.flags.front() & 1) {
            msg.block_state = 1;
        }
        else if(msg.flags.front() & 2) msg.block_state = 2;
        else msg.block_state = 0;
        msg.flags.clear();
    }
    SBS_[f_id].exploration_rate_[b_id] = 1.0 - unknown_num / sub_num;
}

inline void BlockMap::InseartVox(vector<Eigen::Vector3d> &pts, vector<uint8_t> &states){
    int block_id, vox_id;
    Eigen::Vector3d p_it;
    for(int i = 0; i < pts.size(); i++){
        if(states[i] == 0) continue;
        if(!GetVox(block_id, vox_id, pts[i])){
            cout<<"fail:"<<pts[i].transpose()<<endl;
            ros::shutdown();
            return;
            continue;
        }
        float odds_origin = GBS_[block_id]->odds_log_[vox_id];

        if(GBS_[block_id]->odds_log_[vox_id] < thr_min_ - 1.0){
            p_it = Id2LocalPos(GBS_[block_id], vox_id);
            if(stat_){
                if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                stat_n_++;
            }
            odds_origin = 0.0;
        }
        if(states[i] == 1){
            GBS_[block_id]->odds_log_[vox_id] = min(odds_origin + pro_hit_, thr_max_);
        }
        else if(states[i] == 2){
            GBS_[block_id]->odds_log_[vox_id] = max(odds_origin + pro_miss_, thr_min_);
        }
        if(!GBS_[block_id]->show_ && show_block_){
            changed_blocks_.push_back(block_id);
            GBS_[block_id]->show_ = true;
        }
    }
}

inline void BlockMap::UpdateSBS(const int &SBS_id, const int &sub_id){
    Eigen::Vector3d up, down, it;
    if(!GetSBSBound(SBS_id, sub_id, up, down)) return;
    double unknown_num = 0;
    double sub_num = 1.0;
    for(int dim = 0; dim < 3; dim++) sub_num *= ceil((up(dim) - down(dim)) / resolution_);

    for(it(0) = down(0); it(0) < up(0); it(0) += resolution_)
        for(it(1) = down(1); it(1) < up(1); it(1) += resolution_)
            for(it(2) = down(2); it(2) < up(2); it(2) += resolution_){
        VoxelState vs = GetVoxState(it);
        if(vs == out){
            ROS_ERROR("UpdateSBS how out?");
            ros::shutdown();
            return;
        }
        else if(vs == unknown){
            unknown_num += 1.0;
        }
    }

    SBS_[SBS_id].exploration_rate_[sub_id] = 1.0 - unknown_num / sub_num;
}

inline bool BlockMap::GetSBSBound(const uint16_t &SBS_id, const uint8_t &sub_id, Eigen::Vector3d &up, Eigen::Vector3d &down){
    if(SBS_id < 0 || SBS_id >= SBS_.size()) return false;
    if(sub_id < 0 || sub_id >= 8) return false;
    Eigen::Vector3d origin = SBS_[SBS_id].down_;
    for(int dim = 0; dim < 3; dim++){
        uint8_t flag = 1<<dim;
        int half_num = ceil((SBS_[SBS_id].up_(dim) - SBS_[SBS_id].down_(dim)) / 2 / resolution_);
        if(sub_id & flag){
            up(dim) = SBS_[SBS_id].up_(dim);
            down(dim) = SBS_[SBS_id].down_(dim) + half_num * resolution_; 
        }
        else{
            up(dim) = SBS_[SBS_id].down_(dim) + half_num * resolution_ - 2e-3;
            down(dim) = SBS_[SBS_id].down_(dim); 
        }
    }

    return true;
}

inline bool BlockMap::GetVox(int &block_id, int &vox_id, const Vector3i &pos){
    Vector3d pos3d = pos.cast<double>() * resolution_;
    return GetVox(block_id, vox_id, pos3d);
}

inline bool BlockMap::GetVox(int &block_id, int &vox_id, const Vector3d &pos){
    block_id = GetBlockId(pos);
    if(block_id != -1){
        shared_ptr<Grid_Block> GB_ptr = GBS_[block_id];
        if(GB_ptr->state_ == MIXED){
            vox_id = GetVoxId(pos, GB_ptr);
            // cout<<"blk size:"<<GB_ptr->block_size_.transpose()<<"pos:::"<<pos.transpose()<<"  B origin:"<<GB_ptr->origin_.transpose()<<" state:"<<GB_ptr->state_<<" size1:"<<GB_ptr->odds_log_.size()<<" size2:"<<GB_ptr->flags_.size()<<endl;
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }

}

inline float BlockMap::GetVoxOdds(const Eigen::Vector3d &pos){//check if pos is in the block, dont check if pos is in the block
    int blockid = GetBlockId(pos);
    if(blockid != -1){
        shared_ptr<Grid_Block> GB_ptr = GBS_[blockid];
        if(GB_ptr->state_ == MIXED){
            return GetVoxOdds(pos, GB_ptr);
        }
        else if(GB_ptr->state_ == GBSTATE::FREE){
            return thr_min_;
        }
        else if(GB_ptr->state_ == GBSTATE::OCCUPIED){
            return thr_max_;
        }
        else{
            return 0.0;
        }
    }
    else{
        return 0.0;
    }
}

inline float BlockMap::GetVoxOdds(const int &id){//don't check, carefully use
    Eigen::Vector3d pos = IdtoPos(id);
    
    int blockid = GetBlockId(pos);
    if(blockid == -1) {
        return 0.0;
    }
    else if(GBS_[blockid]->state_ == GBSTATE::MIXED){
        return GBS_[blockid]->odds_log_[GetVoxId(pos, GBS_[blockid])];
    } 
    else if(GBS_[blockid]->state_ == GBSTATE::FREE){
        return thr_min_;
    }
    else if(GBS_[blockid]->state_ == GBSTATE::OCCUPIED){
        return thr_max_;
    }
    else {
        return 0.0;
    }
}

inline float BlockMap::GetVoxOdds(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &FG){//don't check
    return FG->odds_log_[GetVoxId(pos, FG)];
}

inline bool BlockMap::GetBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid){//check
    // Eigen::Vector3i pos3;

    // cout<<"pos:"<<pos.transpose()<<endl;
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        blkid.x() = floor(dpos.x() / blockscale_.x());
        blkid.y() = floor(dpos.y() / blockscale_.y());
        blkid.z() = floor(dpos.z() / blockscale_.z());
        return true;
    }
    else{
        return false;
    }
}

inline int BlockMap::GetBlockId(const Eigen::Vector3d &pos){//check
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / blockscale_.x());
        posid.y() = floor(dpos.y() / blockscale_.y());
        posid.z() = floor(dpos.z() / blockscale_.z());
        return posid(2)*block_num_(0)*block_num_(1) + posid(1)*block_num_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int BlockMap::GetBlockId(const Eigen::Vector3i &pos){//check, pos: block id/carefully use 
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  block_num_(0) || pos(1) >= block_num_(1) || pos(2) >= block_num_(2)){
            return -1;
        }
    else{
        return pos(2)*block_num_(0)*block_num_(1) + pos(1)*block_num_(0) + pos(0);
    }
}

inline int BlockMap::GetVoxId(const Eigen::Vector3d &pos, const shared_ptr<Grid_Block> &GB){//don't check, pos of world
    Eigen::Vector3d dpos = pos - origin_ - GB->origin_.cast<double>()*resolution_;
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / resolution_);
    posid.y() = floor(dpos(1) / resolution_);
    posid.z() = floor(dpos(2) / resolution_);

    return posid(2) * GB->block_size_.x() * GB->block_size_.y() + posid(1) * GB->block_size_.x() + posid(0);
}

inline int BlockMap::GetVoxId(const Eigen::Vector3i &pos, const shared_ptr<Grid_Block> &GB){//don't check, pos of world
    Eigen::Vector3i dpos = pos - GB->origin_;
    return dpos(2)*(GB->block_size_.x())*(GB->block_size_.y()) + dpos(1)*(GB->block_size_.x()) + dpos(0);
}

inline Eigen::Vector3d BlockMap::Id2LocalPos(const shared_ptr<Grid_Block> &GB, const int &id){
    int x = id % GB->block_size_(0);
    int y = ((id - x)/GB->block_size_(0)) % GB->block_size_(1);
    int z = ((id - x) - y*GB->block_size_(0))/GB->block_size_(1)/GB->block_size_(0);
    return Eigen::Vector3d((double(x)+0.5)*resolution_,(double(y)+0.5)*resolution_,(double(z)+0.5)*resolution_)+origin_ + GB->origin_.cast<double>() * resolution_;
}

inline bool BlockMap::InsideMap(const Eigen::Vector3i &pos){
    // cout<<"dpos:"<<dpos.transpose()<<endl;
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  voxel_num_(0) || pos(1) >= voxel_num_(1) || pos(2) >= voxel_num_(2))
        return false;
    return true;
}

inline bool BlockMap::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < map_lowbd_(0)|| pos(1) < map_lowbd_(1)|| pos(2) < map_lowbd_(2)||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline Eigen::Vector3d BlockMap::IdtoPos(int id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/voxel_num_(1)/voxel_num_(0);
    return Eigen::Vector3d((double(x)+0.5)*resolution_,(double(y)+0.5)*resolution_,(double(z)+0.5)*resolution_)+origin_;
}

inline void BlockMap::GetCastLine(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &line){
    RayCaster rc;
    Eigen::Vector3d ray_iter;
    Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;
    line.clear();
    rc.setInput((start - origin_) / resolution_, (end - origin_) / resolution_);
    while (rc.step(ray_iter))
    {
        ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
        line.emplace_back(ray_iter);
    }
}

inline int BlockMap::PostoId(const Eigen::Vector3d &pos){
    return floor((pos(2)-origin_(2))/resolution_)*voxel_num_(0)*voxel_num_(1)+
        floor((pos(1)-origin_(1))/resolution_)*voxel_num_(0)+floor((pos(0)-origin_(0))/resolution_);
}

inline Eigen::Vector3i BlockMap::PostoId3(const Eigen::Vector3d &pos){
    return Eigen::Vector3i((int)floor((pos(0) - origin_(0))/resolution_), (int)floor((pos(1) - origin_(1))/resolution_),
         (int)floor((pos(2) - origin_(2))/resolution_)); 
}


inline VoxelState BlockMap::GetVoxState(const int &id){
    Eigen::Vector3d pos = IdtoPos(id);
    
    int blockid = GetBlockId(pos);
    if(blockid == -1) {
        return VoxelState::out;
    }
    else if(GBS_[blockid]->state_ == GBSTATE::MIXED){
        float odds = GBS_[blockid]->odds_log_[GetVoxId(pos, GBS_[blockid])];
        if(odds > 0) return VoxelState::occupied;
        else if(odds < 0 && odds >= thr_min_) return VoxelState::free;
        else return VoxelState::unknown;
    } 
    else if(GBS_[blockid]->state_ == GBSTATE::FREE){
        return VoxelState::free;
    }
    else if(GBS_[blockid]->state_ == GBSTATE::OCCUPIED){
        return VoxelState::occupied;
    }
    else {
        return VoxelState::unknown;
    }
}

inline VoxelState BlockMap::GetVoxState(const Eigen::Vector3i &id){
    int voxid = id(0) + id(1) * voxel_num_(0) + id(2) * voxel_num_(0) * voxel_num_(1);
    return GetVoxState(voxid);
}

inline VoxelState BlockMap::GetVoxState(const Eigen::Vector3d &pos){
    int blockid = GetBlockId(pos);
    if(blockid != -1){
        shared_ptr<Grid_Block> GB_ptr = GBS_[blockid];
        if(GB_ptr->state_ == MIXED){
            float odds = GBS_[blockid]->odds_log_[GetVoxId(pos, GBS_[blockid])];
            // cout<<odds<<"  "<<thr_min_<<endl;
            if(odds > 0) return VoxelState::occupied;
            else if(odds < 0 && odds > thr_min_ - 1e-3) return VoxelState::free;
            else return VoxelState::unknown;
        }
        else if(GBS_[blockid]->state_ == GBSTATE::FREE){
            return VoxelState::free;
        }
        else if(GBS_[blockid]->state_ == GBSTATE::OCCUPIED){
            return VoxelState::occupied;
        }
        else{
            return VoxelState::unknown;
        }
    }
    else{
        return VoxelState::out;
    }
}

inline void BlockMap::GetRayEndInsideMap(const Eigen::Vector3d &start, Eigen::Vector3d &end, bool &occ){
    double lx, ly, lz;
    if(end(0) > map_upbd_(0)){
        lx = (map_upbd_(0) - start(0)) / (end(0) - start(0)) - 1e-4;
        occ = 0;
    }    
    else if(end(0) < map_lowbd_(0)){
        lx = (start(0) - map_lowbd_(0)) / (start(0) - end(0)) - 1e-4;
        occ = 0;
    }    
    else lx = 1.0;

    if(end(1) > map_upbd_(1)){
        ly = (map_upbd_(1) - start(1)) / (end(1) - start(1)) - 1e-4;
        occ = 0;
    }    
    else if(end(1) < map_lowbd_(1)){
        ly = (start(1) - map_lowbd_(1)) / (start(1) - end(1)) - 1e-4;
        occ = 0;
    }    
    else ly = 1.0;

    if(end(2) > map_upbd_(2)){
        lz = (map_upbd_(2) - start(2)) / (end(2) - start(2)) - 1e-4;
        occ = 0;
    }    
    else if(end(2) < map_lowbd_(2)){
        lz = (start(2) - map_lowbd_(2)) / (start(2) - end(2)) - 1e-4;
        occ = 0;
    }    
    else lz = 1.0;

    end = (end - start) * min(lx, min(ly, lz)) + start;
}

inline void BlockMap::SpointToUV(const double &x, const double &y, Eigen::Vector2i &uv){
    // return Eigen::Vector2i(fx_ * x + cx_, fy_ * y + cy_);
    uv.x() = fx_ * x + cx_;
    uv.y() = fy_ * y + cy_;
}

inline void BlockMap::UVToPoint(const int &u, const int &v, const double &depth, Eigen::Vector3d &point){
    point.x() = (u - cx_) * depth / fx_;
    point.y() = (v - cy_) * depth / fy_;
    point.z() = depth;
} 

inline void BlockMap::UpdateSwarmData(){
    Eigen::Quaterniond qua;
    Eigen::Vector3d pos;
    Eigen::Matrix4d pose;
    for(int i = 0; i < swarm_pose_.size(); i++){
        pose.setZero();
        if(i + 1 == self_id_) continue;
        qua.x() = SDM_->Poses_[i].orientation.x;
        qua.y() = SDM_->Poses_[i].orientation.y;
        qua.z() = SDM_->Poses_[i].orientation.z;
        qua.w() = SDM_->Poses_[i].orientation.w;
        pos(0) = SDM_->Poses_[i].position.x;
        pos(1) = SDM_->Poses_[i].position.y;
        pos(2) = SDM_->Poses_[i].position.z;
        // cout<<"use:"<<pos.transpose()<<" "<<endl;
        pose.block(0, 0, 3, 3) = qua.toRotationMatrix();
        pose.block(0, 3, 3, 1) = pos;
        pose(3, 3) = 1.0;
        swarm_pose_[i].second = pose;
        swarm_pose_[i].first = SDM_->Pose_t_[i];
    }
}

inline void BlockMap::LoadSwarmFilter(){
    if(!use_swarm_) return;
    UpdateSwarmData();
    swarm_filter_dict_->clear();
    double cur_t, max_r;
    Eigen::Matrix3d rot;
    Eigen::Vector3d p, c, r, n;
    Eigen::Vector3d cam = cam2world_.block(0,3,3,1);
    for(int i = 0; i < swarm_pose_.size(); i++){
        if(cur_t - swarm_pose_[i].first > swarm_tol_ || i + 1 == self_id_) continue;
        c =  swarm_pose_[i].second.block(0, 3, 3, 1);
        c = IdtoPos(PostoId(c));
        r = robots_scale_[i]/2 + Eigen::Vector3d::Ones() * resolution_;
        max_r = r.maxCoeff();
        r = r.cwiseProduct(r);
        if((cam - c).norm() > max_r + max_range_) continue;
        int k = ceil(max_r / resolution_);
        rot = swarm_pose_[i].second.block(0, 0, 3, 3).transpose(); 
        for(p(0) = c(0) - k * resolution_; p(0) <= c(0) + k * resolution_ + 1e-3; p(0) += resolution_){
            for(p(1) = c(1) - k * resolution_; p(1) <= c(1) + k * resolution_ + 1e-3; p(1) += resolution_){
                for(p(2) = c(2) - k * resolution_; p(2) <= c(2) + k * resolution_ + 1e-3; p(2) += resolution_){
                    if(!InsideMap(p))continue;
                    n = rot * (p - c);
                    if(n(0)*n(0)/r(0) + n(1)*n(1)/r(1) + n(2)*n(2)/r(2) < 1.0){//inside ellipsoid
                        swarm_filter_dict_->insert({PostoId(p), 0});
                    }
                }
            }
        }
    }
    // Debug();
}

inline std_msgs::ColorRGBA BlockMap::Getcolor(const double z){
    std_msgs::ColorRGBA color;
    double difz = z - origin_(2);
    color.a = 1.0;
    if(difz > map_upbd_(2)){
        return color_list_.back();
    }
    else if(difz < origin_(2)){
        return color_list_.front();
    }
    else{
        
        int hieghtf = floor(difz / colorhsize_);
        int hieghtc = hieghtf + 1;
        double gain = (difz - colorhsize_*hieghtf)/colorhsize_;
        color.r = color_list_[hieghtf].r*(1.0-gain) + color_list_[hieghtc].r*gain;
        color.g = color_list_[hieghtf].g*(1.0-gain) + color_list_[hieghtc].g*gain;
        color.b = color_list_[hieghtf].b*(1.0-gain) + color_list_[hieghtc].b*gain;
    }
    return color;
}

#endif