#ifndef SWARM_DATA_H_
#define SWARM_DATA_H_
#include <ros/ros.h>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <list>
#include <tr1/unordered_map>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
// #include <swarm_exp_msgs/IdOdom.h>
#include <data_statistics/computation_statistician.h>

#include <swarm_exp_msgs/IdPose.h>
#include <swarm_exp_msgs/SwarmTraj.h>
#include <swarm_exp_msgs/DtgFFEdge.h>
#include <swarm_exp_msgs/DtgHFEdge.h>
#include <swarm_exp_msgs/DtgHHEdge.h>
#include <swarm_exp_msgs/DtgHNode.h>
#include <swarm_exp_msgs/DtgFNode.h>
#include <swarm_exp_msgs/DtgBag.h>

#include <exp_comm_msgs/DtgBagC.h>
#include <exp_comm_msgs/IdPoseC.h>
#include <exp_comm_msgs/SwarmTrajC.h>
#include <exp_comm_msgs/DtgBagAnswer.h>
#include <exp_comm_msgs/SwarmJobC.h>
#include <exp_comm_msgs/SwarmStateC.h>
#include <exp_comm_msgs/MapC.h>
#include <exp_comm_msgs/MapReqC.h>

#include "gcopter/trajectory.hpp"

using namespace std;


class SwarmDataManager{
public:
    SwarmDataManager(){};
    ~SwarmDataManager(){};

    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    inline void SetFrontierVpNum(const int vp_num);

    inline bool IsEucLocal(geometry_msgs::Pose &Pose1, geometry_msgs::Pose &Pose2);
    inline bool IsEucLocalUAV(const uint8_t &uav_id);
    inline void SetSelfNeiHnodes(const vector<uint32_t> &local_h);
    inline bool IsLocalUAV(const uint8_t &uav_id);

    inline void SetPose(const nav_msgs::Odometry &odom);
    inline void SetTraj(Trajectory<5> &traj, const double &s_t);
    inline void SetDTGHn(const uint32_t &id, const uint32_t &pos_idx);
    // inline void SetDTGFn(const swarm_exp_msgs::DtgFNode &fn);
    // inline void SetDTGFnDeadvps(const uint16_t &f_id, list<uint8_t> &dieing_vps);
    // inline void SetDTGFnDead(const uint16_t &f_id);
    inline void SetDTGFn(const uint16_t &f_id, vector<uint8_t> &local_vps_, uint8_t flag, bool alive);
    inline void SetDeadvp(const uint16_t &v_id, vector<uint8_t> &vp_flags);
    inline void SetAlivevp(const uint16_t &v_id, vector<uint8_t> &vp_flags);


    // inline void EraseDTGHFEdge(const uint16_t &f_id);
    // inline void EraseDTGHHEdge(const uint32_t &head_id, const uint32_t &tail_id);
    inline void SetDTGFFEdge(const swarm_exp_msgs::DtgFFEdge &ffe);
    inline void SetDTGHFEdge(const uint32_t &head_id, const uint32_t &tail_id, const uint32_t &vp_id, const vector<uint32_t> &path);
    inline void SetDTGHHEdge(const uint32_t &head_id, const uint32_t &tail_id, const vector<uint32_t> &path);
    inline void SetJob(const uint8_t &jobstate, const int &target_hn, const double &dist_to_hn, const int &target_fn, const double &dist_to_fn);
    inline void SetState(const uint8_t &flags, list<uint16_t> &local_f, list<double> &dist_to_local_f, list<uint16_t> &connect_f, 
                            list<double> &dist_to_connect_f, list<pair<uint16_t, uint32_t>> &connect_h, list<double> &dist_to_connect_h);
    inline void SetMap(exp_comm_msgs::MapC &msg);
    inline void SetMapReq(exp_comm_msgs::MapReqC &msg);

    inline void GetFnvp(vector<uint8_t> &vp_flags, vector<uint8_t> &vp_states);
    inline void GetLocalSwarmPos(vector<Eigen::Vector3d> &swarm_pos);

    void PoseTimerCallback(const ros::TimerEvent &e);
    void TrajTimerCallback(const ros::TimerEvent  &e);
    void DTGTimerCallback(const ros::TimerEvent &e);
    // void JobTimerCallback(const ros::TimerEvent &e);
    void MapTimerCallback(const ros::TimerEvent &e);
    void DTGAnsTimerCallback(const ros::TimerEvent &e);

    void PoseMSGCallback(const exp_comm_msgs::IdPoseCConstPtr &msg);
    void TrajMSGCallback(const exp_comm_msgs::SwarmTrajCConstPtr &msg);
    void DTGMSGCallback(const exp_comm_msgs::DtgBagCConstPtr &msg);
    void DTGAnsMSGCallback(const exp_comm_msgs::DtgBagAnswerConstPtr &msg);
    void JobMSGCallback(const exp_comm_msgs::SwarmJobCConstPtr &msg);
    void StateMSGCallback(const exp_comm_msgs::SwarmStateCConstPtr &msg);
    void MapMSGCallback(const exp_comm_msgs::MapCConstPtr &msg);
    void MapReqMSGCallback(const exp_comm_msgs::MapReqCConstPtr &msg);

    void ShowTraj(const uint8_t &id);
    /* pose, trajectory, DTG, working state and map(ground <--> UAVs) communication */
    ros::Timer pose_timer_, traj_timer_, DTG_timer_, ans_timer_, map_timer_; /*job_timer_*/ // map_timer_; 
    double pose_pub_g_t_, job_pub_g_t_; //global timer

    ros::Publisher pose_pub_, traj_pub_, DTG_pub_, ans_pub_, job_pub_, state_pub_, map_pub_, mapreq_pub_;
    ros::Subscriber pose_sub_, traj_sub_, DTG_sub_, ans_sub_, job_sub_, state_sub_, map_sub_, mapreq_sub_;

    bool have_ground_, is_ground_;
    uint8_t flags_;// 0(Pose global)(Pose local)(Job global) (Job local)(DTG)(traj)(map)

    /* swarm data */
    uint8_t drone_num_;                   //1-255
    uint8_t self_id_;                      //1-255
    uint8_t ground_id_;                    //0

    /* trajectory */
    vector<Trajectory<5>> trajs_;  
    vector<double> start_t_;  
    pair<list<uint8_t>, swarm_exp_msgs::SwarmTraj> traj_msg_;

    vector<geometry_msgs::Pose> Poses_;
    vector<double> Pose_t_;

    /* DTG recieve from other UAVs */
    list<swarm_exp_msgs::DtgFFEdge> swarm_sub_ffe_;
    list<swarm_exp_msgs::DtgHFEdge> swarm_sub_hfe_;
    list<swarm_exp_msgs::DtgHHEdge> swarm_sub_hhe_;
    list<swarm_exp_msgs::DtgHNode> swarm_sub_hn_;
    list<swarm_exp_msgs::DtgFNode> swarm_sub_fn_;

    /* DTG pub to other UAVs */
    vector<swarm_exp_msgs::DtgFFEdge> swarm_pub_ffe_;
    vector<swarm_exp_msgs::DtgHFEdge> swarm_pub_hfe_;
    vector<swarm_exp_msgs::DtgHHEdge> swarm_pub_hhe_;
    vector<swarm_exp_msgs::DtgHNode> swarm_pub_hn_;
    vector<swarm_exp_msgs::DtgFNode> swarm_pub_fn_;

    /* Job recieve from other UAVs */
    vector<list<exp_comm_msgs::SwarmJobC>> jobs_;
    vector<exp_comm_msgs::SwarmStateC> states_;

    /* DTG controversial msg */
    list<pair<double, swarm_exp_msgs::DtgBag>> bag_answer_;

    /* mapping msgs for ground */
    list<exp_comm_msgs::MapC> swarm_sub_map_;
    exp_comm_msgs::MapReqC mreq_;
    bool req_flag_;
    uint8_t finish_num_;
    double finish_thresh_;
    vector<bool> finish_list_;
    // ofstream debug_l_;

    exp_comm_msgs::DtgBagAnswer ba_;
    bool use_answer_;
    double wait_t_;
    int fn_vp_size_, vp_max_;
    uint32_t bag_id_;

    vector<uint32_t> swarm_root_;                    //the root hnode of each robot
    vector<uint32_t> self_neighbor_hns_;             //the neighbor hnodes of the robot, including root_
    uint32_t vp_num_;
    double local_comm_intv_, global_comm_intv_;
    double local_dist_thresh_;

    bool show_swarm_traj_;
    ros::Publisher show_pub_;

    bool statistic_;
    ComputationStatistician CS_;
};

inline void SwarmDataManager::SetFrontierVpNum(const int vp_num){
    fn_vp_size_ = ceil(vp_num / 4);
    vp_max_ = vp_num;
    cout<<"fn_vp_size_:"<<fn_vp_size_<<endl;
    cout<<"vp_max_:"<<vp_max_<<endl;
}

inline bool SwarmDataManager::IsEucLocal(geometry_msgs::Pose &Pose1, geometry_msgs::Pose &Pose2){
    double dist = pow(Pose1.position.x - Pose2.position.x, 2) + pow(Pose1.position.y - Pose2.position.y, 2) + pow(Pose1.position.z - Pose2.position.z, 2);
    return dist < pow(local_dist_thresh_, 2);
}

inline bool SwarmDataManager::IsEucLocalUAV(const uint8_t &uav_id){
    return IsEucLocal(Poses_[uav_id - 1], Poses_[self_id_ - 1]);
}

inline void SwarmDataManager::SetSelfNeiHnodes(const vector<uint32_t> &local_h){
    self_neighbor_hns_ = local_h;
}

inline bool SwarmDataManager::IsLocalUAV(const uint8_t &uav_id){
    if(uav_id < 1 || uav_id > drone_num_ || uav_id == self_id_) return false;
    Eigen::Vector3d self_p, other_p;
    self_p(0) = Poses_[self_id_-1].position.x;
    self_p(1) = Poses_[self_id_-1].position.y;
    self_p(2) = Poses_[self_id_-1].position.z;
    other_p(0) = Poses_[uav_id-1].position.x;
    other_p(1) = Poses_[uav_id-1].position.y;
    other_p(2) = Poses_[uav_id-1].position.z;
    if((self_p - other_p).norm() > local_dist_thresh_) return false;

    for(auto &hid : self_neighbor_hns_){
        if(hid == swarm_root_[uav_id]){
            return true;
        }
    }
    return false;
}

inline void SwarmDataManager::SetPose(const nav_msgs::Odometry &odom){
    flags_ |= 96;
    Poses_[self_id_-1] = odom.pose.pose;
    Pose_t_[self_id_-1] = ros::WallTime::now().toSec();
}

inline void SwarmDataManager::SetTraj(Trajectory<5> &traj, const double &s_t){
    flags_ |= 2;
    trajs_[self_id_ - 1] = traj;
    traj_msg_.first.emplace_back(0);
    for(uint8_t i = 0; i < drone_num_; i++){
        if(i + 1 == self_id_) continue;
        traj_msg_.first.emplace_back(i + 1);
    }
    traj_msg_.second.from_uav = self_id_;
    traj_msg_.second.to_uavs.clear();
    traj_msg_.second.start_t = s_t;
    traj_msg_.second.coef_p.resize(traj.getPieceNum() * 6);
    traj_msg_.second.t_p.resize(traj.getPieceNum());
    traj_msg_.second.order_p = 5;
    for(int i = 0; i < traj.getPieceNum(); i++){
        auto &cur_p = traj[i];
        Eigen::MatrixXd cM;
        cM = cur_p.getCoeffMat();
        traj_msg_.second.t_p[i] = cur_p.getDuration();
        for(int j = 0; j < cM.cols(); j++){
            traj_msg_.second.coef_p[j + i * 6].x = cM(0, j);
            traj_msg_.second.coef_p[j + i * 6].y = cM(1, j);
            traj_msg_.second.coef_p[j + i * 6].z = cM(2, j);
        }
    }
}

inline void SwarmDataManager::SetDTGHn(const uint32_t &id, const uint32_t &pos_idx){
    flags_ |= 4;
    swarm_exp_msgs::DtgHNode hn;
    hn.h_id = id;
    hn.pos_idx = pos_idx;
    swarm_pub_hn_.emplace_back(hn);
}

inline void SwarmDataManager::SetDTGFn(const uint16_t &f_id, vector<uint8_t> &local_vps_, uint8_t flag, bool alive){
    flags_ |= 4;
    bool new_g_n = true;

    if(f_id == 65535) ROS_ERROR("error 65535 fn vps");
    for(auto &cfn : swarm_pub_fn_){
        if(cfn.f_id == f_id){
            if(cfn.alive){
                cfn.need_help = flag;
                if(!alive){
                    cfn.alive = false;
                    cfn.vp_flags.clear();
                }
            }
            new_g_n = false;
            break;
        }
    }
    if(new_g_n) {
        swarm_exp_msgs::DtgFNode fn;
        fn.f_id = f_id;
        if(alive){
            fn.f_id = f_id;
            fn.alive = false;
            fn.need_help = flag;
            fn.vp_flags.resize(fn_vp_size_, 0);
            for(int i = 0; i < local_vps_.size(); i++){
                if(local_vps_[i] == 1){
                    SetAlivevp(i, fn.vp_flags);
                    fn.alive = true;
                } 
                else if(local_vps_[i] == 2) SetDeadvp(i, fn.vp_flags);
                else {
                    fn.alive = true;
                    // fn. = 0;
                }
            }
            if(!fn.alive) fn.vp_flags.clear();
        }
        else fn.alive = false;
        swarm_pub_fn_.emplace_back(fn);
    }
}

inline void SwarmDataManager::SetDeadvp(const uint16_t &v_id, vector<uint8_t> &vp_flags){
    if(v_id < 0 || v_id > vp_max_) return;
    int id1 = floor(v_id / 4);
    int id2 = v_id % 4;
    int cl;
    vp_flags[id1] &= (255 - (3<<(id2*2)));
    uint8_t flag = 1 << (id2 * 2); 
    vp_flags[id1] |= flag;
}

inline void SwarmDataManager::SetAlivevp(const uint16_t &v_id, vector<uint8_t> &vp_flags){
    if(v_id < 0 || v_id > vp_max_) return;
    int id1 = floor(v_id / 4);
    int id2 = v_id % 4;
    int cl;
    vp_flags[id1] &= (255 - (3<<(id2*2)));
    uint8_t flag = 2 << (id2 * 2); 
    vp_flags[id1] |= flag;
}

inline void SwarmDataManager::GetFnvp(vector<uint8_t> &vp_flags, vector<uint8_t> &vp_states){
    vp_flags.clear();
    vp_flags.resize(vp_max_, 0);
    int id;
    uint8_t flag1, flag2;
    // cout<<"vp_states:"<<vp_states.size()<<endl;
    for(int i = 0; i < fn_vp_size_; i++){
        // cout<<"vp_states i:"<<int(vp_states[i])<<endl;
        for(int j = 0; j < 4; j++){
            id = i * 4 + j;
            if(id >= vp_max_) break;
            flag1 = 1<<(2*j);
            flag2 = 2<<(2*j);
            if(flag1 & vp_states[i]){
                vp_flags[id] = 2;
            }
            else if(flag2 & vp_states[i]){
                vp_flags[id] = 1;
            }
        }
    }
}

// inline void SwarmDataManager::EraseDTGHFEdge(const uint16_t &f_id){
//     flags_ |= 4;
//     bool new_l_e = true;

//     for(auto &lhfe : swarm_pub_hfe_){
//         if(lhfe.f_id == f_id){
//             lhfe.erase = false;
//             lhfe.pub_t = ros::WallTime().toSec();
//             new_l_e = false;
//             break;
//         }
//     }
//     if(new_l_e){
//         swarm_exp_msgs::DtgHFEdge edge;
//         edge.f_id = f_id;
//         edge.erase = false;
//         edge.pub_t = ros::WallTime().toSec();
//         edge.hfe_id = self_id_;
//         swarm_pub_hfe_.emplace_back(edge);
//     }
// }

// inline void SwarmDataManager::EraseDTGHHEdge(const uint32_t &head_id, const uint32_t &tail_id){
//     flags_ |= 4;
//     bool new_l_e = true;

//     for(auto &lhhe : swarm_pub_hhe_){
//         if((lhhe.head_h_id == head_id && lhhe.tail_h_id == tail_id) ||
//                 (lhhe.tail_h_id == head_id && lhhe.head_h_id == tail_id)){
//             lhhe.points_idx.clear();
//             lhhe.erase = true;
//             lhhe.pub_t = ros::WallTime().toSec();
//             new_l_e = false;
//             break;
//         }
//     }
//     if(new_l_e){
//         swarm_exp_msgs::DtgHHEdge edge;
//         edge.head_h_id = head_id;
//         edge.tail_h_id = tail_id;
//         edge.pub_t = ros::WallTime().toSec();
//         edge.erase = true;
//         edge.hhe_id = self_id_;
//         swarm_pub_hhe_.emplace_back(edge);
//     }
// }

inline void SwarmDataManager::SetDTGFFEdge(const swarm_exp_msgs::DtgFFEdge &ffe){
    flags_ |= 4;
    bool new_l_e = true;

    for(auto &lffe : swarm_pub_ffe_){
        if((lffe.head_f_id == ffe.head_f_id && lffe.tail_f_id == ffe.tail_f_id) || 
                (lffe.tail_f_id == ffe.head_f_id && lffe.head_f_id == ffe.tail_f_id)){
            lffe = ffe;
            new_l_e = false;
            break;
        }
    }
    if(new_l_e) swarm_pub_ffe_.emplace_back(ffe);
}

inline void SwarmDataManager::SetDTGHFEdge(const uint32_t &head_id, const uint32_t &tail_id, const uint32_t &vp_id, const vector<uint32_t> &path){
    flags_ |= 4;
    bool new_l_e = true;

    for(auto &lhfe : swarm_pub_hfe_){
        if(lhfe.f_id == tail_id){
            // lhfe.h_id = tail_id;
            lhfe.points_idx = path;
            lhfe.vp_id = vp_id;
            // lhfe.erase = false;
            lhfe.pub_t = ros::WallTime().toSec();
            new_l_e = false;
            break;
        }
    }
    if(new_l_e){
        swarm_exp_msgs::DtgHFEdge edge;
        edge.h_id = head_id;
        edge.f_id = tail_id;
        edge.vp_id = vp_id;
        // edge.erase = false;
        edge.pub_t = ros::WallTime().toSec();
        edge.points_idx = path;
        // edge.hfe_id = self_id_;
        swarm_pub_hfe_.emplace_back(edge);
    }
}

inline void SwarmDataManager::SetDTGHHEdge(const uint32_t &head_id, const uint32_t &tail_id, const vector<uint32_t> &path){
    flags_ |= 4;
    bool new_l_e = true;

    for(auto &lhhe : swarm_pub_hhe_){
        if((lhhe.head_h_id == head_id && lhhe.tail_h_id == tail_id) ||
                (lhhe.tail_h_id == head_id && lhhe.head_h_id == tail_id)){
            lhhe.points_idx = path;
            // lhhe.erase = false;
            lhhe.pub_t = ros::WallTime().toSec();
            new_l_e = false;
            break;
        }
    }
    if(new_l_e){
        swarm_exp_msgs::DtgHHEdge edge;
        edge.head_h_id = head_id;
        edge.tail_h_id = tail_id;
        edge.points_idx = path;
        // edge.erase = false;
        edge.pub_t = ros::WallTime().toSec();
        // edge.hhe_id = self_id_;
        swarm_pub_hhe_.emplace_back(edge);
    }
}

inline void SwarmDataManager::SetJob(const uint8_t &jobstate, const int &target_hn, 
                                        const double &dist_to_hn, const int &target_fn, const double &dist_to_fn){
    exp_comm_msgs::SwarmJobC job_msg;
    if(jobstate & 2){
    }
    job_msg.from_uav = self_id_;
    job_msg.pub_t = ros::WallTime::now().toSec();
    job_msg.JobState = jobstate;
    job_msg.target_hn = target_hn;
    job_msg.target_fn = target_fn;
    job_msg.dist_to_fn = dist_to_hn;
    job_msg.dist_to_hn = dist_to_fn;
    job_pub_.publish(job_msg);
}

inline void SwarmDataManager::SetState(const uint8_t &flags, list<uint16_t> &local_f, list<double> &dist_to_local_f, list<uint16_t> &connect_f, 
                            list<double> &dist_to_connect_f, list<pair<uint16_t, uint32_t>> &connect_h, list<double> &dist_to_connect_h){
    exp_comm_msgs::SwarmStateC state_msg;
    state_msg.flags = flags;
    state_msg.from_uav = self_id_;
    for(auto &f : local_f) state_msg.local_fn.emplace_back(f);
    for(auto &d : dist_to_local_f) state_msg.dist_to_local_fn.emplace_back(float(d));

    for(auto &f : connect_f) state_msg.connect_fn.emplace_back(f);
    for(auto &d : dist_to_connect_f) state_msg.dist_to_connect_fn.emplace_back(float(d));

    for(auto &h : connect_h) state_msg.connect_hn.emplace_back(h.second);
    for(auto &h : connect_h) state_msg.hn_pos_idx.emplace_back(h.first);
    for(auto &d : dist_to_connect_h) state_msg.dist_to_connect_hn.emplace_back(float(d));
    state_msg.pub_t = ros::WallTime::now().toSec();
    if(local_f.size() != dist_to_local_f.size()) ROS_ERROR("error state send f");
    if(connect_f.size() != dist_to_connect_f.size()) ROS_ERROR("error state send connect f");
    if(connect_h.size() != dist_to_connect_h.size()) ROS_ERROR("error state send h");
    state_pub_.publish(state_msg);
}

inline void SwarmDataManager::SetMap(exp_comm_msgs::MapC &msg){
    map_pub_.publish(msg);
}

inline void SwarmDataManager::SetMapReq(exp_comm_msgs::MapReqC &msg){
    mapreq_pub_.publish(msg);
}

inline void SwarmDataManager::GetLocalSwarmPos(vector<Eigen::Vector3d> &swarm_pos){
    swarm_pos.clear();
    for(int i = 0; i < drone_num_; i++){
        // if(i + 1 == self_id_) continue;
        Eigen::Vector3d pos;
        pos(0) = Poses_[i].position.x;
        pos(1) = Poses_[i].position.y;
        pos(2) = Poses_[i].position.z;

        swarm_pos.emplace_back(pos);
    }
}
#endif