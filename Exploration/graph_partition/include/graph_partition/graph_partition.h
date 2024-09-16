#ifndef GRAPH_PARTITION_H_
#define GRAPH_PARTITION_H_

#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <visualization_msgs/MarkerArray.h>

#include <tr1/unordered_map>
#include <frontier_grid/frontier_grid.h>

#include <multiDTG/multiDTG.h>
#include <block_map/color_manager.h>
#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <swarm_data/swarm_data.h>
#include <block_map/color_manager.h>
#include <yaw_planner/yaw_planner.h>

using namespace std;
namespace DTG{
class GraphVoronoiPartition{
public:
    GraphVoronoiPartition(){};
    ~GraphVoronoiPartition(){};
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void SetLowresMap(lowres::LowResMap *LRM){LRM_ = LRM;};
    void SetDTGMap(MultiDTG *MDTG){MDTG_ = MDTG;};
    void SetFrontierMap(FrontierGrid *FG){FG_ = FG;};
    void SetColorManager(ColorManager *CM){CM_ = CM;};
    void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;};
    void LocalGVP();
    void GlobalGVP();
    void SetVA(const double v, const double a){max_v_ = v; max_a_ = a;};
    void SetYawP(YawPlanner *yawp){YawP_ = yawp;};
    /**
     * @brief Get the Local F Nodes object
     * 
     * @param c_state   current pose
     * @param c_vel     current vel
     * @param path      path to target state
     * @param t_state   target state
     * @param f_v       target frontier, viewpoint
     * @param exp_state 0: free path, 1: dangerous path, 2: no path / no target
     */
    void GetLocalFNodes(const Eigen::Vector4d &c_state, const Eigen::Vector3d &c_vel, list<Eigen::Vector3d> &path, double &exp_length, 
                            Eigen::Vector4d &t_state, pair<int, int> &f_v, int &h_id, int &exp_state);

    /**
     * @brief Get the Global F Nodes object
     * 
     * @param c_state   current pose
     * @param path      path to target state
     * @param t_state   target state
     * @param f_v       target frontier, viewpoint
     * @param exp_state -2: free path follow, -1: dangerous path follow, 0: free path to fn, 1: dangerous path to fn, 2: no path / no target
     */
    void GetGlobalFNodes(const Eigen::Vector4d &c_state, list<Eigen::Vector3d> &path, double &exp_length, 
                            Eigen::Vector4d &t_state, pair<int, int> &f_v, int &h_id, int &exp_state);

    bool LocalExplorable();
    void LocalExplorableDebug();
    bool GlobalExplorable();
    inline void SetJob(const uint8_t &jobstate, const int &target_hn, const double &dist_to_hn, const int &target_fn, const double &dist_to_fn);
    list<pair<uint16_t, uint32_t>> *GVP_hn_;  // self global partition
    list<pair<uint16_t, uint32_t>> *local_hn_;  // self local partition
    list<uint16_t> *local_fn_;   // self exploring fn
    list<double> dist_to_f_;
private:
    void LoadLocalData(list<h_ptr> &hn_l, list<f_ptr> &fn_l, list<h_ptr> &fake_hn_l);
    void LoadGlobalData(list<h_ptr> &fake_hn_l);
    inline void CreateFakeHnode(const uint8_t &id, h_ptr &hn);
    inline void CreateFakeEdge(h_ptr &hn_head, h_ptr &hn_tail, const double &dist);
    inline void CreateFakeEdge(h_ptr &hn_head, f_ptr &hn_tail, const double &dist);

    inline void LoadSwarmJob(const exp_comm_msgs::SwarmJobC &job);
    inline void LoadSwarmState(const exp_comm_msgs::SwarmStateC &state);

    void HandleLocalData(list<h_ptr> &hn_l, list<f_ptr> &fn_l, list<h_ptr> &fake_hn_l);
    void HandleGlobalData(list<h_ptr> &hn_l);

    /**
     * @brief 
     * 
     * @param f_target 
     * @return  false: explore, true: follow 
     */
    bool ExploreOrFollow(f_ptr &f_target);

    bool GetBestTarget(list<pair<double, list<Eigen::Vector3d>>> &d_p, list<h_ptr> &hn_l, f_ptr &fn_t,
                        list<Eigen::Vector3d> &path, Eigen::Vector4d &t_state, pair<int, int> &f_v, int &h_id);

    void JobTimerCallback(const ros::TimerEvent &e);
    void StateTimerCallback(const ros::TimerEvent &e);
    void PartitionTimerCallback(const ros::TimerEvent &e);

    void ShowPartition(list<h_ptr> &hn_l);
    void Debug(list<Eigen::Vector3d>  &fl, int type, int pub_id);

    YawPlanner *YawP_;
    MultiDTG *MDTG_;
    SwarmDataManager *SDM_;
    lowres::LowResMap *LRM_;
    ColorManager *CM_;
    FrontierGrid *FG_;

    vector<uint8_t> swarm_job_state_;
    vector<double> last_new_job_; //swarm job pub_t
    /* dont mind self of swarm... */
    vector<double> swarm_job_f_dist_;
    vector<int> swarm_job_f_;
    vector<double> swarm_job_h_dist_;
    vector<int> swarm_job_h_;

    /* swarm partition nodes, no self */
    vector<list<double>> swarm_part_f_dist_;
    vector<list<uint16_t>> swarm_part_f_id_;
    /* swarm connecting nodes */
    vector<list<double>> swarm_connect_f_dist_;
    vector<list<uint16_t>> swarm_connect_f_id_;
    vector<list<double>> swarm_connect_h_dist_;
    vector<list<pair<uint16_t, uint32_t>>> swarm_connect_h_; // connecting hns
    
    bool show_gvd_; 
    ros::Timer job_timer_, state_timer_, GVP_timer_, show_timer_;
    ros::Publisher show_pub_, debug_pub_;

    double max_v_, max_a_, max_yawd_;
    double lambda_, allowance_;
    double tau_;
    double last_state_pub_t_;
};

inline void GraphVoronoiPartition::SetJob(const uint8_t &jobstate, const int &target_hn, 
                                            const double &dist_to_hn, const int &target_fn, const double &dist_to_fn){
    int idx = SDM_->self_id_ - 1;
    SDM_->SetJob(jobstate, target_hn, dist_to_hn, target_fn, dist_to_fn);
    last_new_job_[idx] = ros::WallTime().now().toSec();
    swarm_job_f_dist_[idx] = dist_to_fn;
    swarm_job_f_[idx] = target_fn;
    swarm_job_h_dist_[idx] = dist_to_hn;
    swarm_job_h_[idx] = target_hn;
}

inline void GraphVoronoiPartition::CreateFakeHnode(const uint8_t &id, h_ptr &hn){
    hn = make_shared<H_node>();
    hn->id_ = id;
}

inline void GraphVoronoiPartition::CreateFakeEdge(h_ptr &hn_head, h_ptr &hn_tail, const double &dist){
    hhe_ptr e = make_shared<hhe>();
    e->e_flag_ = 40;
    e->head_n_ = hn_head;
    e->head_ = hn_head->id_;
    e->tail_n_ = hn_tail;
    e->tail_ = hn_tail->id_;
    e->length_s_ = dist;
    e->length_ = dist;
    hn_head->hh_edges_.emplace_back(e);
}

inline void GraphVoronoiPartition::CreateFakeEdge(h_ptr &hn_head, f_ptr &fn_tail, const double &dist){
    hfe_ptr e = make_shared<hfe>();
    e->e_flag_ = 24;
    e->head_n_ = hn_head;
    e->head_ = hn_head->id_;
    e->tail_n_ = fn_tail;
    e->tail_ = fn_tail->id_;
    e->length_ = dist;
    hn_head->hf_edges_.emplace_back(e);
}

inline void GraphVoronoiPartition::LoadSwarmJob(const exp_comm_msgs::SwarmJobC &job){
    int idx = job.from_uav - 1;

    if((job.JobState & 4) && job.pub_t > last_new_job_[idx]){
        last_new_job_[idx] = job.pub_t;
        swarm_job_f_dist_[idx] = job.dist_to_fn;
        swarm_job_f_[idx] = job.dist_to_fn;
        swarm_job_h_dist_[idx] = job.dist_to_hn;
        swarm_job_h_[idx] = job.target_hn;
        
    }
    else if(!(job.JobState & 4) && job.pub_t > last_new_job_[idx]){
        last_new_job_[idx] = job.pub_t;
        swarm_job_f_dist_[idx] = 9999.0;
        swarm_job_f_[idx] = -1;
        swarm_job_h_dist_[idx] = 9999.0;
        swarm_job_h_[idx] = -1;
    }
}

inline void GraphVoronoiPartition::LoadSwarmState(const exp_comm_msgs::SwarmStateC &state){
    int idx = state.from_uav - 1;
    uint8_t old_owner;
    if(state.pub_t > last_new_job_[idx] + 1e-3){
        last_new_job_[idx] = state.pub_t;

        if(!(state.flags & 1)){// partition, clear fn owner
            for(auto &f_id : swarm_part_f_id_[idx]){
                FG_->ClearOwner(f_id);
                MDTG_->F_depot_[f_id]->f_flag_ &= 253;
            }
            swarm_part_f_dist_[idx].clear();
            swarm_part_f_id_[idx].clear();
        }

        for(int i = 0; i < state.dist_to_local_fn.size(); i++){
            if(FG_->ChangeOwner(state.local_fn[i], state.from_uav, state.dist_to_local_fn[i], old_owner)){
                if(old_owner != 0){
                    list<uint16_t>::iterator f_it = swarm_part_f_id_[old_owner - 1].begin();
                    list<double>::iterator fe_it = swarm_part_f_dist_[old_owner - 1].begin();
                    for(; f_it != swarm_part_f_id_[old_owner - 1].end(); f_it++, fe_it++){
                        if(*f_it == state.local_fn[i]){
                            swarm_part_f_id_[old_owner - 1].erase(f_it);
                            swarm_part_f_dist_[old_owner - 1].erase(fe_it);
                            break;
                        }
                    }
                }
                swarm_part_f_id_[idx].emplace_back(state.local_fn[i]);
                swarm_part_f_dist_[idx].emplace_back(state.dist_to_local_fn[i]);
                // ROS_WARN("id:%d   old:%d ,dist:%lf ,success change:%d ,dist:%lf", SDM_->self_id_, old_owner, 
                //             FG_->f_grid_[state.local_fn[i]].owner_dist_, state.from_uav, state.dist_to_local_fn[i]);
                // cout<<"state:"<<int(FG_->f_grid_[state.local_fn[i]].f_state_)<<" msg flag:"<<int(state.flags)<<" f:"<<int(state.local_fn[i])<<endl;
            }
            else{
                // ROS_ERROR("id:%d   old:%d ,dist:%lf ,fail change:%d ,dist:%lf", SDM_->self_id_, old_owner, 
                //             FG_->f_grid_[state.local_fn[i]].owner_dist_, state.from_uav, state.dist_to_local_fn[i]);
                // cout<<"state:"<<int(FG_->f_grid_[state.local_fn[i]].f_state_)<<" msg flag:"<<int(state.flags)<<" f:"<<int(state.local_fn[i])<<endl;
            }
        }


        swarm_connect_f_dist_[idx].clear();
        swarm_connect_f_id_[idx].clear(); 
        for(auto &fd : state.dist_to_connect_fn) swarm_connect_f_dist_[idx].emplace_back(fd);
        for(auto &f : state.connect_fn) swarm_connect_f_id_[idx].emplace_back(f);
        // if(swarm_connect_f_dist_.size() != )

        swarm_connect_h_dist_[idx].clear();
        swarm_connect_h_[idx].clear();
        for(auto &dth : state.dist_to_connect_hn) swarm_connect_h_dist_[idx].emplace_back(dth);
        for(int i = 0; i < state.connect_hn.size(); i++){
            Eigen::Vector3d pos = LRM_->IdtoPos(state.hn_pos_idx[i]);
            swarm_connect_h_[idx].push_back({state.hn_pos_idx[i], state.connect_hn[i]});
        }
    }
}
}


#endif