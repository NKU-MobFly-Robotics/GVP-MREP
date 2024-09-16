#ifndef MULTI_DTG_H_
#define MULTI_DTG_H_

#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/DtgFFEdge.h>
#include <swarm_exp_msgs/DtgHFEdge.h>
#include <swarm_exp_msgs/DtgHHEdge.h>
#include <swarm_exp_msgs/DtgHNode.h>
#include <swarm_exp_msgs/DtgFNode.h>

#include <tr1/unordered_map>
#include <frontier_grid/frontier_grid.h>

#include <multiDTG/dtg_structures.h>
#include <block_map/color_manager.h>
#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <swarm_data/swarm_data.h>

using namespace std;
namespace DTG{
typedef shared_ptr<H_node> h_ptr;
typedef shared_ptr<F_node> f_ptr;
typedef shared_ptr<DTG_edge<H_node, F_node>> hfe_ptr; // only length_ is used
typedef shared_ptr<DTG_edge<H_node, H_node>> hhe_ptr;
typedef shared_ptr<DTG_edge<F_node, F_node>> ffe_ptr;
typedef DTG_edge<H_node, F_node> hfe;
typedef DTG_edge<H_node, H_node> hhe;
typedef DTG_edge<F_node, F_node> ffe;
class GraphVoronoiPartition;
class MultiDTG{

public:
    MultiDTG(){};
    ~MultiDTG(){};

    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void GetHnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<h_ptr> &H_list);
    void GetFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<f_ptr> &F_list);
    void GetHnodesAndFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd,
         list<h_ptr> &H_list, list<f_ptr> &F_list);

    /**
     * @brief Get the Local frontier and viewpoints 
     * 
     * @param p     local center
     * @param d_f_v <dist, <f_id, v_id>>
     */
    void GetLocalFnDetail(const Eigen::Vector3d &p, list<pair<double, pair<int, int>>> &d_f_v);
    void Update(const Eigen::Matrix4d &robot_pose, bool clear_x = false);
    void SetLowresMap(lowres::LowResMap *LRM){LRM_ = LRM;};
    void SetBlockMap(BlockMap *BM){BM_ = BM;};
    void SetFrontierMap(FrontierGrid *FG){FG_ = FG;};
    void SetColorManager(ColorManager *CM){CM_ = CM;};
    void SetSwarmDataManager(SwarmDataManager *SDM){SDM_ = SDM;};
    inline void EraseFnode(const Eigen::Vector3d &center, const uint32_t &id);

    void RemoveVp(const Eigen::Vector3d &center, int const &f_id, int const &v_id, bool broad_cast = false);

    /**
     * @brief Get the Closest Local Target
     * 
     * @return ffe_ptr: the edge to the closest target. If no local target, return Null.
     */
    ffe_ptr GetClosestLocalTarget();

    /**
     * @brief Get the Closest Global Frontier Node. Djkstra.
     * 
     * @param path          path to the corresponding frontier viewpoint
     * @param H_path        hnodes of the path passed by
     * @param f_id          frontier id
     * @param v_id          viewpoint id
     * @param length        path length
     * @return true         find the closest frontier
     * @return false        no frontier in current connected graph
     */
    bool GetClosestGlobalTarget(list<Eigen::Vector3d> &path, list<h_ptr> &H_path, int &f_id, int &v_id, double &length);

    /**
     * @brief Get the Global Targets
     * 
     * @param s_hn start hn
     * @param paths <length, path>
     * @param t_hn target hns
     */
    void GetGlobalTarget(h_ptr &s_hn, vector<pair<double, list<Eigen::Vector3d>>> &paths, vector<h_ptr> &t_hn/*, vector<list<h_ptr>> &debug_paths*/);

    /**
     * @brief Get Global Targets in local partition 
     * 
     * @param s_hn start hn
     * @param paths <length, path>
     * @param t_hn local explorable target hns
     */
    void GlobalLocalSearch(h_ptr &s_hn, list<pair<double, list<Eigen::Vector3d>>> &paths, list<h_ptr> &t_hn);

    /**
     * @brief Get all Global Targets
     * 
     * @param s_hn start hn
     * @param paths <length, path>
     * @param t_hn all explorable target hns
     */
    void FullSearch(h_ptr &s_hn, list<pair<double, list<Eigen::Vector3d>>> &paths, list<h_ptr> &t_hn);

    /**
     * @brief Get all the local frontiers and their viewpoints
     * 
     * @param f_v_l 
     */
    void GetAllLocalFroVps(list<int> &f_l);

    void PathSearch(f_ptr &start, f_ptr &end, list<h_ptr> &H_path,
         list<Eigen::Vector3d> &path, double &length);
    void PathSearch(h_ptr &start, f_ptr &end, list<h_ptr> &H_path,
         list<Eigen::Vector3d> &path, double &length);
    void PathSearch(h_ptr &start, h_ptr &end, list<h_ptr> &H_path,
         list<Eigen::Vector3d> &path, double &length);
    void PathSearch(f_ptr &start, h_ptr &end, list<h_ptr> &H_path,
         list<Eigen::Vector3d> &path, double &length);

    
    void ClearSearched(list<h_ptr> &h_l);
    void ClearSearched(list<f_ptr> &f_l);

    /**
     * @brief retrieve DTG path from a hnode to a fnode
     * 
     * @param tar_f   target fnode
     * @param h_path  h node path
     * @param path    pos path
     * @param length  path length
     */
    void RetrieveHFPath(f_ptr &tar_f, list<h_ptr> &h_path, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief retrieve DTG path from a hnode to a hnode
     * 
     * @param start_h   start hnode
     * @param tar_f   target hnode
     * @param path    pos path
     * @param length  path length
     */
    void RetrieveHPath(h_ptr &start_h, h_ptr &tar_h, list<Eigen::Vector3d> &path, double &length);

    void RetrieveHDebug(h_ptr &start_h, h_ptr &tar_h, list<h_ptr> &h_path);

    inline bool FindFnode(const uint32_t &id, f_ptr &fp);
    inline bool FindHnode(const uint32_t &id, h_ptr &hp);
    inline bool FindHnode(const Eigen::Vector3d &pos, const uint32_t &id, h_ptr &hp);
    inline bool FindHnode(const uint16_t &idx, const uint32_t &id, h_ptr &hp);
    // inline bool SetConnectUAV(const uint32_t &h_id, const Eigen::Vector3d &h_pos, const uint8_t &UAV_id);
    // inline bool EraseConnectUAV(const uint32_t &h_id, const Eigen::Vector3d &h_pos, const uint8_t &UAV_id);
    // inline bool FindFnode(const Eigen::Vector3d &center, const uint32_t &id, f_ptr &fp);
    list<pair<uint16_t, uint32_t>> *GVP_hn_;  // self global partition
    list<pair<uint16_t, uint32_t>> *local_hn_;  // self local partition
    list<uint16_t> *local_fn_;   // self exploring fn
    SwarmDataManager *SDM_;

private:
    // void ClearLocalRootPath();
    friend class GraphVoronoiPartition;

    inline bool InsideMap(const Eigen::Vector3i &idx3);
    inline bool InsideMap(const Eigen::Vector3d &pos);
    inline bool GetVox(const Eigen::Vector3i &idx3, list<h_ptr> &h_l, list<f_ptr> &f_l);
    inline bool GetVox(const Eigen::Vector3d &pos, list<h_ptr> &h_l, list<f_ptr> &f_l);
    inline bool GetVox(const Eigen::Vector3d &pos, const uint32_t &idx, h_ptr &h_l);
    inline bool GetVoxBBX(const Eigen::Vector3d &pos, const Eigen::Vector3d &BBX, const uint32_t &idx, h_ptr &h_l);

    inline Eigen::Vector3i GetVoxId3(const Eigen::Vector3d &pos);
    inline int GetVoxId(const Eigen::Vector3i &idx3);
    inline int GetVoxId(const Eigen::Vector3d &pos);

    inline void BreakHFEdge(h_ptr &h, f_ptr &f);

    inline bool GetEdge(h_ptr &h, f_ptr &f, hfe_ptr &e);
    inline bool GetEdge(f_ptr &f1, f_ptr &f2, ffe_ptr &e);
    inline bool GetEdge(h_ptr &h1, h_ptr &h2, hhe_ptr &e);

    inline h_ptr CreateSwarmHnode(const Eigen::Vector3d &pos, const uint32_t &id);
    inline h_ptr CreateHnode(const Eigen::Vector3d &pos);

    // inline f_ptr CreateSwarmFnode(const Eigen::Vector3d &pos, const uint32_t &id);
    // inline f_ptr CreateFnode(const uint32_t &id, const Eigen::Vector3d &pos, const Eigen::Vector3d &scale);
    // inline f_ptr CreateFnode(const uint32_t &id, const Eigen::Vector3d &pos, const Eigen::Vector3d &scale);
    /**
     * @brief block edge, set edge infeasible, length = 2e5
     * 
     * @param e 
     */
    inline void BlockEdge(hhe_ptr &e);

    /**
     * @brief block edge, set edge infeasible, length = 2e5
     * 
     * @param e 
     */
    inline void BlockEdge(ffe_ptr &e);

    /**
     * @brief block edge, set edge infeasible, length = 2e5, fn->p_id = -1
     * 
     * @param e 
     */
    inline void BlockEdge(hfe_ptr &e);

    inline void EraseEdge(hhe_ptr &e, const bool &broadcast = true);
    inline void EraseEdge(ffe_ptr &e, const bool &broadcast = true);
    inline void EraseEdge(hfe_ptr &e, const bool &broadcast = true);
    /**
     * @brief create edge and connect hnode and fnode
     * 
     * @param h         head
     * @param f         tail
     * @param path      from head to tail
     * @param length    path length
     */
    inline void ConnectHF(h_ptr &h, f_ptr &f, const int &v_id,  list<Eigen::Vector3d> &path, double length);

    /**
     * @brief create edge and connect two Fnodes
     * 
     * @param f1         head fnode
     * @param f2         tail fnode
     * @param path       from head to tail
     * @param length     path length
     */
    inline void ConnectFF(f_ptr &f1, f_ptr &f2, list<Eigen::Vector3d> &path, double length);
    
    /**
     * @brief create edge and connect two Hnodes
     * 
     * @param head      head hnode
     * @param tail      tail hnode
     * @param path      from head to tail
     * @param length    path length
     */
    inline void ConnectHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path, double length);

    /**
     * @brief create edge and connect two Hnodes in swarm
     * 
     * @param head      head hnode
     * @param tail      tail hnode
     * @param path      from head to tail
     */
    inline bool ConnectSwarmHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path);

    /**
     * @brief create edge and connect Hnode and Fnode in swarm
     * 
     * @param head      head hnode
     * @param tail      fead hnode
     * @param path      from head to tail
     * @return true     shortest path
     * @return false    shorter path exists
     */
    inline bool ConnectSwarmHF(h_ptr &head, f_ptr &tail, list<Eigen::Vector3d> &path, uint8_t &vp_id);

    /**
     * @brief check if h and f in the node
     * 
     * @param pos position of the node
     * @param h    
     * @param f 
     * @return true 
     * @return false 
     */
    inline bool CheckNode(const Eigen::Vector3d &pos, const h_ptr &h, const f_ptr &f);

    inline int LoadHHEdgeMSG(swarm_exp_msgs::DtgHHEdge &e); //0 success, 1 no node, 2 not the longest 
    inline int LoadHFEdgeMSG(swarm_exp_msgs::DtgHFEdge &e); //0 success, 1 no node, 2 not the longest
    // inline int LoadFFEdgeMSG(swarm_exp_msgs::DtgFFEdge &e); //0 success, 1 no node, 2 not the longest
    inline h_ptr LoadHNodeMSG(swarm_exp_msgs::DtgHNode &h); //0 success, 1 fail

    void DTGCommunicationCallback(const ros::TimerEvent &e);

    void ShowAll(const ros::TimerEvent &e); 
    void Show();
    void Debug();
    void Debug(list<Eigen::Vector3d>  &fl);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher topo_pub_, debug_pub_;
    uint32_t cur_hid_;
    double H_thresh_, sensor_range_, eurange_;
    int uav_id_;
    bool update_FF_, show_e_details_;

    list<f_ptr> local_f_list_;
    list<double> local_f_dist_list_;
    list<h_ptr> local_h_list_;
    list<double> local_h_dist_list_;

    vector<list<h_ptr>> H_depot_;
    vector<f_ptr> F_depot_;
    list<h_ptr> H_list_;
    list<Eigen::Vector3d> debug_pts_;
    h_ptr root_; 
    list<shared_ptr<ffe>> r_f_edges_;//robot -> frontier
    Eigen::Vector3d origin_, vox_scl_, map_upbd_;
    Eigen::Vector3i vox_num_;

    prio_A open_A_;
    prio_D open_D_;

    ros::Timer show_timer_;

    FrontierGrid *FG_;
    lowres::LowResMap *LRM_;
    BlockMap *BM_;
    ColorManager *CM_;

    //swarm
    ros::Timer swarm_timer_;
    bool use_swarm_;
    int drone_num_;
    
};

inline void MultiDTG::EraseFnode(const Eigen::Vector3d &center, const uint32_t &id){
    // int idx = GetVoxId(center);
    // for(list<f_ptr>::iterator f = F_depot_[idx].begin(); f != F_depot_[idx].end(); f++){
        if(id < F_depot_.size()){
                // while(!F_depot_[id]->edges_.empty()){
                //     EraseEdge(F_depot_[id]->edges_.front());
                //     F_depot_[id]->edges_.pop_front();
                // }
            EraseEdge(F_depot_[id]->hf_edge_);
            
            // if((*f)->id_ == id){
            //     while(!(*f)->edges_.empty()){
            //         EraseEdge((*f)->edges_.front());
            //         (*f)->edges_.pop_front();
            //     }
            //     EraseEdge((*f)->hf_edge_);
            //     F_depot_[idx].erase(f);
            //     return;
            // }
        }
    // }
}

inline bool MultiDTG::InsideMap(const Eigen::Vector3i &idx3){
    if(idx3(0) < 0 || idx3(1) < 0 || idx3(2) < 0 ||
        idx3(0) >  vox_num_(0) - 1 || idx3(1) > vox_num_(1) - 1 || idx3(2) > vox_num_(2) - 1 )
        return false;
    return true;
}

inline bool MultiDTG::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < origin_(0)|| pos(1) < origin_(1)|| pos(2) < origin_(2)||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool MultiDTG::GetVox(const Eigen::Vector3i &idx3, list<h_ptr> &h_l, list<f_ptr> &f_l){
    int idx = GetVoxId(idx3);
    if(idx != -1){
        h_l = H_depot_[idx];
        f_l.clear();
        f_l.emplace_back(F_depot_[idx]);
        return true;
    }
    return false;
}

inline bool MultiDTG::GetVox(const Eigen::Vector3d &pos, list<h_ptr> &h_l, list<f_ptr> &f_l){
    int idx = GetVoxId(pos);
    if(idx != -1){
        h_l = H_depot_[idx];
        f_l.clear();
        f_l.emplace_back(F_depot_[idx]);
        return true;
    }
    return false;
}

inline bool MultiDTG::GetVox(const Eigen::Vector3d &pos, const uint32_t &idx, h_ptr &h_l){
    int vox_idx = GetVoxId(pos);
    if(vox_idx != -1){
        for(auto &h : H_depot_[vox_idx]){
            if(h->id_ == idx){
                h_l = h;
                return true;
            }
        }
    }
    return false;
}

inline bool MultiDTG::GetVoxBBX(const Eigen::Vector3d &pos, const Eigen::Vector3d &BBX, const uint32_t &idx, h_ptr &h_l){
    Eigen::Vector3d up, low;
    Eigen::Vector3i up_idx, low_idx, it_idx;
    up = pos + BBX / 2;
    low = pos - BBX / 2;
    for(int dim = 0; dim < 3; dim++){
        up_idx(dim) = floor((up(dim) - origin_(dim)) / vox_scl_(dim));
        low_idx(dim) = floor((low(dim) - origin_(dim)) / vox_scl_(dim));
    }
    // cout<<"BBX:"<<BBX.transpose()<<"  pos:"<<pos.transpose()<<endl;
    // cout<<"low_bd:"<<low_idx.transpose()<<"  up_idx:"<<up_idx.transpose()<<endl;
    for(it_idx(0) = low_idx(0); it_idx(0) <= up_idx(0); it_idx(0)++)
        for(it_idx(1) = low_idx(1); it_idx(1) <= up_idx(1); it_idx(1)++)
            for(it_idx(2) = low_idx(2); it_idx(2) <= up_idx(2); it_idx(2)++){
        int vox_idx = GetVoxId(it_idx);
        // cout<<"vox id:"<<vox_idx<<endl;
        if(vox_idx != -1){
            for(auto &h : H_depot_[vox_idx]){
                // cout<<"hid:"<<int(h->id_)<<" id:"<<int(idx)<<endl;
                if(h->id_ == idx){
                    h_l = h;
                    return true;
                }
            }
        }
    }

    return false;
}

inline Eigen::Vector3i MultiDTG::GetVoxId3(const Eigen::Vector3d &pos){
    return Eigen::Vector3i((int)floor((pos(0) - origin_(0))/vox_scl_(0)), (int)floor((pos(1) - origin_(1))/vox_scl_(1)),
         (int)floor((pos(2) - origin_(2))/vox_scl_(2))); 
}

inline int MultiDTG::GetVoxId(const Eigen::Vector3i &idx3){
    if(InsideMap(idx3)){
        return idx3(0) + idx3(1) * vox_num_(0) + idx3(2) * vox_num_(0) * vox_num_(1); 
    }
    else return -1;
}

inline int MultiDTG::GetVoxId(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        int x = floor((pos(0) - origin_(0)) / vox_scl_(0));
        int y = floor((pos(1) - origin_(1)) / vox_scl_(1));
        int z = floor((pos(2) - origin_(2)) / vox_scl_(2));
        return x + y * vox_num_(0) + z * vox_num_(0) * vox_num_(1); 
    }
    else return -1;
}

inline void MultiDTG::BreakHFEdge(h_ptr &h, f_ptr &f){
    for(list<shared_ptr<DTG_edge<H_node, F_node>>>::iterator e = h->hf_edges_.begin(); e != h->hf_edges_.end(); e++){
        if((*e)->tail_n_ == f){
            // ROS_WARN("id:%d erase0 h:%d f:%d", SDM_->self_id_, h->id_, f->id_);
            h->hf_edges_.erase(e);
            return;
        }
    }
}

inline bool MultiDTG::GetEdge(h_ptr &h, f_ptr &f, hfe_ptr &e){
    if(f->hf_edge_ != NULL && h == f->hf_edge_->head_n_){
        e = f->hf_edge_;
        return true;
    }
    return false;
}

inline bool MultiDTG::GetEdge(f_ptr &f1, f_ptr &f2, ffe_ptr &e){
    for(auto &ei : f1->edges_){
        if(ei->tail_n_ == f2 || ei->head_n_ == f2){
            e = ei;
            return true;
        }
    }
    return false;
}

inline bool MultiDTG::GetEdge(h_ptr &h1, h_ptr &h2, hhe_ptr &e){
    for(auto &ei : h1->hh_edges_){
        if(ei->tail_n_ == h2 || ei->head_n_ == h2){
            e = ei;
            return true;
        }
    }
    return false;
}

inline bool MultiDTG::FindFnode(const uint32_t &id, f_ptr &fp){
    if(id >= F_depot_.size() || id < 0) return false;
    fp = F_depot_[id];
    return true;
}

inline bool MultiDTG::FindHnode(const uint32_t &id, h_ptr &hp){
    //BFS
    h_ptr c_h, n_h;
    list<h_ptr> h_l, h_out_l;
    c_h = root_;
    bool find_h = false;
    if(root_ != NULL){
        if(c_h->id_ == id){
            hp = c_h;
            return true;
        }
        c_h->h_flags_ |= 1;
        h_l.emplace_back(c_h);
        while (!h_l.empty())
        {
            c_h = h_l.front();
            h_l.pop_front();
            h_out_l.emplace_back(c_h);
            for(list<hhe_ptr>::iterator e_it = c_h->hh_edges_.begin(); e_it != c_h->hh_edges_.end(); e_it++){
                if((*e_it)->head_n_ == c_h) n_h = (*e_it)->tail_n_;
                else n_h = (*e_it)->head_n_;

                if(n_h->h_flags_ & 1) continue;
                if(n_h->id_ == id){
                    hp = n_h;
                    find_h = true;
                }
                else{
                    n_h->h_flags_ |= 1;
                    h_l.emplace_back(n_h);
                }
            }
            if(find_h){
                for(auto &h : h_l) h->h_flags_ &= 254;
                for(auto &h : h_out_l) h->h_flags_ &= 254;
                return true;
            }
        }
    }

    //force search
    for(auto &h : H_list_){
        if(h->id_ == id){
            hp = h;
            for(auto &h : h_l) h->h_flags_ &= 254;
            for(auto &h : h_out_l) h->h_flags_ &= 254;
            return true;
        }
    }
    for(auto &h : h_l) h->h_flags_ &= 254;
    for(auto &h : h_out_l) h->h_flags_ &= 254;
    return false;
}

inline bool MultiDTG::FindHnode(const Eigen::Vector3d &pos, const uint32_t &id, h_ptr &hp){
    int idx = GetVoxId(pos);
    if(idx == -1) return false;
    for(auto &h_l : H_depot_){
        for(auto &h : h_l){
            if(h->id_ == id){
                hp = h;
                return true;
            }
        }
    }
    return false;
}

inline bool MultiDTG::FindHnode(const uint16_t &idx, const uint32_t &id, h_ptr &hp){
    if(idx < 0 || idx >= H_depot_.size()) return false;
        for(auto &h_l : H_depot_){
        for(auto &h : h_l){
            if(h->id_ == id){
                hp = h;
                return true;
            }
        }
    }
    return false;
}


// inline bool MultiDTG::SetConnectUAV(const uint32_t &h_id, const Eigen::Vector3d &h_pos, const uint8_t &UAV_id){
//     h_ptr hn;
//     if(FindHnode(h_pos, h_id, hn)){
//         bool new_connect = false;
//         for(auto &u_d : hn->uav_dist_){
//             if(u_d.first == UAV_id){
//                 new_connect = true;
//                 break;
//             }
//         }
//         if(new_connect){
//             u_d.second = 
//         }

//     }
//     return false;
// }

// inline bool MultiDTG::EraseConnectUAV(const uint32_t &h_id, const Eigen::Vector3d &h_pos, const uint8_t &UAV_id){

// }

// inline bool MultiDTG::FindFnode(const Eigen::Vector3d &center, const uint32_t &id, f_ptr &fp){
//     int idx = GetVoxId(center);
//     if(idx == -1) return false;
//     else{
//         for(auto &f : F_depot_[idx]){
//             if(f->id_ == id){
//                 fp = f;
//                 return true;
//             }
//         }
//         return false;
//     }
// }

inline h_ptr MultiDTG::CreateSwarmHnode(const Eigen::Vector3d &pos, const uint32_t &id){
    h_ptr hnode = make_shared<H_node>();
    hnode->pos_ = LRM_->GetCenterPos(pos);
    std::cout<<"id:"<<int(SDM_->self_id_)<< "\033[0;42m try create swarm h:"<<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<<" \033[0m"<< std::endl;
    int idx = GetVoxId(hnode->pos_ );
    if(idx == -1) return NULL;
    hnode->state_ = BLOCKED;
    hnode->id_ = id;
    std::cout << "\033[0;42m new id:"<<int(hnode->id_)<<"  "<<idx<<" \033[0m" <<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<< std::endl;
    H_depot_[idx].emplace_back(hnode);
    H_list_.emplace_back(hnode);
    return hnode;
}

inline h_ptr MultiDTG::CreateHnode(const Eigen::Vector3d &pos){
    h_ptr hnode = make_shared<H_node>();
    hnode->pos_ = LRM_->GetCenterPos(pos);
    std::cout<<"id:"<<int(SDM_->self_id_) << "\033[0;42m try create:"<<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<<" \033[0m"<< std::endl;
    int idx = GetVoxId(hnode->pos_ );
    if(idx == -1) return NULL;

    hnode->state_ = L_FREE;
    hnode->id_ = cur_hid_;
    cur_hid_ += drone_num_;
    std::cout << "\033[0;42m new id:"<<int(hnode->id_)<<"  "<<idx<<" \033[0m" <<hnode->pos_.transpose()<<"  ;"<<pos.transpose()<< std::endl;
    H_depot_[idx].emplace_back(hnode);
    H_list_.emplace_back(hnode);
    SDM_->SetDTGHn(hnode->id_, uint32_t(LRM_->PostoId(pos)));
    hnode->h_flags_ |= 6;
    local_hn_->push_back({idx, hnode->id_});
    GVP_hn_->push_back({idx, hnode->id_});
    return hnode;
}

// inline f_ptr MultiDTG::CreateSwarmFnode(const Eigen::Vector3d &pos, const uint32_t &id){
//     if(id >= FG_->f_grid_.size())return NULL;
//     // f_ptr f = make_shared<F_node>();
//     // f->center_ = FG_->f_grid_[id].center_;
//     // cout<<"get swarmf:"<<int(id)<<"      "<<f->center_.transpose()<<endl;
//     // f->id_ = id;
//     // int idx = GetVoxId(f->center_);
    
//     // F_depot_[idx].emplace_back(f);
//     return F_depot_[id];
// }

// inline f_ptr MultiDTG::CreateFnode(const uint32_t &id, const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd){
//     f_ptr f = make_shared<F_node>();
//     // f->pos_ = pos;
//     f->center_ = (upbd + lowbd) / 2;
//     f->id_ = id;
//     // f->upbd_ = upbd;
//     // f->lowbd_ = lowbd;
//     int idx = GetVoxId(f->center_);
    
//     F_depot_[idx].emplace_back(f);
//     return f;
// }

inline void MultiDTG::BlockEdge(hhe_ptr &e){
    if(e == NULL) return;
    e->length_ = 2e5;
    e->e_flag_ &= 239;
}

inline void MultiDTG::BlockEdge(ffe_ptr &e){
    if(e == NULL) return;
    e->length_ = 2e5;
    e->e_flag_ &= 239;
}

inline void MultiDTG::BlockEdge(hfe_ptr &e){
    if(e == NULL) return;
    e->length_ = 2e5;
    e->e_flag_ &= 239;
    f_ptr f;
    f = e->tail_n_;
    f->vp_id_ = -1;
    // ROS_WARN("id:%d vp = -1 !%d", SDM_->self_id_, f->id_);
}

inline void MultiDTG::EraseEdge(hhe_ptr &e, const bool &broadcast){
    if(e == NULL) return;
    list<hhe_ptr>::iterator e_it;
    h_ptr h;
    h = e->head_n_;
    for(e_it = h->hh_edges_.begin(); e_it != h->hh_edges_.end(); e_it++){
        if((*e_it) == e) {
            h->hh_edges_.erase(e_it);
            // if(use_swarm_ && broadcast) SDM_->EraseDTGHHEdge(e->head_, e->tail_);
            break;
        }
    }
    h = e->tail_n_;
    for(e_it = h->hh_edges_.begin(); e_it != h->hh_edges_.end(); e_it++){
        if((*e_it) == e) {
            // if(use_swarm_ && broadcast) SDM_->EraseDTGHHEdge(e->head_, e->tail_);
            h->hh_edges_.erase(e_it);
            break;
        }
    }
}

inline void MultiDTG::EraseEdge(ffe_ptr &e, const bool &broadcast){
    if(e == NULL) return;
    list<ffe_ptr>::iterator e_it;
    f_ptr f;
    f = e->head_n_;
    for(e_it = f->edges_.begin(); e_it != f->edges_.end(); e_it++){
        if((*e_it) == e) {
            f->edges_.erase(e_it);
            break;
        }
    }
    f = e->tail_n_;
    for(e_it = f->edges_.begin(); e_it != f->edges_.end(); e_it++){
        if((*e_it) == e) {
            f->edges_.erase(e_it);
            break;
        }
    }
}

inline void MultiDTG::EraseEdge(hfe_ptr &e, const bool &broadcast){
    if(e == NULL) return;
    list<hfe_ptr>::iterator e_it;
    h_ptr h;
    f_ptr f;
    h = e->head_n_;
    f = e->tail_n_;
    for(e_it = h->hf_edges_.begin(); e_it != h->hf_edges_.end(); e_it++){
        if((*e_it) == e) {
            // ROS_WARN("id:%d erase1 h:%d f:%d", SDM_->self_id_, h->id_, f->id_);
            h->hf_edges_.erase(e_it);
            break;
        }
    }

    // if(use_swarm_ && broadcast) SDM_->EraseDTGHFEdge(f->id_);

    f->vp_id_ = -1;
    f->hf_edge_ = NULL;
}

inline void MultiDTG::ConnectHF(h_ptr &h, f_ptr &f, const int &v_id, list<Eigen::Vector3d> &path, double length){
    hfe_ptr e_ptr; 

    if(!GetEdge(h, f, e_ptr)) {
        if(f->hf_edge_ != NULL && (f->hf_edge_->e_flag_ & 16) && length + 1e-3 > f->hf_edge_->length_) return;
        e_ptr = make_shared<hfe>();
        if(f->hf_edge_ != NULL && f->hf_edge_->head_n_ != NULL){
            h_ptr h_old = f->hf_edge_->head_n_;
            for(list<hfe_ptr>::iterator e_it = h_old->hf_edges_.begin(); e_it != h_old->hf_edges_.end(); e_it++){
                if((*e_it)->tail_n_ == f){
                    // ROS_WARN("id:%d erase2 h:%d f:%d", SDM_->self_id_, h->id_, f->id_);
                    h_old->hf_edges_.erase(e_it);
                    break;
                }
            }
        }
        // ROS_WARN("id:%d NEW h:%d f:%d", SDM_->self_id_, h->id_, f->id_);
        h->hf_edges_.emplace_back(e_ptr);
    }
    else{
        if((e_ptr->e_flag_ & 16) && length + 1e-3 > e_ptr->length_) return;
    }
    e_ptr->head_ = h->id_;
    e_ptr->tail_ = f->id_;
    e_ptr->head_n_ = h;
    e_ptr->tail_n_ = f;
    e_ptr->length_ = length;
    e_ptr->path_ = path;
    f->vp_id_ = v_id;
    if(f->vp_id_ == -1){
        ROS_ERROR("error vp_id");
        ros::shutdown();
        return;
    }
    // Eigen::Vector3d debug_p;
    // if(FG_->GetVpPos(f->id_, v_id, debug_p)){
    //     if((debug_p - path.back()).norm() > 1e-3) ROS_ERROR("error connect");
    //     cout<<debug_p.transpose()<<endl;
    //     cout<<path.back().transpose()<<endl;
    // }
    // f->pos_ = path.back();
    f->hf_edge_ = e_ptr;
    /* add to local partition */
    if(f->cf_->flags_[3]){
        uint8_t temp_owner;
        FG_->ChangeOwner(f->id_, SDM_->self_id_, length, temp_owner);
        f->cf_->flags_.reset(3);
        f->f_flag_ |= 2;
        local_fn_->emplace_back(f->id_);
    }
    if(!(h->h_flags_ & 2)){
        h->h_flags_ |= 2;
        local_hn_->push_back({GetVoxId(h->pos_), h->id_});
    }

    //     e_ptr->length_s_ = e_ptr->length_;
    //     e_ptr->tail_n_s_ = e_ptr->tail_n_;
    //     e_ptr->head_n_s_ = e_ptr->head_n_;
    //     e_ptr->path_s_ = e_ptr->path_;
    //     e_ptr->head_s_ = e_ptr->head_;
    //     e_ptr->tail_s_ = e_ptr->tail_;
    if(use_swarm_){
        vector<uint32_t> path;
        int p_idx;
        for(auto &p : e_ptr->path_){
            p_idx = LRM_->PostoId(p);
            if(path.empty() || p_idx != path.back())
                path.push_back(p_idx);
        }
        SDM_->SetDTGHFEdge(e_ptr->head_, e_ptr->tail_, v_id, path);
        // cout<<"id:"<<int(SDM_->self_id_)<<" send h:"<<int(h->id_)<<"  f:"<<int(f->id_)<<" length:"<<length<<" flag:"<<int(e_ptr->e_flag_)<<endl;
    }
    e_ptr->e_flag_ |= 16;
    
    // }
}

inline void MultiDTG::ConnectFF(f_ptr &f1, f_ptr &f2, list<Eigen::Vector3d> &path, double length){
    ffe_ptr e_ptr; 
    bool push_edge = false;
    if(!GetEdge(f1, f2, e_ptr)) {
        push_edge = true;
        e_ptr = make_shared<ffe>(); 
    }
    if(length >= e_ptr->length_) return;

    e_ptr->head_ = f1->id_;
    e_ptr->tail_ = f2->id_;
    e_ptr->head_n_ = f1;
    e_ptr->tail_n_ = f2;
    e_ptr->length_ = length;
    e_ptr->path_ = path;
    // e_ptr->e_flag_ |= 4;
    e_ptr->e_flag_ |= 16;
    if(push_edge){
        f1->edges_.emplace_back(e_ptr);
        f2->edges_.emplace_back(e_ptr);
    }
}

inline void MultiDTG::ConnectHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path, double length){
    hhe_ptr e_ptr; 
    // cout<<"ConnectHH:"<<endl;
    bool push_edge = false;
    if(!GetEdge(head, tail, e_ptr)){
        e_ptr = make_shared<hhe>(); 
        push_edge = true;
    }
    if(length >= e_ptr->length_) return;

    e_ptr->head_ = head->id_;
    e_ptr->tail_ = tail->id_;
    e_ptr->head_n_ = head;
    e_ptr->tail_n_ = tail;
    e_ptr->length_ = length;
    e_ptr->path_ = path;

    // e_ptr->e_flag_ |= 4;
    e_ptr->e_flag_ |= 16;

    // cout<<"SET H:"<<head->id_<<" T:"<<tail->id_<<endl;
    // for(auto &p : path){
    //     cout<<p.transpose()<<endl;
    // }
    if(push_edge){
        head->hh_edges_.emplace_back(e_ptr);
        tail->hh_edges_.emplace_back(e_ptr);
    }

    if(e_ptr->length_ < e_ptr->length_s_){
        e_ptr->e_flag_ |= 32;
        e_ptr->length_s_ = e_ptr->length_;
    }

    // send to avoid error search
    if(use_swarm_){
        vector<uint32_t> path;
        int p_idx;
        for(auto &p : e_ptr->path_){
            p_idx = LRM_->PostoId(p);
            if(path.empty() || p_idx != path.back())
                path.push_back(p_idx);
        }
        SDM_->SetDTGHHEdge(e_ptr->head_, e_ptr->tail_, path);
    }
    // }
    // cout<<"End Connect:"<<endl;
}

inline bool MultiDTG::ConnectSwarmHH(h_ptr &head, h_ptr &tail, list<Eigen::Vector3d> &path){
    hhe_ptr edge;
    bool new_edge = true;
    for(auto &e : head->hh_edges_){
        if(e->tail_ == tail->id_ || e->head_ == tail->id_){
            new_edge = false;
            edge = e;
            break;
        }
    }

    if(new_edge){
        edge = make_shared<hhe>(head, tail);
    }

    double length = 0;
    list<Eigen::Vector3d>::iterator p1, p2;
    p1 = path.begin();
    p2 = path.begin();
    p2++;
    for(; p2 != path.end(); p2++, p1++){
        length += (*p1 - *p2).norm();
    }
    if(length + 1e-3 < edge->length_s_){
        if(!(edge->e_flag_ & 16)){
            edge->path_ = path;
            // edge->e_flag_ |= 4;
        }
        if(length + 1e-3 < edge->length_ && LRM_->PathCheck(path)){
            // edge->e_flag_ |= 4;
            edge->e_flag_ |= 16;
            edge->length_ = length;
        }
        // else{
        //     ROS_WARN("id:%d PathCheck1 h:%d h:%d", SDM_->self_id_, head->id_, tail->id_);
        // }
        edge->e_flag_ |= 32;
        edge->length_s_ = length;
        if(new_edge){
            head->hh_edges_.emplace_back(edge);
            tail->hh_edges_.emplace_back(edge);
        }
        return true;
    }
    else return false;
}

inline bool MultiDTG::ConnectSwarmHF(h_ptr &head, f_ptr &tail, list<Eigen::Vector3d> &path, uint8_t &vp_id){
    bool new_e;
    hfe_ptr edge;
    if(tail->cf_->f_state_ == 2) return false;
    if(vp_id < 0 || vp_id >= tail->cf_->local_vps_.size() || tail->cf_->local_vps_[vp_id] == 2) return false;
    if(tail->hf_edge_ != NULL && tail->hf_edge_->head_ == head->id_){
        new_e = false;
        edge = tail->hf_edge_;
    }
    else new_e = true;

    if(new_e){
        if(tail->hf_edge_ == NULL)
            tail->hf_edge_ = make_shared<hfe>(head, tail);
        else{
            for(auto e_it = tail->hf_edge_->head_n_->hf_edges_.begin(); e_it != tail->hf_edge_->head_n_->hf_edges_.end(); e_it++){
                if((*e_it) == tail->hf_edge_) {
                    // ROS_WARN("id:%d erase3 h:%d f:%d", SDM_->self_id_, head->id_, tail->id_);
                    tail->hf_edge_->head_n_->hf_edges_.erase(e_it);
                    break;
                }
            }
            tail->hf_edge_->head_ = head->id_;
            tail->hf_edge_->head_n_ = head;
        }
        edge = tail->hf_edge_;
    }

    double length = 0;
    list<Eigen::Vector3d>::iterator p1, p2;
    Eigen::Vector3d vp_pos;
    FG_->GetVpPos(tail->id_, vp_id, vp_pos);
    path.emplace_back(vp_pos);
    p1 = path.begin();
    p2 = path.begin();
    p2++;
    for(; p2 != path.end(); p2++, p1++){
        length += (*p1 - *p2).norm();
    }

    tail->cf_->flags_.reset(3);
    
    if((tail->f_flag_ & 2) && !(head->h_flags_ & 2)){
        head->h_flags_ |= 2;
        local_hn_->push_back({GetVoxId(head->pos_), head->id_});
    }

    if(!new_e && head->id_ != edge->head_){
        for(list<hfe_ptr>::iterator e_it = edge->head_n_->hf_edges_.begin(); e_it != edge->head_n_->hf_edges_.end(); e_it++){
            if((*e_it)->tail_ == tail->id_){
                // ROS_WARN("id:%d erase4 h:%d f:%d", SDM_->self_id_, head->id_, tail->id_);
                edge->head_n_->hf_edges_.erase(e_it);
                break;
            }
        }
        // ROS_WARN("id:%d push1 h:%d f:%d", SDM_->self_id_, head->id_, tail->id_);
        head->hf_edges_.emplace_back(edge);
    }
    else if(new_e){
        // ROS_WARN("id:%d push1 h:%d f:%d", SDM_->self_id_, head->id_, tail->id_);
        head->hf_edges_.emplace_back(edge);
    }
    // edge->length_ = length;
    // if(!(edge->e_flag_ & 16)){
    //     tail->vp_id_ = vp_id;
    //     edge->path_ = path;
    // }

    if((!(edge->e_flag_ & 16) || length + 1e-3 < edge->length_) && LRM_->PathCheck(path, true)){
        edge->e_flag_ |= 16;
        edge->length_ = length;
        edge->path_ = path;
        tail->vp_id_ = vp_id;
        // cout<<"id:"<<int(SDM_->self_id_)<<"  h:"<<int(head->id_)<<"  f:"<<int(tail->id_)<<" h:"<<head->pos_.transpose()<<" f:"<<endl;
        if(tail->vp_id_ == -1){
            ROS_ERROR("error vp_id2");
            ros::shutdown();
            return false;
        }
    }
    else{
        // ROS_WARN("id:%d FAIL SWARM h:%d f:%d", SDM_->self_id_, head->id_, tail->id_);
        // if(!(edge->e_flag_ & 16) || length + 1e-3 < edge->length_){
        //     Eigen::Vector3d vpos;
        //     FG_->GetVpPos(tail->id_, vp_id, vpos);
        //     // cout<<"id:"<<int(SDM_->self_id_)<<"  h:"<<int(head->id_)<<"  f:"<<int(tail->id_)<<" h:"<<head->pos_.transpose()<<" f:"<<vpos.transpose()<<endl;
        //     if(path.size() == 1) cout<<path.back().transpose()<<endl;
        // }
        return false;
    }

    return true;
}

inline bool MultiDTG::CheckNode(const Eigen::Vector3d &pos, const h_ptr &h, const f_ptr &f){
    bool have_h = false;
    bool have_f = false;
    list<h_ptr> h_list;
    list<f_ptr> f_list;
    GetVox(pos, h_list, f_list);
    if(h != NULL){
        for(auto &h_it: h_list){
            if(h == h_it){
                have_h = true;
                break;
            }
        }
    }
    else have_h = true;

    if(f != NULL){
        for(auto &f_it: f_list){
            if(f == f_it){
                have_f = true;
                break;
            }
        }
    }
    else have_f = true;

    return (have_f && have_h);
}

// inline int MultiDTG::LoadHHEdgeMSG(swarm_exp_msgs::DtgHHEdge &e){
//     Eigen::Vector3d head_p, tail_p;
//     h_ptr h_s, t_s;
//     list<Eigen::Vector3d> path;
//     head_p = LRM_->IdtoPos(int(e.points_idx.front()));
//     tail_p = LRM_->IdtoPos(int(e.points_idx.back()));
//     if(!FindHnode(head_p, e.head_h_id, h_s) || !FindHnode(head_p, e.tail_h_id, t_s)){
//         ROS_ERROR("LoadHHEdgeMSG fail");
//         cout<<int(e.head_h_id)<<"success:"<<(h_s==NULL)<<FindHnode(e.head_h_id, h_s)<<endl;
//         cout<<int(e.tail_h_id)<<"success:"<<(t_s==NULL)<<FindHnode(e.tail_h_id, t_s)<<endl;
//         return 1;
//     }

//     for(auto &p : e.points_idx) path.emplace_back(LRM_->IdtoPos(int(p)));
//     if(ConnectSwarmHH(h_s, t_s, path)) return 0;
//     else return 2;
// }

// inline int MultiDTG::LoadHFEdgeMSG(swarm_exp_msgs::DtgHFEdge &e){
//     if(e.f_id >= F_depot_.size()) {
//         cout<<int(e.f_id)<<endl;
//         ROS_ERROR("LoadHFEdgeMSG fail1");
//         return 1;
//     }
//     list<Eigen::Vector3d> path;
//     f_ptr t_s = F_depot_[e.f_id];
//     h_ptr h_s;
//     Eigen::Vector3d head_p = LRM_->IdtoPos(int(e.points_idx.front()));
//     if(!FindHnode(head_p, e.h_id, h_s) ){
//         ROS_ERROR("LoadHFEdgeMSG fail2");
//         cout<<int(e.h_id)<<"success:"<<(h_s==NULL)<<FindHnode(e.h_id, h_s)<<endl;
//         return 1;
//     }
//     for(auto &p : e.points_idx) path.emplace_back(LRM_->IdtoPos(int(p)));
//     if(ConnectSwarmHF(h_s, t_s, path)) return 0;
//     else return 2;
// }

// inline int MultiDTG::LoadFFEdgeMSG(swarm_exp_msgs::DtgFFEdge &e){

// }

inline h_ptr MultiDTG::LoadHNodeMSG(swarm_exp_msgs::DtgHNode &h){
    h_ptr hnode;
    if(FindHnode(LRM_->IdtoPos(int(h.h_id)), h.h_id, hnode)){
        return hnode;
    }
    hnode = make_shared<H_node>();
    hnode->pos_ = LRM_->IdtoPos(int(h.h_id));
    std::cout << "\033[0;42m try swarm create:"<<hnode->pos_.transpose()<<"  ;"<<hnode->pos_ <<" \033[0m"<< std::endl;
    int idx = GetVoxId(hnode->pos_ );
    if(idx == -1) return NULL;

    hnode->state_ = L_FREE;
    hnode->id_ = h.h_id;
    std::cout << "\033[0;42m new swarmid:"<<int(hnode->id_)<<"  "<<idx<<" \033[0m" <<hnode->pos_.transpose()<< std::endl;
    H_depot_[idx].emplace_back(hnode);
    H_list_.emplace_back(hnode);
    return hnode;
}


}
#endif