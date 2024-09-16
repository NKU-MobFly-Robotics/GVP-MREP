#ifndef LOW_RESOLUTION_MAP_H_
#define LOW_RESOLUTION_MAP_H_
#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <block_map/block_map.h>
#include <block_map/raycast.h>
#include <block_map/color_manager.h>
#include <visualization_msgs/MarkerArray.h>
#include <tr1/unordered_map>
#include <queue>
#include <random>
#include <mutex>
#include <bitset>

#include <glog/logging.h>

using namespace std;

namespace lowres{
// For path searching
enum SchState{ in_close = 1, in_open = 2, not_expand = 3};

struct sch_node{
    sch_node(){
        parent_ = NULL;
        h_parent_= NULL;
        f_score_ = 0;
        g_score_ = 0;
        h_g_score_ = 0;
        status_ = not_expand;
        h_status_ = not_expand;
    }
    sch_node(shared_ptr<sch_node> &node){
        pos_ = node->pos_;
        parent_ = node->parent_;
        parent_ = node->h_parent_;
        status_ = node->status_;
        h_status_ = node->h_status_;
        f_score_ = node->f_score_;
        h_g_score_ = node->g_score_;
        g_score_ = node->h_g_score_;
    }
    Eigen::Vector3i pos_;
    shared_ptr<sch_node> parent_, h_parent_;
    SchState status_, h_status_;
    double f_score_;
    double g_score_, h_g_score_;
};

class ACompare {
public:
  bool operator()(shared_ptr<sch_node> node1, shared_ptr<sch_node> node2) {
    return node1->f_score_ > node2->f_score_;
  }
};

class DCompare {
public:
  bool operator()(shared_ptr<sch_node> node1, shared_ptr<sch_node> node2) {
    return node1->g_score_ > node2->g_score_;
  }
};

struct lr_root{
    uint32_t root_id_;
    u_char in_dir_; // (root)0(x+)(x-) (y+)(y-)(z+)(z-)
    u_char out_dir_; // 00(x+)(x-) (y+)(y-)(z+)(z-)
    float dist_;
};

//For data maintaining
struct LR_node{
    LR_node(){
        flags_.reset();
        flags_.set(2);
        topo_sch_ = NULL;
    }
    list<lr_root> ties_;        
    shared_ptr<sch_node> topo_sch_; //for graph searching, multi-threads are not allowed
    double last_update_;//for xnode
    bitset<8> flags_;  //0000 (Unode)(new)(local)(Xnode)
};

struct LR_block{
    LR_block(){
        alive_num_ = 0;
        local_grid_.clear();
        show_flag_ = true;
    };
    ~LR_block(){};
    vector<shared_ptr<LR_node>> local_grid_; 
    int alive_num_;
    Eigen::Vector3i origin_;    
    Eigen::Vector3i block_size_;          
    bool show_flag_;     
};

class LowResMap{
typedef tr1::unordered_map<int, pair<int, Eigen::Vector3i>> target_dict;
typedef priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, DCompare> prio_D;
typedef priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, ACompare> prio_A;
public:
    LowResMap(){};
    void init(const ros::NodeHandle &nh, 
            const ros::NodeHandle &nh_private);
    // void setOctoMap(volumetric_mapping::OctomapManager * OcMap){Octomap_ = OcMap;}
    void SetMap(BlockMap *map){map_ = map;};
    void SetTopoRange(const double &max_topo_range){max_g_cost_ = max_topo_range;}
    void SetColorManager(ColorManager *CM){CM_ = CM;};
    void SetEuRange(const double &eurange){eu_range_ = eurange;}
    void SetHthresh(const double &h_thresh){h_thresh_ = h_thresh;}
    inline Eigen::Vector3d GetStdPos(const Eigen::Vector3d &pos);
    inline bool InsideMap(const Eigen::VectorXd &pos);//
    inline bool InsideMap(const Eigen::Vector3i &pos);//
    inline bool InsideMap(const Eigen::Vector3d &pos);//

    /**
     * @brief Set the Eternal Node 
     * 
     * @param pos robot pos
     */
    void SetEternalNode(const Eigen::Vector3d &pos);

    /**
     * @brief Call before planning. Expand lowresmap, erase occupied nodes in a bounding box
     * 
     * @param rob_pose 
     * @param occ_list 
     */
    void UpdateLocalBBX(const Eigen::Matrix4d rob_pose, vector<Eigen::Vector3d> &occ_list);
    
    /**
     * @brief Call before planning. Expand lowresmap through Djkstra algorithm, erase occupied nodes in a bounding box
     * 
     * @param rob_pose 
     * @param occ_list 
     */
    bool UpdateLocalTopo(const Eigen::Matrix4d rob_pose, vector<Eigen::Vector3d> &occ_list, bool clear_x = false);

    /**
     * @brief Get the Path, A*
     * 
     * @param start 
     * @param end 
     * @param path      end-->start
     * @param local     search in local map
     * @return true     Path is found
     * @return false 
     */
    bool GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, vector<Eigen::Vector3d> &path, bool local = false, int search_num = 500);

    /**
     * @brief Get the Path, A*
     * 
     * @param start 
     * @param end 
     * @param path      start-->end
     * @param local     search in local map
     * @return true     Path is found
     * @return false 
     */
    bool GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &path, bool local = false, int search_num = 500);

    /**
     * @brief Force clear corresponding nodes
     * 
     * @param occ_list 
     */
    void ClearInfeasible(vector<Eigen::Vector3d> &occ_list);

    /**
     * @brief Force clear corresponding nodes, and break ties
     * 
     * @param occ_list 
     */
    void ClearInfeasibleTopo(vector<Eigen::Vector3d> &occ_list);

    /**
     * @brief call after ClearInfeasible()
     * 
     */
    void PruneBlock();  

    /**
     * @brief Clear expired Xnode. Clear dead blocks. Update topological relationships. 
     * 
     */
    void PruneTopoBlock();  

    /**
     * @brief check if the path is feasible
     * 
     * @param path 
     * @param allow_uknown 
     * @return true 
     * @return false 
     */
    bool PathCheck(list<Eigen::Vector3d> &path, bool allow_uknown = false);

    /**
     * @brief prune path and get rectangle safe corridors
     * 
     * @param path input raw path
     * @param corridors safecorridors
     * @param corridorVs vertexs of corridors
     * @param pruned_path shorten path
     * @param prune_length
     */
    bool FindCorridors(const vector<Eigen::Vector3d> path, 
                    vector<Eigen::MatrixX4d> &corridors, 
                    vector<Eigen::Matrix3Xd> &corridorVs,
                    vector<Eigen::Vector3d> &pruned_path,
                    double prune_length = INFINITY);

    // /**
    //  * @brief Get the Local Dist from current pos, only used after UpdateLocalTopo()
    //  * 
    //  * @param pos 
    //  * @return Manhattan dist, if pos is not in local area, return -1
    //  */
    // inline double GetLocalDist(const Eigen::Vector3d &pos);

    // /**
    //  * @brief Get the shortest local dist to the bbx, only used after UpdateLocalTopo()
    //  * 
    //  * @param upbd  upbound   
    //  * @param lowbd lowbound
    //  * @param c_idx connect lrnode index
    //  * @return Manhattan dist, if pos is not in local area, return -1
    //  */
    // inline double GetBBXLocalDist(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, int &c_idx);
    
    inline Eigen::Vector3d GetLowResolution(){return node_scale_;}

    inline Eigen::VectorXd GetLowResolutionXd(){return node_scale_;}
    inline bool IsFeasible(const Eigen::VectorXd &pos, bool allow_uknown = false);

    inline bool IsFeasible(const Eigen::Vector3d &pos, bool allow_uknown = false);

    inline bool IsFeasible(const Eigen::Vector3i &pos, bool allow_uknown = false); //dont check

    inline bool IsLocalFeasible(const Eigen::Vector3d &pos);
    
    /**
     * @brief get path 
     * 
     * @param start     the index of start pos 
     * @param target_id the id of the end hnode
     * @param path      start--->target_id
     * @param length    path length
     * @return true     path exist
     * @return false    path not exist
     */
    inline bool RetrieveHPath(const int &start, const uint32_t &target_id, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief get path 
     * 
     * @param start     the index of start pos 
     * @param target_id the id of the end hnode
     * @param path      start--->target_id
     * @param length    path length
     * @return true     path exist
     * @return false    path not exist
     */
    inline bool RetrieveHPath(const Eigen::Vector3d &start, const uint32_t &target_id, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief get the shortest(Manhattan) path 
     * 
     * @param start     start pos 
     * @param target_id the id of the end hnode
     * @param path      start--->target_id
     * @param length    path length
     * @return true     path exist
     * @return false    path not exist
     */
    inline bool RetrieveHPathShortest(const Eigen::Vector3d &start, uint32_t &target_id, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief get the shortest Manhattan distance to root node 
     * 
     * @param start     start pos
     * @param target_id hnode id
     * @param length    length
     * @return true     have path
     * @return false    no path
     */
    inline bool ShortestLength2Root(const Eigen::Vector3d &start, uint32_t &target_id, double &length);

    /**
     * @brief get path of current djkstra, the end is robot_pos
     * 
     * @param start     the index of start pos 
     * @param path      start--->robot_pos
     * @param length    path length
     */
    inline void RetrieveHPathTemp(const int &start, list<Eigen::Vector3d> &path, double &length);

    /**
     * @brief prune the path
     * 
     * @param path          raw path
     * @param pruned_path   shortten path
     * @param length        length of the pruned path
     * @return true         success
     * @return false        fail
     */
    bool PrunePath(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length);

    /**
     * @brief Check if the hnode is alive, if dead try to save it. For multi robots and obstacles.
     * 
     * @param pos position of the hnode
     * @param id  id of the hnode
     * @return true 
     * @return false 
     */
    inline bool CheckAddHnode(const Eigen::Vector3d &pos, const int &id);

    inline double GetRootHnCost(){return cur_h_cost_;};
    
    /**
     * @brief get the dist to p
     * 
     * @param pos p
     * @param paths paths
     * @param pruned_paths pruned_paths
     * @param p_l target poses
     * @param d_l dist result
     */
    bool DjkstraLocalDist(const Eigen::Vector3d &pos, list<list<Eigen::Vector3d>> &paths, list<list<Eigen::Vector3d>> &pruned_paths, 
                                                                                        list<Eigen::Vector3d> &p_l, list<double> &d_l);
    void GetSafePath(list<Eigen::Vector3d> &danger_path, list<Eigen::Vector3d> &safe_path, 
        list<Eigen::Vector3d> &unknown_path, const double &max_dist, const double &unknown_dist);
    inline int PostoId(const Eigen::Vector3d &pos);//dont check in side map, carefully use
    inline Eigen::Vector3d GetRobotSize(){return Robot_size_;}

    inline Eigen::Vector3d GetCenterPos(const Eigen::Vector3d &p);
    inline bool StrangePoint(Eigen::Vector3d &p); 
    inline shared_ptr<LR_node> GetNode(const Eigen::Vector3d &pos);//check if pos is in the block
    inline shared_ptr<LR_node> GetNode(const int &id);//don't check, carefully use
    inline shared_ptr<LR_node> GetNode(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG);//don't check
    inline Eigen::Vector3d IdtoPos(int id);//
    inline Eigen::Vector3d IdtoPos(const Eigen::Vector3i &id);//
    void Debug(vector<Eigen::Vector3d> &pts);
    void Debug(const uint32_t &id);
    void Debug(vector<Eigen::Vector4d> &pts);
    void Debug2(vector<Eigen::Vector3d> &pts);
    void Debug2(list<Eigen::Vector3d> &pts);
    void Debug2(list<list<Eigen::Vector3d>> &paths);
    void DebugTie(const Eigen::Vector3d &p);

    list<pair<int, list<pair<int, Eigen::Vector3d>>>> frontier_list_;//<f_id, <v_id, viewpoint>>, check if the veiwpoint is in local space
    list<pair<pair<int, double>, pair<int, list<Eigen::Vector3d>>>> frontier_path_;//<f_id, length>, v_id, path>, frontier id and corresponding path to the viewpoint
    list<vector<Eigen::MatrixXd>> Inc_list_; // worker; path vec; path; path point
    vector<Eigen::MatrixXd> Inc_listII_;
    Eigen::Vector3d local_up_bd_, local_low_bd_;
    tr1::unordered_map<uint32_t, pair<int, float>> id_idx_dist_;          //hands shaking hnodes, for updating local Hnodes
    list<pair<uint32_t, Eigen::Vector3d>> id_Hpos_dist_;                  // local H
    list<uint32_t> h_id_clear_;                                           //for DTG edge checking
    list<pair<Eigen::Vector3i, lr_root>> idx_tie_clear_;

    Eigen::Vector3d origin_, mapscale_, blockscale_, node_scale_, edgeblock_scale_, map_upbd_, map_lowbd_;
    uint32_t prep_idx_;
    uint32_t cur_root_h_id_;
    double cur_h_cost_;
    Eigen::Vector3d cur_root_h_idx_;
    double range_;

private:

    void ShowGridLocal(const ros::TimerEvent& e);
    void ExpandLocalMap();
    void ExpandTopoMap();
    void LoadShowList();
    void ClearXNodes();

    bool Astar(const Eigen::Vector3d &start, const Eigen::Vector3d &end, vector<Eigen::Vector3d> &path, const int &workid, int search_num, bool local = false);
    bool Astar(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &path, const int &workid, int search_num, bool local = false);
    void Djkstra(int workid, Eigen::Vector3d start, list<Eigen::Vector3d> targets);
    //for DTG update
    void DjkstraLocal(Eigen::Vector3d start);
    void GetClosestTarget(LowResMap::target_dict &t_dict, Eigen::Vector3i std_start, Eigen::Vector3i &std_end);
    void RetrievePath(vector<Eigen::Vector3d> &path, shared_ptr<sch_node> &end_node);
    void RetrievePath(list<Eigen::Vector3d> &path, shared_ptr<sch_node> &end_node);
    void RetrievePath(Eigen::MatrixXd &path, shared_ptr<sch_node> &end_node);
    bool GetWorker(int &workid);
    void FreeWorker(const int &workid, vector<shared_ptr<sch_node>> &nodelist);
    void ReSortOpenset(int &workid, Eigen::Vector3i &std_end);
    bool ExpandPath(Eigen::Vector3i &cor_start, Eigen::Vector3i &cor_end, const Eigen::Vector3d &pos);
    void CoarseExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end);
    void FineExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end, Eigen::Vector3d &up_corner, Eigen::Vector3d &down_corner);
    void UpdateFOV();
    bool PrunePathTempt(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length);

    inline double GetHue(const Eigen::Vector3i &p1, const Eigen::Vector3i &p2);       //For astar
    inline double GetDist(const int &dx, const int &dy, const int &dz);       //For astar
    inline double GetHueL1(const Eigen::Vector3i &p1, const Eigen::Vector3i &p2);       //For astar
    inline double GetDistL1(const Eigen::Vector3i &dp);       //For astar


    inline bool GetBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid);//check
    inline int GetBlockId(const Eigen::Vector3d &pos);//check
    inline int GetBlockId(const Eigen::Vector3i &pos);//check, pos: block id/carefully use 
    inline int GetNodeId(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG);//don't check, pos of world
    inline int GetNodeId(const Eigen::Vector3i &pos, const shared_ptr<LR_block> &FG);//don't check, pos of world


    inline bool EraseNode(const Eigen::Vector3d &pos);
    inline bool EraseNode(const Eigen::Vector3i &pos);
    inline bool EraseNode(const int &id);
    inline bool HaveLocalNeighbour(const Eigen::Vector3i &pos);
    
    inline void GetExistNeighbours(int id, list<int> &Existlist, list<int> &Expandlist);  //check
    inline void GetExpandNeighbours(int id, Eigen::Vector3i &posid, list<int> &Expandlist, list<int> &Existlist);  //check
    inline void GetTopoNeighbours(shared_ptr<sch_node> &node);
    inline void SetLocalH(const shared_ptr<LR_node> node, const int &idx, const double &dist2robot);

    inline bool InsideFOV(const Eigen::Vector3d pos);
    
    inline pair<int,int>  SetNode(const Eigen::Vector3d &pos, bool get_h = false);
    inline void SetXNode(const Eigen::Vector3d &pos, const double &time);
    inline void SetUNode(const Eigen::Vector3d &pos, const double &time);
    inline void SetEXPNode(const Eigen::Vector3d &pos);         
    inline void SetTopoNode(const Eigen::Vector3d &pos, shared_ptr<LR_node> &lr_node, shared_ptr<sch_node> &c_node);         
    inline void SetEXPNode(const int &b_id, const int &n_id, const Eigen::Vector3i &bk3i);       
    inline void PostoId3(const Eigen::Vector3d &pos, Eigen::Vector3i &id3);              //dont check
    inline Eigen::Vector3i GetBolckSize(const Eigen::Vector3i &blockid);              //dont check
    inline int CheckNode(const Eigen::Vector3i pos);
    inline void UpdateTie(const int &bid, const int &nid, const uint32_t &rid, bool new_h);
    inline bool TopoDirIter(const u_char &dir, Eigen::Vector3i &idx, double &length); 
    inline void MultiTopoDirIter(const u_char &dir, const Eigen::Vector3i &idx, vector<Eigen::Vector3i> &neighbours); 
    inline u_char InverseDir(const u_char &dir); 
    ros::NodeHandle nh_, nh_private_;
    BlockMap *map_;
    // volumetric_mapping::OctomapManager * Octomap_;
    vector<shared_ptr<LR_block>> gridBLK_;

    ros::Publisher node_pub_, debug_pub_;
    ros::Timer show_timer_;

    bool debug_, showmap_;
    vector<int> localnode_list_; 


    Eigen::Vector3i block_size_, voxel_num_, block_num_;       //block_size: size of voxels in a block, block_num_:total number of blocks
    Eigen::Vector3i v_n_, b_n_;//for quick compute
    Eigen::Vector3i edgeblock_size_, local_origin_, localgraph_size_, local_up_idx_, local_low_idx_;
    Eigen::Vector3i node_size_;                                //expand, for collision check;
    double resolution_;
    bool first_move_;
    double occ_duration_, unknown_duration_;

    list<int> Showblocklist_;
    vector<bool> Astar_worktable_; //for multi thread A* search
    vector<prio_A> open_set_; 
    //for UpdateLocalTopo
    prio_D open_TS_, open_NTS_; 

    list<pair<int, Eigen::Vector3d>> check_listII_;
    mutex mtx_, Astar_mtx_;
    list<pair<int, int>> Xlist_;
    list<pair<int, int>> Topolist_, H_Topolist_;

    vector<int> DeadBlockList_;
    Eigen::Matrix4d Robot_pose_;
    Eigen::Vector3d Robot_pos_;
    shared_ptr<LR_node> /*Xnode_,*/ Outnode_, Rootnode_, Expandnode_; 
    int Eternal_bid_, Eternal_nid_;
    bool workable_;          
    double lambda_heu_; 

    //for local Topo space check 
    Eigen::Matrix4d FOV_ieqs_;  //ax+by+cz+d<=0
    double ver_up_dir_, ver_down_dir_, hor_left_dir_, hor_right_dir_;
    double max_g_cost_, h_thresh_, eu_range_;        
    bool show_dtg_;

    //for prune path
    double seg_length_, prune_seg_length_;
    Eigen::Vector3d expand_r_, Robot_size_;
    Eigen::Vector3i corridor_exp_r_;

    uint32_t root_h_id_;
    ColorManager *CM_;

    int drone_num_;
};

inline bool LowResMap::GetBlock3Id(const Eigen::Vector3d &pos, Eigen::Vector3i &blkid){
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

inline Eigen::Vector3d LowResMap::GetStdPos(const Eigen::Vector3d &pos){
    Eigen::Vector3i std_pos;
    PostoId3(pos, std_pos);
    return IdtoPos(std_pos);
}

// inline double LowResMap::GetLocalDist(const Eigen::Vector3d &pos){
//     shared_ptr<LR_node> lr_node = GetNode(pos);
//     if(lr_node == NULL || lr_node == Outnode_ || !lr_node->flags_[1]) return -1;
//     else{
//         return lr_node->topo_sch_->g_score_;
//     }
// }

// inline double LowResMap::GetBBXLocalDist(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, int &c_idx){
//     double dist = -1, dist_temp;
//     Vector3i s, e, i;
//     PostoId3(upbd, s);
//     PostoId3(lowbd, e);
//     for(int dim = 0; dim < 3; dim++){
//         if(s(dim) < 0 || e(dim) >= voxel_num_(dim)) return dist;
//     }
//     dist = 99999.0;
//     for(i(0) = s(0); i(0) <= e(0); i(0)++){
//         for(i(0) = s(0); i(0) <= e(0); i(0)++){
//             for(i(0) = s(0); i(0) <= e(0); i(0)++){
//                 dist_temp = GetLocalDist(IdtoPos(i));
//                 if(dist_temp > 0 && dist_temp < dist){
//                     dist = dist_temp;
//                     c_idx = i(2)*v_n_(1) + i(1)*v_n_(0) + i(0);
//                 }
//             }
//         }
//     }
//     if(dist > 99998.0) dist = -1;
//     return dist;
// }

inline bool LowResMap::IsFeasible(const Eigen::VectorXd &pos, bool allow_uknown){
    Eigen::Vector3d p(pos(0), pos(1), pos(2));
    int blockid = GetBlockId(p);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = gridBLK_[blockid];
        if(fp_ptr != NULL){
            shared_ptr<LR_node> node = GetNode(p, fp_ptr);
            if(node != NULL && node != Outnode_ && !node->flags_[0]) return true;
            else if(allow_uknown && node == NULL) return true;
            else if(allow_uknown && node != NULL && node != Outnode_ && node->flags_[3]) return true;
            else return false;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

inline bool LowResMap::IsFeasible(const Eigen::Vector3d &pos, bool allow_uknown){
    int blockid = GetBlockId(pos);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = gridBLK_[blockid];
        if(fp_ptr != NULL){
            shared_ptr<LR_node> node = GetNode(pos, fp_ptr);
            if(node != NULL && node != Outnode_ && !node->flags_[0]) return true;
            else if(allow_uknown && node == NULL) return true;
            else if(allow_uknown && node != NULL && node != Outnode_ && node->flags_[3]) return true;
            else return false;
        }
        else{
            if(allow_uknown) return true;
            return false;
        }
    }
    else{
        return false;
    }
}

inline bool LowResMap::IsFeasible(const Eigen::Vector3i &pos, bool allow_uknown){
    Eigen::Vector3d p = IdtoPos(pos);
    int blockid = GetBlockId(p);
    shared_ptr<LR_block> fp_ptr = gridBLK_[blockid];
    if(fp_ptr != NULL){
        // Eigen::Vector3d p = IdtoPos(pos);
        shared_ptr<LR_node> node = GetNode(p, fp_ptr);
        if(node != NULL && node != Outnode_ && !node->flags_[0]) return true;
        else if(allow_uknown && node == NULL) return true;
        else if(allow_uknown && node != NULL && node != Outnode_ && node->flags_[3]) return true;
        else return false;
    }
    else{
        return false;
    }
}


inline bool LowResMap::IsLocalFeasible(const Eigen::Vector3d &pos){
    int blockid = GetBlockId(pos);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = gridBLK_[blockid];
        if(fp_ptr != NULL){
            shared_ptr<LR_node> node = GetNode(pos, fp_ptr);
            if(node != NULL && node != Outnode_ && !node->flags_[0]){
                if(node->flags_[1])
                    return true;
                else
                    return false;
            } 
            else return false;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}

inline bool LowResMap::RetrieveHPath(const int &start, const uint32_t &target_id, list<Eigen::Vector3d> &path, double &length){
    Eigen::Vector3d start_p = IdtoPos(start);
    Eigen::Vector3i start_idx;
    u_char parent_dir_;
    PostoId3(start_p, start_idx);
    path.clear();
    length = 0;
    int debug_count = 0;
    while(1){
        // cout<<"RetrieveHPath: "<<debug_count<<endl;
        int node_id = start_idx(2)*v_n_(1) + start_idx(1)*voxel_num_(0) + start_idx(0);
        shared_ptr<LR_node> cur_node = GetNode(node_id);
        path.push_back(IdtoPos(start_idx));
        if(cur_node == NULL || cur_node == Outnode_){ 
            cout<<"error:"<<start_idx.transpose()<<endl;
            cout<<"path l:"<<path.size()<<endl;
            cout<<"path:"<<path.front()<<endl;
            return false;
        }
        parent_dir_ = 0;
        for(auto &tie: cur_node->ties_){
            if(tie.root_id_ == target_id){
                parent_dir_ = tie.in_dir_;
            }
        }
        if(debug_count >= 3000) {//debug
            // cout<<target_id<<endl;
            Debug(target_id);
            Debug2(path);
            // getchar();
            return false;
        }
        debug_count++;
        // cout<<"dir:"<<int(parent_dir_)<<" idx:"<<start_idx.transpose()<<endl;
        if(!TopoDirIter(parent_dir_, start_idx, length)){
            // cout<<"retrieve l:"<<length<<endl;
            return true;
        }
    }
}

inline bool LowResMap::RetrieveHPath(const Eigen::Vector3d &start, const uint32_t &target_id, list<Eigen::Vector3d> &path, double &length){
    int start_id = PostoId(start);
    return RetrieveHPath(start_id, target_id, path, length);
}

inline bool LowResMap::RetrieveHPathShortest(const Eigen::Vector3d &start, uint32_t &target_id, list<Eigen::Vector3d> &path, double &length){
    int start_id = PostoId(start);
    shared_ptr<LR_node> node = GetNode(start_id);
    if(node == NULL || node->flags_[0] || node->ties_.empty()) return false;
    float min_l = 99999.0;
    for(auto &tie : node->ties_){
        if(tie.dist_ < min_l) {
            min_l = tie.dist_;
            target_id = tie.root_id_;
        }
    }
    return RetrieveHPath(start_id, target_id, path, length);
}

inline bool LowResMap::ShortestLength2Root(const Eigen::Vector3d &start, uint32_t &target_id, double &length){
    int start_id = PostoId(start);
    shared_ptr<LR_node> node = GetNode(start_id);
    if(node == NULL || node->flags_[0] || node->ties_.empty()) return false;
    float min_l = 99999.0;
    for(auto &tie : node->ties_){
        if(tie.dist_ < min_l) {
            min_l = tie.dist_;
            target_id = tie.root_id_;
        }
    }
    length = min_l;
    return true;
}

inline void LowResMap::RetrieveHPathTemp(const int &start, list<Eigen::Vector3d> &path, double &length){
    Eigen::Vector3d start_p = IdtoPos(start);
    Eigen::Vector3i start_idx;
    PostoId3(start_p, start_idx);
    path.clear();
    length = 0;

    int node_id = start_idx(2)*v_n_(1) + start_idx(1)*voxel_num_(0) + start_idx(0);
    shared_ptr<LR_node> node = GetNode(node_id);
    shared_ptr<sch_node> cur_node, next_node;
    if(node == NULL) {//debug
        ROS_ERROR("error RetrieveHPathTemp1!");
        ros::shutdown();
        return;
    }

    cur_node = node->topo_sch_;
    if(cur_node == NULL || cur_node->status_ != in_close) {//debug
        ROS_ERROR("error RetrieveHPathTemp2!");
        ros::shutdown();
        return;
    }
    path.push_back(IdtoPos(cur_node->pos_));
    while (1)
    {   
        if(cur_node->parent_ == NULL) return;
        start_p = IdtoPos(cur_node->pos_);
        length += (start_p - path.back()).norm();
        path.emplace_back(start_p);
        cur_node = cur_node->parent_;
    }
}

inline shared_ptr<LR_node> LowResMap::GetNode(const Eigen::Vector3d &pos){
    int blockid = GetBlockId(pos);
    if(blockid != -1){
        shared_ptr<LR_block> fp_ptr = gridBLK_[blockid];
        if(fp_ptr != NULL){
            return GetNode(pos, fp_ptr);
        }
        else{
            return NULL;
        }
    }
    else{
        return Outnode_;
    }
}

inline shared_ptr<LR_node> LowResMap::GetNode(const int &id){
    Eigen::Vector3d pos = IdtoPos(id);
    int blockid = GetBlockId(pos);
    if(blockid == -1) {
        return Outnode_;
    }
    else if(gridBLK_[blockid] == NULL){
        return NULL;
    }
    return gridBLK_[blockid]->local_grid_[GetNodeId(pos, gridBLK_[blockid])];
}

inline shared_ptr<LR_node> LowResMap::GetNode(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG){
    return FG->local_grid_[GetNodeId(pos, FG)];
}

inline int LowResMap::GetBlockId(const Eigen::Vector3d &pos){
    if(InsideMap(pos)){
        Eigen::Vector3d dpos = pos - origin_;
        Eigen::Vector3i posid;
        posid.x() = floor(dpos.x() / blockscale_.x());
        posid.y() = floor(dpos.y() / blockscale_.y());
        posid.z() = floor(dpos.z() / blockscale_.z());
        return posid(2)*b_n_(1) + posid(1)*b_n_(0) + posid(0);
    }
    else{
        return -1;
    }
}

inline int LowResMap::GetBlockId(const Eigen::Vector3i &pos){
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  block_num_(0) || pos(1) >= block_num_(1) || pos(2) >= block_num_(2)){
            return -1;
        }
    else{
        return pos(2)*b_n_(1) + pos(1)*b_n_(0) + pos(0);
    }
}

inline int LowResMap::GetNodeId(const Eigen::Vector3d &pos, const shared_ptr<LR_block> &FG){
    Eigen::Vector3d dpos = pos - origin_ - FG->origin_.cast<double>().cwiseProduct(node_scale_);
    Eigen::Vector3i posid;
    posid.x() = floor(dpos(0) / node_scale_(0));
    posid.y() = floor(dpos(1) / node_scale_(1));
    posid.z() = floor(dpos(2) / node_scale_(2));

    return posid(2)*FG->block_size_(0)*FG->block_size_(1) + posid(1)*FG->block_size_(0) + posid(0);
}

inline int LowResMap::GetNodeId(const Eigen::Vector3i &pos, const shared_ptr<LR_block> &FG){
    Eigen::Vector3i dpos = pos - FG->origin_;
    return dpos(2)*(FG->block_size_(0))*(FG->block_size_(1)) + dpos(1)*(FG->block_size_(0)) + dpos(0);
}

inline bool LowResMap::InsideMap(const Eigen::VectorXd &pos){
    if(pos(0) < map_lowbd_(0) || pos(1) < map_lowbd_(1) || pos(2) < map_lowbd_(2) ||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool LowResMap::InsideMap(const Eigen::Vector3d &pos){
    if(pos(0) < map_lowbd_(0) || pos(1) < map_lowbd_(1) || pos(2) < map_lowbd_(2) ||
        pos(0) >  map_upbd_(0) || pos(1) > map_upbd_(1) || pos(2) > map_upbd_(2) )
        return false;
    return true;
}

inline bool LowResMap::InsideMap(const Eigen::Vector3i &pos){
    if(pos(0) < 0 || pos(1) < 0 || pos(2) < 0 ||
        pos(0) >=  voxel_num_(0) || pos(1) >= voxel_num_(1) || pos(2) >= voxel_num_(2))
        return false;
    return true;
}

inline Eigen::Vector3d LowResMap::IdtoPos(int id){
    int x = id % voxel_num_(0);
    int y = ((id - x)/voxel_num_(0)) % voxel_num_(1);
    int z = ((id - x) - y*voxel_num_(0))/v_n_(1);
    return Eigen::Vector3d((double(x)+0.5)*node_scale_(0),(double(y)+0.5)*node_scale_(1),(double(z)+0.5)*node_scale_(2))+origin_;
}   

inline Eigen::Vector3d LowResMap::IdtoPos(const Eigen::Vector3i &id){
    return Eigen::Vector3d((double(id(0))+0.5)*node_scale_(0),(double(id(1))+0.5)*node_scale_(1),(double(id(2))+0.5)*node_scale_(2))+origin_;
}

inline int LowResMap::PostoId(const Eigen::Vector3d &pos){
    return floor((pos(2)-origin_(2))/node_scale_(2))*v_n_(1)+
        floor((pos(1)-origin_(1))/node_scale_(1))*voxel_num_(0)+floor((pos(0)-origin_(0))/node_scale_(0));
}


inline bool LowResMap::EraseNode(const Eigen::Vector3d &pos){
    int blockid = GetBlockId(pos);
    if(blockid == -1 || gridBLK_[blockid] == NULL){
          std::cout << "\033[0;34m EraseNode(3d) Can not erase: \033[0m"<<pos.transpose()<<" "<<(gridBLK_[blockid]==NULL)<< std::endl;
        return false;
    }
    int nodeid = GetNodeId(pos, gridBLK_[blockid]);
    gridBLK_[blockid]->local_grid_[nodeid] = NULL;
    gridBLK_[blockid]->alive_num_--;
    if(gridBLK_[blockid]->alive_num_ == 0){
        DeadBlockList_.push_back(blockid);
    }
    return true;
}

inline bool LowResMap::EraseNode(const Eigen::Vector3i &pos){
    Eigen::Vector3i blkpos(floor(pos(0)/double(block_size_(0))), floor(pos(1)/double(block_size_(1))),floor(pos(2)/double(block_size_(2))));
    int blockid = GetBlockId(blkpos);
    if(blockid == -1 || gridBLK_[blockid] == NULL){
          std::cout << "\033[0;34m EraseNode(3i) Can not erase: \033[0m"<<pos.transpose()<<" "<<(gridBLK_[blockid]==NULL)<< std::endl;
        return false;
    }
    int nodeid = GetNodeId(pos, gridBLK_[blockid]);
    gridBLK_[blockid]->local_grid_[nodeid] = NULL;
    gridBLK_[blockid]->alive_num_--;
    if(gridBLK_[blockid]->alive_num_ == 0){
        DeadBlockList_.push_back(blockid);
    }
    return true;
}

inline bool LowResMap::EraseNode(const int &id){
    if(id < 0 || id >= v_n_(1)*voxel_num_(2)){
        std::cout << "\033[0;34m EraseNode(id) Can not erase: \033[0m"<<id<<std::endl;
        return false;
    }
    Eigen::Vector3d pos = IdtoPos(id);
    return EraseNode(pos);
}

inline void LowResMap::GetExistNeighbours(int id, list<int> &Existlist, list<int> &Expandlist){
    if(id >= v_n_(1)*voxel_num_(2) || id < 0) return;

    Eigen::Vector3i center, blockid, nodeid;
    Eigen::Vector3d pos;
    int b_id, n_id;
    int node_id;
    center(0) = id % voxel_num_(0);
    center(1) = ((id - center(0))/voxel_num_(0)) % voxel_num_(1);
    center(2) = ((id - center(0)) - center(1)*voxel_num_(0))/v_n_(1);
    
    for(int i = 0; i < 3; i++){
        center(i) += 1;
        if(center(i) <= local_up_idx_(i)){
            blockid(0) = floor(center(0) / block_size_(0));
            blockid(1) = floor(center(1) / block_size_(1));
            blockid(2) = floor(center(2) / block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            nodeid = center - blockid.cwiseProduct(block_size_);
            b_id = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
            n_id = nodeid(2)*blocksize(0)*blocksize(1) + nodeid(1)*blocksize(0) + nodeid(0);
            node_id = center(2)*v_n_(1) + center(1)*voxel_num_(0) + center(0);
            pos = (center.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5)).cwiseProduct(node_scale_) + origin_;
            if(gridBLK_[b_id] != NULL){
                if(gridBLK_[b_id]->local_grid_[n_id] == NULL){
                    // SetEXPNode(pos);
                    SetEXPNode(b_id, n_id, blockid);
                    Expandlist.push_back(node_id);
                }
                else if(!(gridBLK_[b_id]->local_grid_[n_id]->flags_[2]) && !gridBLK_[b_id]->local_grid_[n_id]->flags_[1]
                    && !gridBLK_[b_id]->local_grid_[n_id]->flags_[0]){
                    localnode_list_.push_back(node_id);
                    // if(!gridBLK_[b_id]->local_grid_[n_id]->vp_candidate_){
                    //     // localVpNodes_list_.push_back({b_id, n_id});
                    //     gridBLK_[b_id]->local_grid_[n_id]->vp_candidate_ = true;
                    // }
                    gridBLK_[b_id]->local_grid_[n_id]->flags_.set(1);
                    Existlist.push_back(node_id);
                }
            }
            else{
                // SetEXPNode(pos);
                SetEXPNode(b_id, n_id, blockid);
                Expandlist.push_back(node_id);
            }
        }
        center(i) -= 2;
        if(center(i) >= local_low_idx_(i)){
            blockid(0) = floor(center(0) / block_size_(0));
            blockid(1) = floor(center(1) / block_size_(1));
            blockid(2) = floor(center(2) / block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            nodeid = center - blockid.cwiseProduct(block_size_);
            b_id = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
            n_id = nodeid(2)*blocksize(0)*blocksize(1) + nodeid(1)*blocksize(0) + nodeid(0);
            node_id = center(2)*v_n_(1) + center(1)*voxel_num_(0) + center(0);
            pos = (center.cast<double>() + Eigen::Vector3d(0.5,0.5,0.5)).cwiseProduct(node_scale_) + origin_;
            if(gridBLK_[b_id] != NULL){
                if(gridBLK_[b_id]->local_grid_[n_id] == NULL){
                    // SetEXPNode(pos);
                    SetEXPNode(b_id, n_id, blockid);
                    Expandlist.push_back(node_id);
                }
                else if(!(gridBLK_[b_id]->local_grid_[n_id]->flags_[2]) && !(gridBLK_[b_id]->local_grid_[n_id]->flags_[1])
                 && !gridBLK_[b_id]->local_grid_[n_id]->flags_[0]){
                    localnode_list_.push_back(node_id);
                    gridBLK_[b_id]->local_grid_[n_id]->flags_.set(1);
                    Existlist.push_back(node_id);
                }
            }
            else{
                // SetEXPNode(pos);
                SetEXPNode(b_id, n_id, blockid);
                Expandlist.push_back(node_id);
            }
        }
        center(i) += 1;
    }
}

inline void LowResMap::GetExpandNeighbours(int id, Eigen::Vector3i &posid, list<int> &Expandlist, list<int> &Existlist){
    if(id >= v_n_(2) || id < 0) return;

    Eigen::Vector3i center, blockid, nodeid;
    Eigen::Vector3d pos;
    int b_id, n_id;
    int node_id;
    center = posid;
    for(int i = 0; i < 3; i++){
        center(i) += 1;
        if(center(i) <= local_up_idx_(i)){
            blockid(0) = floor(center(0) / block_size_(0));
            blockid(1) = floor(center(1) / block_size_(1));
            blockid(2) = floor(center(2) / block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            nodeid = center - blockid.cwiseProduct(block_size_);
            b_id = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
            n_id = nodeid(2)*blocksize(0)*blocksize(1) + nodeid(1)*blocksize(0) + nodeid(0);
            node_id = center(2)*v_n_(1) + center(1)*v_n_(0) + center(0);
            pos = (center.cast<double>()+Eigen::Vector3d(0.5,0.5,0.5)).cwiseProduct(node_scale_) + origin_;
            if(gridBLK_[b_id] != NULL){
                if(gridBLK_[b_id]->local_grid_[n_id] == NULL){
                    SetEXPNode(b_id, n_id, blockid);
                    Expandlist.push_back(node_id);
                }
                else if(!(gridBLK_[b_id]->local_grid_[n_id]->flags_[2]) && !gridBLK_[b_id]->local_grid_[n_id]->flags_[0]){
                    Existlist.push_back(node_id);
                }
            }
            else{
                SetEXPNode(b_id, n_id, blockid);
                Expandlist.push_back(node_id);
            }
        }
        center(i) -= 2;
        if(center(i) >= local_low_idx_(i)){
            blockid(0) = floor(center(0) / block_size_(0));
            blockid(1) = floor(center(1) / block_size_(1));
            blockid(2) = floor(center(2) / block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            nodeid = center - blockid.cwiseProduct(block_size_);
            b_id = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
            n_id = nodeid(2)*blocksize(0)*blocksize(1) + nodeid(1)*blocksize(0) + nodeid(0);
            node_id = center(2)*v_n_(1) + center(1)*voxel_num_(0) + center(0);
            pos = (center.cast<double>()+Eigen::Vector3d(0.5,0.5,0.5)).cwiseProduct(node_scale_) + origin_;
            if(gridBLK_[b_id] != NULL){
                if(gridBLK_[b_id]->local_grid_[n_id] == NULL){
                    SetEXPNode(b_id, n_id, blockid);
                    Expandlist.push_back(node_id);
                }
                else if(!(gridBLK_[b_id]->local_grid_[n_id]->flags_[2]) && !gridBLK_[b_id]->local_grid_[n_id]->flags_[0]){
                    Existlist.push_back(node_id);
                }
            }
            else{
                SetEXPNode(b_id, n_id, blockid);
                Expandlist.push_back(node_id);
            }
        }
        center(i) += 1;
    }
}

inline void LowResMap::GetTopoNeighbours(shared_ptr<sch_node> &node){
    shared_ptr<LR_node> lr_node;
    shared_ptr<sch_node> c_node;
    Eigen::Vector3d pos;
    for(int dim = 0; dim < 3; dim++){
        Eigen::Vector3i diff(0, 0, 0);
        for(int off = -1; off <= 1; off += 2){
            diff(dim) = off;
            pos = IdtoPos(diff+node->pos_);
            lr_node = GetNode(pos);
            double g = node->g_score_ + GetDist(diff(0), diff(1), diff(2));
            if((g > max_g_cost_ || (Robot_pos_ - pos).norm() > eu_range_)) continue;

            if(lr_node != NULL && lr_node->flags_[0]){
                continue;
            } 

            if(lr_node != NULL) c_node = lr_node->topo_sch_;
            else c_node = NULL;

            if(c_node == NULL){                         //create a new node
                c_node = make_shared<sch_node>();
                c_node->pos_ = node->pos_ + diff;
                c_node->g_score_ = g;
                c_node->parent_ = node;
                c_node->status_ = in_open;
                if(lr_node == NULL){// NTS
                    SetTopoNode(pos, lr_node, c_node);
                    open_NTS_.push(c_node);
                }
                else{// TS
                    open_TS_.push(c_node);
                    lr_node->topo_sch_ = c_node;
                }
            }
            else{
                if(c_node->status_ == in_close) continue;
                if(g + 1e-4 < c_node->g_score_){
                    if(lr_node->flags_[2]){// NTS
                        lr_node->topo_sch_ = make_shared<sch_node>(c_node);
                        lr_node->topo_sch_->g_score_ = g;
                        lr_node->topo_sch_->parent_ = node;
                        open_NTS_.push(lr_node->topo_sch_);
                    }
                    else{
                        lr_node->topo_sch_ = make_shared<sch_node>(c_node);
                        lr_node->topo_sch_->g_score_ = g;
                        lr_node->topo_sch_->parent_ = node;
                        open_TS_.push(lr_node->topo_sch_);
                    }
                    c_node->status_ = in_close;
                }
            }
        }
    }
}

inline void LowResMap::SetLocalH(const shared_ptr<LR_node> node,  const int &idx, const double &dist2robot){
    for(auto &tie : node->ties_){
        tr1::unordered_map<uint32_t, pair<int, float>>::iterator tie_it = id_idx_dist_.find(tie.root_id_);
        if(tie_it != id_idx_dist_.end()){
            if(tie.in_dir_ == 128){
                list<Eigen::Vector3d> path, path_prune;
                double length;
                RetrieveHPathTemp(idx, path, length);
                if(PrunePathTempt(path, path_prune, length)){
                    if(cur_root_h_id_ == 0 && h_thresh_ > length && cur_h_cost_ > length){
                        cur_root_h_id_ = tie.root_id_;
                        cur_root_h_idx_ = IdtoPos(idx);
                        cur_h_cost_ = length;
                        id_idx_dist_.erase(tie_it);
                    }
                    else id_Hpos_dist_.push_back({tie.root_id_, IdtoPos(idx)});
                }
                else{
                    ROS_ERROR("error PrunePathTempt!");
                    for(auto &p : path){
                        cout<<p.transpose()<<endl;
                    }
                    ros::shutdown();
                }
            }
            else{
                if(tie_it->second.second > tie.dist_ + dist2robot){
                    tie_it->second.first = idx;
                    tie_it->second.second = tie.dist_ + dist2robot;
                }
            }
        }
        else{
            if(tie.in_dir_ == 128){
                list<Eigen::Vector3d> path, path_prune;
                double length;
                RetrieveHPathTemp(idx, path, length);
                if(PrunePathTempt(path, path_prune, length)){
                    // cout<<length<<endl;
                    if(cur_root_h_id_ == 0 && h_thresh_ > length && cur_h_cost_ > length){
                        cur_root_h_id_ = tie.root_id_;
                        cur_root_h_idx_ = IdtoPos(idx);
                        cur_h_cost_ = length;
                    }
                    else id_Hpos_dist_.push_back({tie.root_id_, IdtoPos(idx)});
                }
                else{
                    ROS_ERROR("error PrunePathTempt!");
                    for(auto &p : path){
                        cout<<p.transpose()<<endl;
                    }
                    ros::shutdown();
                }
            }
            else{
                bool new_h = !(tie.root_id_ == cur_root_h_id_);
                for(auto &l_h : id_Hpos_dist_){
                    if(l_h.first == tie.root_id_) {
                        new_h = false;
                        break;
                    }
                }
                if(new_h){
                    pair<uint32_t, pair<int, float>> it;
                    it.first = tie.root_id_;
                    it.second.first = idx;
                    it.second.second = tie.dist_ + dist2robot;
                    id_idx_dist_.insert(it);
                }
            }
        }
    }
}

// inline void LowResMap::GetTopoTSNeighbours(shared_ptr<sch_node> &node){
//     shared_ptr<LR_node> lr_node;
//     for(int dim = 0; dim < 3; dim++){
//         Eigen::Vector3i diff(0, 0, 0);
//         for(int off = -1; off <= 1; off += 2){
//             diff(dim) = off;
//             lr_node = GetNode(IdtoPos(diff+node->pos_));

//         }
//     }
// }

inline bool LowResMap::InsideFOV(const Eigen::Vector3d pos){
    double x, y, z;
    x = pos(0);
    y = pos(1);
    z = pos(2);
    if(FOV_ieqs_(0, 0) * x + FOV_ieqs_(1, 0) * y + FOV_ieqs_(2, 0) * z + FOV_ieqs_(3, 0) > 0 
      || FOV_ieqs_(0, 1) * x + FOV_ieqs_(1, 1) * y + FOV_ieqs_(2, 1) * z + FOV_ieqs_(3, 1) > 0 
      || FOV_ieqs_(0, 2) * x + FOV_ieqs_(1, 2) * y + FOV_ieqs_(2, 2) * z + FOV_ieqs_(3, 2) > 0 
      || FOV_ieqs_(0, 3) * x + FOV_ieqs_(1, 3) * y + FOV_ieqs_(2, 3) * z + FOV_ieqs_(3, 3) > 0 
      || (pos - Robot_pos_).norm() > range_){
        return false;
    }
    return true;
}

inline pair<int,int> LowResMap::SetNode(const Eigen::Vector3d &pos, bool get_h){
    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        int blkid = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
        if(gridBLK_[blkid] == NULL){
            gridBLK_[blkid] = make_shared<LR_block>();
            gridBLK_[blkid]->origin_ = Eigen::Vector3i(blockid(0)*block_size_(0), 
                blockid(1)*block_size_(1), blockid(2)*block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            gridBLK_[blkid]->block_size_ = blocksize;
            gridBLK_[blkid]->local_grid_.resize(blocksize(0)*blocksize(1)*blocksize(2));
        }
        //set node
        int nodeid = GetNodeId(pos, gridBLK_[blkid]);
        if(gridBLK_[blkid]->local_grid_[nodeid] == NULL){
            ROS_ERROR("Expanding NULL NODE %d %d!!!", blkid, nodeid);
        }
        else if(gridBLK_[blkid]->local_grid_[nodeid] == Expandnode_){
            gridBLK_[blkid]->local_grid_[nodeid] = make_shared<LR_node>();
            gridBLK_[blkid]->alive_num_++;
        }

        gridBLK_[blkid]->local_grid_[nodeid]->flags_.set(1);

        if(get_h) {
            Topolist_.push_back({blkid, nodeid});
            PostoId3(pos, blockid);//
            int idx = PostoId(pos);
            SetLocalH(gridBLK_[blkid]->local_grid_[nodeid], idx, gridBLK_[blkid]->local_grid_[nodeid]->topo_sch_->g_score_);
        }
        return {blkid, nodeid};
    }
    return {-1, -1};
}       

inline void LowResMap::SetXNode(const Eigen::Vector3d &pos, const double &time){
    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        int blkid = blockid(2)*b_n_(1) + blockid(1)*block_num_(0) + blockid(0);
        if(gridBLK_[blkid] == NULL){
            gridBLK_[blkid] = make_shared<LR_block>();
            gridBLK_[blkid]->origin_ = Eigen::Vector3i(blockid(0)*block_size_(0), 
                blockid(1)*block_size_(1), blockid(2)*block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            gridBLK_[blkid]->block_size_ = blocksize;
            gridBLK_[blkid]->local_grid_.resize(blocksize(0)*blocksize(1)*blocksize(2));
        }
        //set node
        int nodeid = GetNodeId(pos, gridBLK_[blkid]);
        
        if(gridBLK_[blkid]->local_grid_[nodeid] == Expandnode_ || gridBLK_[blkid]->local_grid_[nodeid] == NULL){
            gridBLK_[blkid]->local_grid_[nodeid] = make_shared<LR_node>();
            gridBLK_[blkid]->alive_num_++;
            Xlist_.push_back({blkid, nodeid});
        }
        // else if(blkid == Eternal_bid_ && nodeid == Eternal_nid_){
        //     gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset(1);
        //     gridBLK_[blkid]->local_grid_[nodeid]->last_update_ = ros::Time::now().toSec();
        //     gridBLK_[blkid]->local_grid_[nodeid]->flags_.set(0);
        //     return;
        // }
        else if(!gridBLK_[blkid]->local_grid_[nodeid]->flags_[0]){
            Xlist_.push_back({blkid, nodeid});
            Eigen::Vector3i nodeidx;
            PostoId3(pos, nodeidx);
            for(auto &tie : gridBLK_[blkid]->local_grid_[nodeid]->ties_){
                h_id_clear_.emplace_back(tie.root_id_);
                idx_tie_clear_.push_back({nodeidx, tie});
            }
        }
        
        // gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset(1);
        // gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset(2);
        gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset();
        gridBLK_[blkid]->local_grid_[nodeid]->last_update_ = time;
        gridBLK_[blkid]->local_grid_[nodeid]->flags_.set(0);
    }
}      

inline void LowResMap::SetUNode(const Eigen::Vector3d &pos, const double &time){
    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        int blkid = blockid(2)*b_n_(1) + blockid(1)*block_num_(0) + blockid(0);
        if(gridBLK_[blkid] == NULL){
            gridBLK_[blkid] = make_shared<LR_block>();
            gridBLK_[blkid]->origin_ = Eigen::Vector3i(blockid(0)*block_size_(0), 
                blockid(1)*block_size_(1), blockid(2)*block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            gridBLK_[blkid]->block_size_ = blocksize;
            gridBLK_[blkid]->local_grid_.resize(blocksize(0)*blocksize(1)*blocksize(2));
        }
        //set node
        int nodeid = GetNodeId(pos, gridBLK_[blkid]);
        
        if(gridBLK_[blkid]->local_grid_[nodeid] == Expandnode_ || gridBLK_[blkid]->local_grid_[nodeid] == NULL){
            gridBLK_[blkid]->local_grid_[nodeid] = make_shared<LR_node>();
            gridBLK_[blkid]->alive_num_++;
            Xlist_.push_back({blkid, nodeid});
        }
        // else if(blkid == Eternal_bid_ && nodeid == Eternal_nid_){
        //     gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset(1);
        //     gridBLK_[blkid]->local_grid_[nodeid]->last_update_ = ros::Time::now().toSec();
        //     gridBLK_[blkid]->local_grid_[nodeid]->flags_.set(0);
        //     return;
        // }
        else if(!gridBLK_[blkid]->local_grid_[nodeid]->flags_[0]){
            Xlist_.push_back({blkid, nodeid});
            Eigen::Vector3i nodeidx;
            PostoId3(pos, nodeidx);
            for(auto &tie : gridBLK_[blkid]->local_grid_[nodeid]->ties_){
                h_id_clear_.emplace_back(tie.root_id_);
                idx_tie_clear_.push_back({nodeidx, tie});
            }
        }
        
        // gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset(1);
        // gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset(2);
        gridBLK_[blkid]->local_grid_[nodeid]->flags_.reset();
        gridBLK_[blkid]->local_grid_[nodeid]->last_update_ = time;
        gridBLK_[blkid]->local_grid_[nodeid]->flags_.set(3);
        gridBLK_[blkid]->local_grid_[nodeid]->flags_.set(0);
    }
}

inline void LowResMap::SetEXPNode(const Eigen::Vector3d &pos){
    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        int blkid = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
        if(gridBLK_[blkid] == NULL){
            gridBLK_[blkid] = make_shared<LR_block>();
            gridBLK_[blkid]->origin_ = Eigen::Vector3i(blockid(0)*block_size_(0), 
                blockid(1)*block_size_(1), blockid(2)*block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            gridBLK_[blkid]->block_size_ = blocksize;
            gridBLK_[blkid]->local_grid_.resize(blocksize(0)*blocksize(1)*blocksize(2));
        }
        //set node
        int nodeid = GetNodeId(pos, gridBLK_[blkid]);
        if(gridBLK_[blkid]->local_grid_[nodeid] != NULL){
            ROS_ERROR("Expanding expanded NODE %d %d!!!", blkid, nodeid);
        }
        // ROS_INFO("Expand NODE %d %d!!!", blkid, nodeid);
        gridBLK_[blkid]->local_grid_[nodeid] = Expandnode_;
        // EXPlist_.push_back({blkid, nodeid});
    }

}

inline void LowResMap::SetTopoNode(const Eigen::Vector3d &pos, shared_ptr<LR_node> &lr_node, shared_ptr<sch_node> &node){
    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        int blkid = blockid(2)*b_n_(1) + blockid(1)*b_n_(0) + blockid(0);
        if(gridBLK_[blkid] == NULL){
            gridBLK_[blkid] = make_shared<LR_block>();
            gridBLK_[blkid]->origin_ = Eigen::Vector3i(blockid(0)*block_size_(0), 
                blockid(1)*block_size_(1), blockid(2)*block_size_(2));
            Eigen::Vector3i blocksize = GetBolckSize(blockid);
            gridBLK_[blkid]->block_size_ = blocksize;
            gridBLK_[blkid]->local_grid_.resize(blocksize(0)*blocksize(1)*blocksize(2));
        }
        //set node
        int nodeid = GetNodeId(pos, gridBLK_[blkid]);
        if(gridBLK_[blkid]->local_grid_[nodeid] != NULL){
            ROS_ERROR("Expanding NULL TOPO NODE %d %d!!!", blkid, nodeid);
        }
        else{
            gridBLK_[blkid]->local_grid_[nodeid] = make_shared<LR_node>();
            lr_node = gridBLK_[blkid]->local_grid_[nodeid];
            gridBLK_[blkid]->local_grid_[nodeid]->topo_sch_ = node;
            gridBLK_[blkid]->alive_num_++;
        }
    }

}

inline void LowResMap::SetEXPNode(const int &b_id, const int &n_id, const Eigen::Vector3i &bk3i){
    if(b_id >= gridBLK_.size()){//debug
        cout<<GetBolckSize(bk3i).transpose()<<endl;
        cout<<b_id<<" "<<gridBLK_.size()<<endl;
        ROS_ERROR("SetEXPNode(const int &b_id, const int &n_id, const Eigen::Vector3i &bk3i)0");
        return;
    }
    if(gridBLK_[b_id] == NULL){
        gridBLK_[b_id] = make_shared<LR_block>();
        gridBLK_[b_id]->origin_ = Eigen::Vector3i(bk3i(0)*block_size_(0), 
            bk3i(1)*block_size_(1), bk3i(2)*block_size_(2));
        Eigen::Vector3i blocksize = GetBolckSize(bk3i);
        gridBLK_[b_id]->block_size_ = blocksize;
        gridBLK_[b_id]->local_grid_.resize(blocksize(0)*blocksize(1)*blocksize(2));
    }
    if(n_id >= gridBLK_[b_id]->local_grid_.size()){//debug
        cout<<bk3i.transpose()<<endl;
        cout<<GetBolckSize(bk3i).transpose()<<endl;
        cout<<b_id<<" "<<gridBLK_.size()<<endl;
        cout<<n_id<<" "<<gridBLK_[b_id]->local_grid_.size()<<endl;
        ROS_ERROR("SetEXPNode(const int &b_id, const int &n_id, const Eigen::Vector3i &bk3i)1");
        return;
    }
    //set node
    if(gridBLK_[b_id]->local_grid_[n_id] != NULL){
        ROS_ERROR("Expanding expanded NODE %d %d!!!", b_id, n_id);
    }
    gridBLK_[b_id]->local_grid_[n_id] = Expandnode_;
}

inline void LowResMap::PostoId3(const Eigen::Vector3d &pos, Eigen::Vector3i &id3){
    id3(0) = floor((pos(0)-origin_(0)) / node_scale_(0)); 
    id3(1) = floor((pos(1)-origin_(1)) / node_scale_(1)); 
    id3(2) = floor((pos(2)-origin_(2)) / node_scale_(2)); 
}

inline Eigen::Vector3i LowResMap::GetBolckSize(const Eigen::Vector3i &blockid){
    Eigen::Vector3i blocksize;
    if(blockid(0) == block_num_(0) - 1){
        blocksize(0) = edgeblock_size_(0);
    }
    else{
        blocksize(0)  = block_size_(0);
    }
    if(blockid(1) == block_num_(1) - 1){
        blocksize(1) = edgeblock_size_(1);
    }
    else{
        blocksize(1) = block_size_(1);
    }
    if(blockid(2) == block_num_(2) - 1){
        blocksize(2) = edgeblock_size_(2);
    }
    else{
        blocksize(2) = block_size_(2);
    }
    return blocksize;
}

inline int LowResMap::CheckNode(const Eigen::Vector3i pos){
    Eigen::Vector3d startpos = pos.cast<double>().cwiseProduct(node_scale_) + origin_;
    Eigen::Vector3d chk_pos;
    startpos(0) += resolution_/2;
    startpos(1) += resolution_/2;
    startpos(2) += resolution_/2;
    int state = 0;
    for(int x = 0; x < node_size_(0); x++)
        for(int y = 0; y < node_size_(1); y++)
            for(int z = 0; z < node_size_(2); z++){
        // volumetric_mapping::WorldBase::CellStatus cstatus = Octomap_->getCellStatusPoint(startpos+Eigen::Vector3d(x,y,z)*resolution_);
        chk_pos = startpos+Eigen::Vector3d(x,y,z)*resolution_;
        VoxelState cstatus = map_->GetVoxState(chk_pos);
        if(cstatus == VoxelState::occupied){
            return 1;
        }
        else if(cstatus == VoxelState::unknown){
            return 2;
        }
    }
    return 0;
}

inline void LowResMap::UpdateTie(const int &bid, const int &nid, const uint32_t &rid, bool new_h){
    shared_ptr<sch_node> s_n = gridBLK_[bid]->local_grid_[nid]->topo_sch_;
    shared_ptr<sch_node> n_n;

    bool new_tie = true;
    for(auto &tie : gridBLK_[bid]->local_grid_[nid]->ties_){
        if(rid == tie.root_id_){
            new_tie = false;
            // if( (!new_h && tie.dist_ < s_n->h_g_score_) || (tie.dist_ < s_n->g_score_ && new_h)){
                //clear old
                Eigen::Vector3i old_p_pos = s_n->pos_;
                double l_temp;
                if(TopoDirIter(tie.in_dir_, old_p_pos, l_temp)){
                    int parent_id = old_p_pos(2)*v_n_(1) + old_p_pos(1)*voxel_num_(0) + old_p_pos(0);
                    shared_ptr<LR_node> parent_n = GetNode(parent_id);
                    u_char opr = ~InverseDir(tie.in_dir_);
                    for(auto &p_tie : parent_n->ties_){
                        if(p_tie.root_id_ == rid){
                            p_tie.out_dir_ &= opr;
                            break;
                        }
                    }
                }

                //update new
                if(new_h) tie.dist_ = s_n->g_score_;
                else tie.dist_ = s_n->h_g_score_;

                if((s_n->h_parent_ == NULL && !new_h) || (s_n->parent_ == NULL && new_h)){
                   tie.in_dir_ = 128; 
                }
                else{
                    if(new_h) n_n = s_n->parent_;
                    else n_n = s_n->h_parent_;

                    Eigen::Vector3i dir = n_n->pos_ - s_n->pos_;
                    int parent_id = n_n->pos_(2)*v_n_(1) + n_n->pos_(1)*voxel_num_(0) + n_n->pos_(0);
                    shared_ptr<LR_node> parent_n = GetNode(parent_id);
                    u_char opr;

                    if(dir(0) == 1){
                        tie.in_dir_ = 32;
                        opr = 16;
                    }
                    else if(dir(0) == -1){
                        tie.in_dir_ = 16;
                        opr = 32;
                    }
                    else if(dir(1) == 1){
                        tie.in_dir_ = 8;
                        opr = 4;
                    }
                    else if(dir(1) == -1){
                        tie.in_dir_ = 4;
                        opr = 8;
                    }
                    else if(dir(2) == 1){
                        tie.in_dir_ = 2;
                        opr = 1;
                    }
                    else if(dir(2) == -1){
                        tie.in_dir_ = 1;
                        opr = 2;
                    }
                    else ROS_ERROR("error dir tie");

                    for(auto &p_tie: parent_n->ties_){
                        if(p_tie.root_id_ == rid){
                            p_tie.out_dir_ |= opr;
                            break;
                        }
                    }
                }
            // }
            break;
        }
    }
    if(new_tie){
        lr_root tie;
        if(new_h) tie.dist_ = s_n->g_score_;
        else tie.dist_ = s_n->h_g_score_;
        tie.root_id_ = rid;
        if((s_n->h_parent_ == NULL && !new_h) || (s_n->parent_ == NULL && new_h)){
            tie.in_dir_ = 128;
        }
        else{
            if(new_h) n_n = s_n->parent_;
            else n_n = s_n->h_parent_;
            Eigen::Vector3i dir = n_n->pos_ - s_n->pos_;
            int parent_id = n_n->pos_(2)*v_n_(1) + n_n->pos_(1)*voxel_num_(0) + n_n->pos_(0);
            shared_ptr<LR_node> parent_n = GetNode(parent_id);
            u_char opr = 255;

            if(dir(0) == 1){
                tie.in_dir_ = 32;
                opr = 16;
            }
            else if(dir(0) == -1){
                tie.in_dir_ = 16;
                opr = 32;
            }
            else if(dir(1) == 1){
                tie.in_dir_ = 8;
                opr = 4;
            }
            else if(dir(1) == -1){
                tie.in_dir_ = 4;
                opr = 8;
            }
            else if(dir(2) == 1){
                tie.in_dir_ = 2;
                opr = 1;
            }
            else if(dir(2) == -1){
                tie.in_dir_ = 1;
                opr = 2;
            }
            else ROS_ERROR("error dir tie2");

            for(auto &p_tie: parent_n->ties_){
                if(p_tie.root_id_ == rid){
                    p_tie.out_dir_ |= opr;
                    break;
                }
            }
        }
        gridBLK_[bid]->local_grid_[nid]->ties_.emplace_back(tie);
    }
    gridBLK_[bid]->local_grid_[nid]->topo_sch_ = NULL;
}

inline bool LowResMap::TopoDirIter(const u_char &dir, Eigen::Vector3i &idx, double &length){
    switch (dir)
    {
    case 1:{
        idx(2)--;
        length += node_scale_(2);
        break;
    }
    case 2:{
        idx(2)++;
        length += node_scale_(2);
        break;
    }
    case 4:{
        idx(1)--;
        length += node_scale_(1);
        break;
    }
    case 8:{
        idx(1)++;
        length += node_scale_(1);
        break;
    }
    case 16:{
        idx(0)--;
        length += node_scale_(0);
        break;
    }
    case 32:{
        idx(0)++;
        length += node_scale_(0);
        break;
    }
    default:
        return false;
    }
    return true;
}

inline void LowResMap::MultiTopoDirIter(const u_char &dir, const Eigen::Vector3i &idx, vector<Eigen::Vector3i> &neighbours){
    neighbours.clear();
    if(dir & 1) neighbours.push_back(idx + Eigen::Vector3i(0, 0, -1));
    if(dir & 2) neighbours.push_back(idx + Eigen::Vector3i(0, 0, 1));
    if(dir & 4) neighbours.push_back(idx + Eigen::Vector3i(0, -1, 0));
    if(dir & 8) neighbours.push_back(idx + Eigen::Vector3i(0, 1, 0));
    if(dir & 16) neighbours.push_back(idx + Eigen::Vector3i(-1, 0, 0));
    if(dir & 32) neighbours.push_back(idx + Eigen::Vector3i(1, 0, 0));
}

inline u_char LowResMap::InverseDir(const u_char &dir){
    switch (dir)
    {
    case 1:{
        return 2;
    }
    case 2:{
        return 1;
    }
    case 4:{
        return 8;
    }
    case 8:{
        return 4;
    }
    case 16:{
        return 32;
    }
    case 32:{
        return 16;
    }
    default: return 0;
    }
}

inline bool LowResMap::HaveLocalNeighbour(const Eigen::Vector3i &pos){
    Eigen::Vector3i npos = pos;
    int nodeid;
    shared_ptr<LR_node> node;
    for(int i = 0; i < 3; i++){
        npos(i) += 1;
        if(npos(i) <= local_up_idx_(i)){
            nodeid = npos(2)*v_n_(1)+npos(1)*voxel_num_(0) + npos(0);
            node = GetNode(nodeid);
            if(node != NULL && !node->flags_[0]){
                if(node->flags_[1]) return true;
            }
        }
        npos(i) -= 2;
        if(npos(i) >= local_low_idx_(i)){
            nodeid = npos(2)*v_n_(1)+npos(1)*voxel_num_(0) + npos(0);
            node = GetNode(nodeid);
            if(node != NULL && !node->flags_[0]){
                if(node->flags_[1]) return true;
            }
        }
        npos(i) += 1;
    }
    return false;
}

inline double LowResMap::GetHue(const Eigen::Vector3i &p1, const Eigen::Vector3i &p2){
    return (p1-p2).cast<double>().cwiseProduct(node_scale_).norm() * 1.000;
}

inline double LowResMap::GetDist(const int &dx, const int &dy, const int &dz){
    return sqrt(pow(dx*node_scale_(0), 2) + pow(dy*node_scale_(1), 2) + pow(dz*node_scale_(2), 2));
}

inline double LowResMap::GetHueL1(const Eigen::Vector3i &p1, const Eigen::Vector3i &p2){
    return (abs(p1(0) - p2(0))*node_scale_(0) + abs(p1(1) - p2(1))*node_scale_(1) + abs(p1(2) - p2(2))*node_scale_(2)) * 1.0001;
}

inline double LowResMap::GetDistL1(const Eigen::Vector3i &dp){
    return abs(dp(0))*node_scale_(0) + abs(dp(1))*node_scale_(1) + abs(dp(2))*node_scale_(2);
}

inline bool LowResMap::CheckAddHnode(const Eigen::Vector3d &pos, const int &id){
    shared_ptr<LR_node> node = GetNode(pos);
    if(node == NULL || node->flags_[0]) return false;
    for(auto &tie : node->ties_){
        if(tie.root_id_ == id) return true;
    }
    lr_root tie;
    tie.dist_ = 0;
    tie.root_id_ = id;
    tie.in_dir_ = 128;
    tie.out_dir_ = 0;
    node->ties_.emplace_back(tie);
    return true;
}

inline Eigen::Vector3d LowResMap::GetCenterPos(const Eigen::Vector3d &p){
    int id = PostoId(p);
    return IdtoPos(id);
}

inline bool LowResMap::StrangePoint(Eigen::Vector3d &p){
    Eigen::Vector3d dp = p - origin_;
    for(int dim = 0; dim < 3; dim++){
        if(abs(dp(dim) / node_scale_(dim) - floor(dp(dim) / node_scale_(dim)) - 0.5) > 0.5 - 1e-4){
            return true;
        }
    }
    return false;
}


}
#endif