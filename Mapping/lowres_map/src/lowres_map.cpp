#include <lowres_map/lowres_map.h>
using namespace std;
using namespace Eigen;

namespace lowres{

void LowResMap::init(const ros::NodeHandle &nh, 
        const ros::NodeHandle &nh_private){
    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    Eigen::Vector3d localgraph_scale;
    nh_private_.param(ns + "/LowResMap/localgraph_sizex", 
        localgraph_scale.x(), 14.0);
    nh_private_.param(ns + "/LowResMap/localgraph_sizey", 
        localgraph_scale.y(), 14.0);
    nh_private_.param(ns + "/LowResMap/localgraph_sizez", 
        localgraph_scale.z(), 8.0);
    nh_private_.param(ns + "/LowResMap/node_x", 
        node_scale_.x(), 0.5);
    nh_private_.param(ns + "/LowResMap/node_y", 
        node_scale_.y(), 0.5);
    nh_private_.param(ns + "/LowResMap/node_z", 
        node_scale_.z(), 0.3);
    nh_private_.param(ns + "/Exp/robot_sizeX", 
        Robot_size_.x(), 0.5);
    nh_private_.param(ns + "/Exp/robot_sizeY", 
        Robot_size_.y(), 0.5);
    nh_private_.param(ns + "/Exp/robot_sizeZ", 
        Robot_size_.z(), 0.3);
    nh_private_.param(ns + "/LowResMap/lambda_heu", 
        lambda_heu_, 1.5);
    nh_private_.param(ns + "/Exp/minX", 
        origin_.x(), -10.0);
    nh_private_.param(ns + "/Exp/minY", 
        origin_.y(), -10.0);
    nh_private_.param(ns + "/Exp/minZ", 
        origin_.z(), 0.0);
    nh_private_.param(ns + "/LowResMap/corridor_expX", 
        corridor_exp_r_.x(), 1);
    nh_private_.param(ns + "/LowResMap/corridor_expY", 
        corridor_exp_r_.y(), 1);
    nh_private_.param(ns + "/LowResMap/corridor_expZ", 
        corridor_exp_r_.z(), 1);
    nh_private_.param(ns + "/Exp/maxX", 
        mapscale_.x(), 10.0);
    nh_private_.param(ns + "/Exp/maxY", 
        mapscale_.y(), 10.0);
    nh_private_.param(ns + "/Exp/maxZ", 
        mapscale_.z(), 0.0);
    nh_private_.param(ns + "/LowResMap/blockX", 
        block_size_.x(), 5);
    nh_private_.param(ns + "/LowResMap/blockY", 
        block_size_.y(), 5);
    nh_private_.param(ns + "/LowResMap/blockZ", 
        block_size_.z(), 3);
    nh_private_.param(ns + "/LowResMap/resolution", 
        resolution_, 0.2);
    nh_private_.param(ns + "/LowResMap/showmap", 
        showmap_, false);
    nh_private_.param(ns + "/LowResMap/debug", 
        debug_, false);
    nh_private_.param(ns + "/LowResMap/seg_length", 
        seg_length_, 3.0);
    nh_private_.param(ns + "/LowResMap/prune_seg_length", 
        prune_seg_length_, 3.0);
    nh_private_.param(ns + "/LowResMap/show_dtg", 
        show_dtg_, false);
    nh_private_.param(ns + "/block_map/occ_duration", 
        occ_duration_, 1.0);
    nh_private_.param(ns + "/block_map/unknown_duration", 
        unknown_duration_, 0.5);
    nh_private.param(ns + "/Exp/drone_num", drone_num_, 1);

    //pub
    if(showmap_){
        node_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/LowResMap/Nodes", 10);
        show_timer_ = nh_private_.createTimer(ros::Duration(0.15), &LowResMap::ShowGridLocal, this);
    }
    if(debug_){
        debug_pub_ = nh_.advertise<visualization_msgs::Marker>(ns + "/LowResMap/debug", 10);
    }

    node_size_(0) = ceil(node_scale_(0) / resolution_) + 1;
    node_size_(1) = ceil(node_scale_(1) / resolution_) + 1;
    node_size_(2) = ceil(node_scale_(2) / resolution_) + 1;

    node_scale_ = node_size_.cast<double>() * resolution_;
    expand_r_ = Robot_size_ * 0.5;

    mapscale_.x() = ceil((mapscale_.x() - origin_.x())/node_scale_(0)) * node_scale_(0);
    mapscale_.y() = ceil((mapscale_.y() - origin_.y())/node_scale_(1)) * node_scale_(1);
    mapscale_.z() = ceil((mapscale_.z() - origin_.z())/node_scale_(2)) * node_scale_(2);

    double dx = origin_.x() - (floor((origin_.x())/node_scale_(0))) * node_scale_(0);
    double dy = origin_.y() - (floor((origin_.y())/node_scale_(1))) * node_scale_(1);
    double dz = origin_.z() - (floor((origin_.z())/node_scale_(2))) * node_scale_(2);

    origin_.x() -= dx;
    origin_.y() -= dy;
    origin_.z() -= dz;
    // mapscale_.x() += node_scale_(0);
    // mapscale_.y() += node_scale_(1);
    // mapscale_.z() += node_scale_(2);
    voxel_num_.x() = ceil((mapscale_.x()-1e-3)/node_scale_(0));
    voxel_num_.y() = ceil((mapscale_.y()-1e-3)/node_scale_(1));
    voxel_num_.z() = ceil((mapscale_.z()-1e-3)/node_scale_(2));
    v_n_.x() = voxel_num_.x();
    v_n_.y() = voxel_num_.y() * voxel_num_.x();
    v_n_.z() = voxel_num_.z() * voxel_num_.y() * voxel_num_.x();

    map_upbd_ = origin_+mapscale_ - Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Eigen::Vector3d(1e-4, 1e-4, 1e-4);



    block_num_.x() = ceil(double(voxel_num_.x()) / block_size_.x());
    block_num_.y() = ceil(double(voxel_num_.y()) / block_size_.y());
    block_num_.z() = ceil(double(voxel_num_.z()) / block_size_.z());
    b_n_.x() = block_num_.x();
    b_n_.y() = block_num_.y() * block_num_.x();
    b_n_.z() = block_num_.z() * block_num_.y() * block_num_.x();
    blockscale_.x() = node_scale_(0)*block_size_.x();
    blockscale_.y() = node_scale_(1)*block_size_.y();
    blockscale_.z() = node_scale_(2)*block_size_.z();

    edgeblock_size_.x() = voxel_num_.x() - floor(voxel_num_.x() / double(block_size_.x()))*block_size_.x();
    edgeblock_size_.y() = voxel_num_.y() - floor(voxel_num_.y() / double(block_size_.y()))*block_size_.y();
    edgeblock_size_.z() = voxel_num_.z() - floor(voxel_num_.z() / double(block_size_.z()))*block_size_.z();

    if(edgeblock_size_.x() == 0) edgeblock_size_.x() = block_size_.x();
    if(edgeblock_size_.y() == 0) edgeblock_size_.y() = block_size_.y();
    if(edgeblock_size_.z() == 0) edgeblock_size_.z() = block_size_.z();

    edgeblock_scale_.x() = node_scale_(0)*edgeblock_size_.x();
    edgeblock_scale_.y() = node_scale_(1)*edgeblock_size_.y();
    edgeblock_scale_.z() = node_scale_(2)*edgeblock_size_.z();
    gridBLK_.resize(block_num_.x()*block_num_.y()*block_num_.z());


    localgraph_size_(0) = ceil(localgraph_scale(0)/node_scale_(0));
    localgraph_size_(1) = ceil(localgraph_scale(1)/node_scale_(1));
    localgraph_size_(2) = ceil(localgraph_scale(2)/node_scale_(2));
    
    Inc_list_.resize(4);
    open_set_.resize(4);
    for(int i = 0; i < 4; i++) {
        Astar_worktable_.push_back(true);
    }
    
    Eternal_bid_ = -1;
    Eternal_nid_ = -1;
    workable_ = true;
    Outnode_ = make_shared<LR_node>();
    Expandnode_ = make_shared<LR_node>();
    cur_root_h_id_ = 0;
    // caster_.setParams()
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"map_lowbd_:"<<map_lowbd_.transpose()<<endl;
    cout<<"mapscale_:"<<mapscale_.transpose()<<endl;
    cout<<"blockscale_:"<<blockscale_.transpose()<<endl;
    cout<<"edgeblock_scale_:"<<edgeblock_scale_.transpose()<<endl;
    cout<<"block_size_:"<<block_size_.transpose()<<endl;
    cout<<"voxel_num_:"<<voxel_num_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;
    cout<<"node_size_:"<<node_size_.transpose()<<endl;
    cout<<"node_scale_:"<<node_scale_.transpose()<<endl;
    cout<<"localgraph_scale:"<<localgraph_scale.transpose()<<endl;
    cout<<"drone_num:"<<drone_num_<<endl;
    cout<<"localgraph_size_:"<<localgraph_size_.transpose()<<endl;
}

void LowResMap::UpdateLocalBBX(const Eigen::Matrix4d rob_pose, vector<Eigen::Vector3d> &occ_list){
    mtx_.lock();
    Robot_pose_ = rob_pose;
    ros::WallTime start = ros::WallTime::now();

    Robot_pos_ = Robot_pose_.block(0,3,3,1);
    Eigen::Vector3i robot_pid;
    PostoId3(Robot_pos_, robot_pid);
    for(int i = 0; i < 3; i++){
        local_up_idx_(i) = min(voxel_num_(i)-1, robot_pid(i)+int(ceil(localgraph_size_(i)/2)));
        local_low_idx_(i) = max(0, robot_pid(i)-int(ceil(localgraph_size_(i)/2)));
    }

    ClearInfeasible(occ_list);
    ExpandLocalMap();
    PruneBlock();

    vector<Eigen::Vector4d> pts;
    // Debug(pts);
    mtx_.unlock();
    if(showmap_){
        thread t1(&LowResMap::LoadShowList, this);
        t1.detach();
    }
}

bool LowResMap::UpdateLocalTopo(const Eigen::Matrix4d rob_pose, vector<Eigen::Vector3d> &occ_list, bool clear_x){
    mtx_.lock();
    Robot_pose_ = rob_pose;
    ros::WallTime start = ros::WallTime::now();
    Robot_pos_ = Robot_pose_.block(0,3,3,1);
    Eigen::Vector3i robot_pid;
    PostoId3(Robot_pos_, robot_pid);
    for(int i = 0; i < 3; i++){
        local_up_idx_(i) = min(voxel_num_(i)-1, robot_pid(i));
        local_low_idx_(i) = max(0, robot_pid(i));
    }
    if(clear_x) ClearXNodes();
    UpdateFOV();
    ClearInfeasibleTopo(occ_list);
    ExpandTopoMap();
    PruneTopoBlock();
    mtx_.unlock();
    if(showmap_){
        thread t1(&LowResMap::LoadShowList, this);
        t1.detach();
    }
    // cout<<"2 cur_root_h_id_:"<<cur_root_h_id_<<"  prep_idx_:"<<prep_idx_<<endl;
    // cout<<(prep_idx_ == cur_root_h_id_)<<endl;
    return prep_idx_ == cur_root_h_id_;
}

void LowResMap::ExpandLocalMap(){
    for(std::vector<int>::iterator lastit = localnode_list_.begin(); lastit != localnode_list_.end(); lastit++){
        shared_ptr<LR_node> node = GetNode(*lastit);
        if(node != NULL){
            node->flags_.reset(1);
            node->flags_.reset(2);
        }
    }
    H_Topolist_.clear();
    Topolist_.clear();
    localnode_list_.clear();
    Robot_pos_ = Robot_pose_.block(0,3,3,1);

    Rootnode_ = GetNode(Robot_pos_);
    int nodeid = PostoId(Robot_pos_);
    std::list<int> TSList, NTSList;
    if(Rootnode_ == NULL){
        NTSList.push_back(nodeid);
        std::cout <<Robot_pos_.transpose()<<"\033[0;34m New root \033[0m" << std::endl;
        SetEXPNode(Robot_pos_);
    }
    else if(Rootnode_ == Outnode_){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m Outside!!!!! \033[0m" << std::endl;
        return;
    }
    else if(Rootnode_->flags_[0]){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m collide!!!!! \033[0m" << std::endl;
        return;
    }
    else{
        TSList.push_back(nodeid);
    }
    Eigen::Vector3i node3i;
    Eigen::Vector3d pos, offset;
    offset = Eigen::Vector3d(0.5,0.5,0.5);
    double time = ros::Time::now().toSec();
    
    while((!NTSList.empty() || !TSList.empty())){
        while(!NTSList.empty()){
            node3i(0) = NTSList.front() % v_n_(0);
            node3i(1) = ((NTSList.front() - node3i(0))/v_n_(0)) % voxel_num_(1);
            node3i(2) = ((NTSList.front() - node3i(0)) - node3i(1)*voxel_num_(0))/v_n_(1);
            // cout<<"node3i:"<<node3i.transpose()<<";;;"<<NTSList.front()<<endl;
            pos = (node3i.cast<double>() + offset).cwiseProduct(node_scale_)+origin_;
            int node_status = CheckNode(node3i);
            if(node_status == 2){
                SetXNode(pos, time + unknown_duration_);
            }
            else if(node_status == 1){
                SetXNode(pos, time + occ_duration_);
            }
            else{
                GetExpandNeighbours(NTSList.front(), node3i, NTSList, TSList);
                SetNode(pos);
                localnode_list_.push_back(NTSList.front());    
            }
            NTSList.pop_front();
        }
        while(!TSList.empty()){
            GetExistNeighbours(TSList.front(), TSList, NTSList);
            TSList.pop_front();
        }
    }   
}

void LowResMap::ExpandTopoMap(){
    for(auto &lastit : localnode_list_){
        shared_ptr<LR_node> node = GetNode(lastit);
        if(node != NULL){
            node->topo_sch_ = NULL;
            node->flags_.reset(1);
        }
    }
    localnode_list_.clear();

    cur_root_h_id_ = 0;
    cur_h_cost_ = 999999;
    Topolist_.clear();
    H_Topolist_.clear();
    id_idx_dist_.clear();
    id_Hpos_dist_.clear();
    Eigen::Vector3i robot_pid;
    PostoId3(Robot_pos_, robot_pid);

    Rootnode_ = GetNode(Robot_pos_);
    shared_ptr<LR_node> lr_node;
    prio_D empty_TS, empty_NTS; 
    shared_ptr<sch_node> c_node, ep_node;

    open_TS_.swap(empty_TS);
    open_NTS_.swap(empty_NTS);
    c_node = make_shared<sch_node>();
    c_node->pos_ = robot_pid;

    if(Rootnode_ == NULL){
        open_NTS_.push(c_node);
        std::cout <<Robot_pos_.transpose()<<"\033[0;34m New root \033[0m" << std::endl;
        SetTopoNode(Robot_pos_, Rootnode_, c_node);
    }
    else if(Rootnode_ == Outnode_){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m Outside!!!!! \033[0m" << std::endl;
        return;
    }
    else if(Rootnode_->flags_[0]){
        std::cout <<Robot_pos_.transpose()<<"\033[0;31m collide!!!!! \033[0m" << std::endl;
        return;
    }
    else{
        Rootnode_->topo_sch_ = c_node;
        open_TS_.push(c_node);
    }
    Eigen::Vector3i node3i;
    Eigen::Vector3d pos, offset;
    offset = Eigen::Vector3d(0.5, 0.5, 0.5);
    double time = ros::Time::now().toSec();
    
    while((!open_NTS_.empty() || !open_TS_.empty())){
        while(!open_NTS_.empty()){
            ep_node = open_NTS_.top();
            open_NTS_.pop();
            if(ep_node->status_ == in_close) continue;
            ep_node->status_ = in_close;
            pos = (ep_node->pos_.cast<double>() + offset).cwiseProduct(node_scale_)+origin_;
            PostoId3(pos, node3i);
            int node_status = CheckNode(node3i);
            if(node_status == 2){
                SetUNode(pos, time + unknown_duration_);
            }
            else if(node_status == 1){
                SetXNode(pos, time + occ_duration_);
            }
            else {
                int id = ep_node->pos_(0) + ep_node->pos_(1) * v_n_(0) + ep_node->pos_(2) * v_n_(1);
                GetTopoNeighbours(ep_node);
                SetNode(pos, true);
                localnode_list_.push_back(id);
                for(int i = 0; i < 3; i++){
                    local_up_idx_(i) = max(ep_node->pos_(i), local_up_idx_(i));
                    local_low_idx_(i) = min(ep_node->pos_(i), local_low_idx_(i));
                }
            }
        }

        while(!open_TS_.empty()){
            ep_node = open_TS_.top();
            open_TS_.pop();
            if(ep_node->status_ == in_close) continue;
            ep_node->status_ = in_close;
            int id = ep_node->pos_(0) + ep_node->pos_(1) * v_n_(0) + ep_node->pos_(2) * v_n_(1);
            pos = (ep_node->pos_.cast<double>() + offset).cwiseProduct(node_scale_)+origin_;
            GetTopoNeighbours(ep_node);
            SetNode(pos, true);
            localnode_list_.push_back(id);
            for(int i = 0; i < 3; i++){
                local_up_idx_(i) = max(ep_node->pos_(i), local_up_idx_(i));
                local_low_idx_(i) = min(ep_node->pos_(i), local_low_idx_(i));
            }
        }
    }   
    local_up_bd_ = IdtoPos(local_up_idx_) + node_scale_ * 0.499;
    local_low_bd_ = IdtoPos(local_low_idx_) - node_scale_ * 0.499;
}


void LowResMap::SetEternalNode(const Eigen::Vector3d &pos){

    Eigen::Vector3i blockid;
    if(GetBlock3Id(pos, blockid)){
        Eternal_bid_ = blockid(2)*b_n_(1) + blockid(1)*block_num_(0) + blockid(0);
        if(gridBLK_[Eternal_bid_] != NULL){
            Eternal_nid_ = GetNodeId(pos, gridBLK_[Eternal_bid_]);
        }
        else{
            Eternal_bid_ = -1;
            Eternal_nid_ = -1;
        }
    }
    else{
        Eternal_bid_ = -1;
        Eternal_nid_ = -1;
    }
}

void LowResMap::ClearInfeasible(vector<Eigen::Vector3d> &occ_list){
    idx_tie_clear_.clear();
    double time = ros::Time::now().toSec();
    h_id_clear_.clear();
    for(vector<Eigen::Vector3d>::iterator ocit = occ_list.begin(); ocit != occ_list.end(); ocit++){
        SetXNode(*ocit, time + occ_duration_);
    }
}

void LowResMap::ClearInfeasibleTopo(vector<Eigen::Vector3d> &occ_list){
    list<pair<Eigen::Vector3i, lr_root>> idx_tie_clear_temp_;
    shared_ptr<LR_node> cn;
    Eigen::Vector3i idx;
    double length;
    u_char dir;
    idx_tie_clear_.clear();
    double time = ros::Time::now().toSec();
    h_id_clear_.clear();
    for(vector<Eigen::Vector3d>::iterator ocit = occ_list.begin(); ocit != occ_list.end(); ocit++){
        SetXNode(*ocit, time + occ_duration_);
    }
    idx_tie_clear_temp_ = idx_tie_clear_;
    //clear the outdirs of the root to the cleared nodes
    for(auto &it : idx_tie_clear_){
        idx = it.first;
        if(TopoDirIter(it.second.in_dir_, idx, length)){
            int node_id = idx(2)*v_n_(1) + idx(1)*voxel_num_(0) + idx(0);
            cn = GetNode(node_id);
            for(auto &tie : cn->ties_){
                if(tie.root_id_ == it.second.root_id_){
                    dir = ~InverseDir(it.second.in_dir_);
                    tie.out_dir_ &= dir;
                    break;
                }
            }
        }
    }

    vector<Eigen::Vector3i> neis;
    //clear the leaves of the cleared nodes
    for(auto &it : idx_tie_clear_temp_){
        MultiTopoDirIter(it.second.out_dir_, it.first, neis);
        int c_id = it.first(2)*v_n_(1) + it.first(1)*voxel_num_(0) + it.first(0);
        cn = GetNode(c_id);
        for(int dim = 0; dim < 3; dim++){
                local_up_idx_(dim) = max(it.first(dim), local_up_idx_(dim));
                local_low_idx_(dim) = min(it.first(dim), local_low_idx_(dim));
        }

        for(list<lr_root>::iterator tie_it = cn->ties_.begin(); tie_it != cn->ties_.end(); tie_it++){
            if(tie_it->root_id_ == it.second.root_id_){
                cn->ties_.erase(tie_it);
                break;
            }
        }
        for(auto &nei: neis){
            int n_id = nei(2)*v_n_(1) + nei(1)*voxel_num_(0) + nei(0);
            cn = GetNode(n_id);
            for(auto &tie : cn->ties_){
                if(tie.root_id_ == it.second.root_id_){
                    idx_tie_clear_temp_.push_back({nei, tie});
                    break;
                }
            }
        }
    }

    local_up_bd_ = IdtoPos(local_up_idx_);
    local_low_bd_ = IdtoPos(local_low_idx_);
}

void LowResMap::PruneBlock(){
    //clear Xnodes
    double time_now = ros::Time::now().toSec();
    for(list<pair<int, int>>::iterator x_it = Xlist_.begin(); x_it != Xlist_.end(); x_it++){
        if(gridBLK_[x_it->first] != NULL){
            if(gridBLK_[x_it->first]->local_grid_[x_it->second] != NULL){
                if(gridBLK_[x_it->first]->local_grid_[x_it->second]->flags_[0]){
                    if(time_now - gridBLK_[x_it->first]->local_grid_[x_it->second]->last_update_ > 0.0){
                        gridBLK_[x_it->first]->alive_num_--;
                        gridBLK_[x_it->first]->local_grid_[x_it->second] = NULL;
                        if(gridBLK_[x_it->first]->alive_num_ == 0) DeadBlockList_.push_back(x_it->first);
                        list<pair<int, int>>::iterator erase_it = x_it;
                        x_it--;
                        Xlist_.erase(erase_it);
                    }
                }
                else{//debug
                    ROS_ERROR("Error PruneBlock xnode");
                }
            }
        }
        else{
            list<pair<int, int>>::iterator erase_it = x_it;
            x_it--;
            Xlist_.erase(erase_it);
        }
    }
    //clear dead block
    for(vector<int>::iterator deadit = DeadBlockList_.begin(); deadit != DeadBlockList_.end(); deadit++){
        if(gridBLK_[*deadit] != NULL){
            if(gridBLK_[*deadit]->alive_num_ == 0){
                gridBLK_[*deadit] = NULL;
            }
            else{
                std::cout << "\033[0;35m Error PruneBlock1(), alive_num_ != 0! \033[0m"<<*deadit<< std::endl;
            }
        }
    }
    DeadBlockList_.clear();
}

void LowResMap::PruneTopoBlock(){
    //clear Xnodes
    double time_now = ros::Time::now().toSec();

    for(list<pair<int, int>>::iterator x_it = Xlist_.begin(); x_it != Xlist_.end(); x_it++){
        if(gridBLK_[x_it->first] != NULL){
            if(gridBLK_[x_it->first]->local_grid_[x_it->second] != NULL){
                //delete sch_node
                gridBLK_[x_it->first]->local_grid_[x_it->second]->topo_sch_ = NULL;
                if(gridBLK_[x_it->first]->local_grid_[x_it->second]->flags_[0]){
                    //delete expired xnode
                    if(time_now - gridBLK_[x_it->first]->local_grid_[x_it->second]->last_update_ > 0.0){
                        gridBLK_[x_it->first]->alive_num_--;
                        gridBLK_[x_it->first]->local_grid_[x_it->second] = NULL;
                        if(gridBLK_[x_it->first]->alive_num_ == 0) DeadBlockList_.push_back(x_it->first);
                        list<pair<int, int>>::iterator erase_it = x_it;
                        x_it--;
                        Xlist_.erase(erase_it);
                    }
                }
                else{//debug, error happens, doing
                    ROS_ERROR("Error PruneBlock xnode");
                    ros::shutdown();
                }
            }
            else{
                list<pair<int, int>>::iterator erase_it = x_it;
                x_it--;
                Xlist_.erase(erase_it);
            }
        }
        else{
            list<pair<int, int>>::iterator erase_it = x_it;
            x_it--;
            Xlist_.erase(erase_it);
        }
    }

    //get paths to frontier viewpoints
    frontier_path_.clear();
    // cout<<"frontier_list_:"<<frontier_list_.size()<<endl;
    for(auto &i_v : frontier_list_){
        shared_ptr<LR_node> n;
        shared_ptr<sch_node> best_vp;
        Eigen::Vector3d debug_pos;
        int best_v_id;
        double length = 99999.0;
        //search viewpoint with the shortest path length to the robot
        // ROS_WARN("PruneTopoBlock0");
        for(auto &vp : i_v.second){
            n = GetNode(vp.second);
            if(n != NULL  && n->topo_sch_ != NULL && n->topo_sch_->g_score_ < length){
                length = n->topo_sch_->g_score_;
                best_vp = n->topo_sch_;
                best_v_id = vp.first;
                debug_pos = vp.second;
            }
        }
        // ROS_WARN("PruneTopoBlock1");
        if(length < 99998.0){
            list<Eigen::Vector3d> path;
            RetrievePath(path, best_vp);
            reverse(path.begin(), path.end());
            path.emplace_back(IdtoPos(best_vp->pos_));
            if(!IsFeasible(debug_pos)) ROS_ERROR("error debug pos");
            frontier_path_.push_back({{i_v.first, length}, {best_v_id ,path}});
        }
        // ROS_WARN("PruneTopoBlock2");
    }
    frontier_list_.clear();

    //clear dead block
    for(vector<int>::iterator deadit = DeadBlockList_.begin(); deadit != DeadBlockList_.end(); deadit++){
        if(gridBLK_[*deadit] != NULL){
            if(gridBLK_[*deadit]->alive_num_ == 0){
                gridBLK_[*deadit] = NULL;
            }
            else{//debug
                std::cout << "\033[0;35m Error PruneBlock2(), alive_num_ != 0! \033[0m"<<*deadit<<"  "<<int(gridBLK_[*deadit]->alive_num_)<< std::endl;
                int al = 0;
                for(auto &n : gridBLK_[*deadit]->local_grid_){
                    if(n != NULL) {
                        cout<<n->flags_<<endl;
                        al++;
                    }
                }
                cout<<al<<endl;
                ros::shutdown();
            }
        }
    }
    DeadBlockList_.clear();

    // cout<<"cur_root_h_id_:"<<cur_root_h_id_<<"  prep_idx_:"<<prep_idx_<<endl;
    if(cur_root_h_id_ != 0) 
    {   
        DjkstraLocal(cur_root_h_idx_); 
        //debug djkstra
        for(auto &topo : H_Topolist_){
            UpdateTie(topo.first, topo.second, cur_root_h_id_, false);
        }
    }
    else{
        cur_root_h_id_ = prep_idx_;
        //update topological relationship
        for(auto &topo : Topolist_){
            UpdateTie(topo.first, topo.second, prep_idx_, true);
        }
    }

    for(auto &lastit : localnode_list_){
        shared_ptr<LR_node> node = GetNode(lastit);
        if(node != NULL){
            node->topo_sch_ = NULL;
            node->flags_.reset(2);
        }
    }

}  

void LowResMap::ClearXNodes(){
    double time_now = ros::Time::now().toSec();

    for(list<pair<int, int>>::iterator x_it = Xlist_.begin(); x_it != Xlist_.end(); x_it++){
        if(gridBLK_[x_it->first] != NULL){
            if(gridBLK_[x_it->first]->local_grid_[x_it->second] != NULL){
                //delete sch_node
                gridBLK_[x_it->first]->local_grid_[x_it->second]->topo_sch_ = NULL;
                if(gridBLK_[x_it->first]->local_grid_[x_it->second]->flags_[0]){
                    gridBLK_[x_it->first]->alive_num_--;
                    gridBLK_[x_it->first]->local_grid_[x_it->second] = NULL;
                    if(gridBLK_[x_it->first]->alive_num_ == 0) DeadBlockList_.push_back(x_it->first);
                    list<pair<int, int>>::iterator erase_it = x_it;
                    x_it--;
                    Xlist_.erase(erase_it);
                }
                else{//debug, error happens, doing
                    ROS_ERROR("Error PruneBlock xnode");
                    ros::shutdown();
                }
            }
            else{
                list<pair<int, int>>::iterator erase_it = x_it;
                x_it--;
                Xlist_.erase(erase_it);
            }
        }
        else{
            list<pair<int, int>>::iterator erase_it = x_it;
            x_it--;
            Xlist_.erase(erase_it);
        }
    }

    //clear dead block
    for(vector<int>::iterator deadit = DeadBlockList_.begin(); deadit != DeadBlockList_.end(); deadit++){
        if(gridBLK_[*deadit] != NULL){
            if(gridBLK_[*deadit]->alive_num_ == 0){
                gridBLK_[*deadit] = NULL;
            }
            else{//debug
                std::cout << "\033[0;35m Error PruneBlock2(), alive_num_ != 0! \033[0m"<<*deadit<<"  "<<int(gridBLK_[*deadit]->alive_num_)<< std::endl;
                int al = 0;
                for(auto &n : gridBLK_[*deadit]->local_grid_){
                    if(n != NULL) {
                        cout<<n->flags_<<endl;
                        al++;
                    }
                }
                cout<<al<<endl;
                ros::shutdown();
            }
        }
    }
    DeadBlockList_.clear();
}

bool LowResMap::PathCheck(list<Eigen::Vector3d> &path, bool allow_uknown){
    RayCaster rc;
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);

    if(path.size() <= 1) {
        ROS_ERROR("strange path! %ld", path.size());
        return false;
    }

    ps_it = path.begin();
    pe_it = path.begin();
    pe_it++;
    while(pe_it != path.end()){
        rc.setInput((*pe_it - origin_).cwiseProduct(inv_res), (*ps_it - origin_).cwiseProduct(inv_res));
        while(rc.step(ray_iter)){
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsFeasible(ray_iter, allow_uknown)) {
                return false;
            }
        }
        ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
        if(!IsFeasible(ray_iter, allow_uknown)) {
            return false;
        }
        ps_it++;
        pe_it++;
    }
    return true;
}

bool LowResMap::FindCorridors(const vector<Eigen::Vector3d> path, 
                vector<Eigen::MatrixX4d> &corridors, 
                vector<Eigen::Matrix3Xd> &corridorVs,
                vector<Eigen::Vector3d> &pruned_path,
                double prune_length){
    vector<Eigen::Vector3d> path_cast;
    Eigen::Vector3i cor_size, cor_sidx, cor_eidx;
    Eigen::Vector3d cor_start, cor_end;
    double cur_total_length = 0, cur_length = 0;
    bool newseg_flag = true;
    int end_idx = 0, start_idx = 0;
    pruned_path.clear();
    corridors.clear();
    corridorVs.clear(); 
    if(!path.empty()){
        path_cast.emplace_back(path.front());
        // path_cast.emplace_back(GetStdPos(path.front()));
    }
    RayCaster rc;

    for(auto pt : path){
        if(!InsideMap(pt)){
            ROS_ERROR("out!!!");
            return false;
        }
        int last_id = PostoId(path_cast.back());
        if(PostoId(pt) != last_id){
            Eigen::Vector3d rc_s, rc_e, it;
            rc_s = path_cast.back() - origin_;
            rc_e = pt - origin_;
            for(int dim = 0; dim < 3; dim ++){
                rc_s(dim) /= node_scale_(dim);
                rc_e(dim) /= node_scale_(dim);
            }

            rc.setInput(rc_s, rc_e);
            while (rc.step(it)) {
                it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
                int id = PostoId(it);
                if(id == last_id) continue;
                path_cast.emplace_back(it);
            }
            it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
            int id = PostoId(it);
            if(id != last_id) path_cast.emplace_back(it);
        }
    }
    path_cast.emplace_back(path.back());

    pruned_path.emplace_back(path[0]);
    // cout<<"pruned_path"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;

    while(1){
        if(newseg_flag){
            cor_size.setOnes();
            cur_length = 0;
            start_idx = end_idx;
            cor_start = path_cast[start_idx];
            cor_end = cor_start;
            // pruned_path.emplace_back(cor_start);
            // cout<<"pruned_path_n"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;
            PostoId3(cor_start, cor_sidx);
            cor_eidx = cor_sidx;
            newseg_flag = false;
        }

        end_idx++;
        cur_length += (path_cast[end_idx] - path_cast[end_idx - 1]).norm();
        cur_total_length += (path_cast[end_idx] - path_cast[end_idx - 1]).norm();

        bool success = ExpandPath(cor_sidx, cor_eidx, path_cast[end_idx]);


        if(!success || cur_length >= seg_length_ || end_idx + 1 >= path_cast.size() || start_idx == 0){
            
            if(end_idx == start_idx && cur_length < seg_length_){
                ROS_ERROR("error path");
                return false;
            }

            Eigen::Vector3d up_corner, down_corner;
            CoarseExpand(cor_sidx, cor_eidx);
            FineExpand(cor_sidx, cor_eidx, up_corner, down_corner);
            Eigen::MatrixX4d h(6, 4);
            Eigen::Matrix3Xd p(3, 8);
            h.setZero();
            p.setZero();
            for(int dim = 0; dim < 3; dim++){
                up_corner(dim) -= Robot_size_(dim) * 0.52;
                down_corner(dim) += Robot_size_(dim) * 0.52;
                if(up_corner(dim) <= down_corner(dim)) {
                    ROS_ERROR("narrow!!!");
                    return false;
                }
                h(dim*2, dim) = 1;
                h(dim*2, 3) = -up_corner(dim);
                h(dim*2 + 1, dim) = -1;
                h(dim*2 + 1, 3) = down_corner(dim);
            }

            for(int dim1 = 0; dim1 <= 1; dim1++){
                for(int dim2 = 0; dim2 <= 1; dim2++){
                    for(int dim3 = 0; dim3 <= 1; dim3++){
                        p(0, 4*dim3 + 2*dim2 + dim1) = dim1 ? down_corner(0) : up_corner(0);
                        p(1, 4*dim3 + 2*dim2 + dim1) = dim2 ? down_corner(1) : up_corner(1);
                        p(2, 4*dim3 + 2*dim2 + dim1) = dim3 ? down_corner(2) : up_corner(2);
                    }
                }
            }


            if(!success) end_idx--;
            corridors.emplace_back(h);
            corridorVs.emplace_back(p);
            pruned_path.emplace_back(path_cast[end_idx]);
            // cout<<"pruned_path_f"<<pruned_path.size()<<"  :"<<pruned_path.back().transpose()<<endl;
            newseg_flag = true;
            if(end_idx + 1 >= path_cast.size() || cur_total_length >= prune_length){
                break;
            }
        }
    }
    return true;
}

bool LowResMap::ExpandPath(Eigen::Vector3i &cor_start, Eigen::Vector3i &cor_end, const Eigen::Vector3d &pos){
    Eigen::Vector3i corp_idx, d_corsize, cor_start_temp, cor_end_temp, cor_it;
    cor_start_temp = cor_start;
    cor_end_temp = cor_end;
    PostoId3(pos, corp_idx);
    for(int i = 0; i < 3; i++){
        d_corsize = cor_end_temp - cor_start_temp + Eigen::Vector3i(1, 1, 1);
        cor_it = cor_start_temp;
        
        if(corp_idx(i) < cor_start_temp(i)){
            d_corsize(i) = cor_start_temp(i) - corp_idx(i);
            cor_it(i) = corp_idx(i);
            cor_start_temp(i) = corp_idx(i);
        }
        else if(corp_idx(i) > cor_end_temp(i)){
            d_corsize(i) = corp_idx(i) - cor_end_temp(i);
            cor_it(i) = corp_idx(i);
            cor_end_temp(i) = corp_idx(i);
        } 
        else{
            continue;
        }


        for(int x = 0; x < d_corsize(0); x++){
            for(int y = 0; y < d_corsize(1); y++){
                for(int z = 0; z < d_corsize(2); z++){
                    Eigen::Vector3i chk_idx = cor_it + Eigen::Vector3i(x, y, z);
                    if(!IsFeasible(chk_idx)) return false;
                }
            }
        }   
    }
    cor_start = cor_start_temp;
    cor_end = cor_end_temp;
    return true;
}

void LowResMap::CoarseExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end){
    vector<Eigen::Vector3i> corners(2);
    vector<bool> expand(6, true);
    vector<int> expanded(6, 0);
    vector<Eigen::Vector3d> debug_pts;
    corners[0] = coridx_end;
    corners[1] = coridx_start;
    ROS_WARN("CoarseExpand0");
    while(1){
        bool expandable = false;
        ROS_WARN("CoarseExpand0.1");

        for(int dim = 0; dim < 3; dim++){
            for(int dir = -1; dir <= 1; dir += 2){
                if(!expand[dim*2 + int((1-dir)/2)]) continue;
                Eigen::Vector3i corridor_scale = corners[0] - corners[1] + Eigen::Vector3i::Ones();
                Eigen::Vector3i chk_it1, chk_it2;
                Eigen::Vector3i start_p = corners[1];
                start_p(dim) = corners[int((1-dir)/2)](dim) + dir;

                if(!InsideMap(start_p)) expand[dim*2 + int((1-dir)/2)] = false;
                ROS_WARN("CoarseExpand0.112");

                for(chk_it1(0) = 0; chk_it1(0) < (dim == 0 ? 1 : corridor_scale(0)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(0)++){
                    for(chk_it1(1) = 0; chk_it1(1) < (dim == 1 ? 1 : corridor_scale(1)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(1)++){
                        for(chk_it1(2) = 0; chk_it1(2) < (dim == 2 ? 1 : corridor_scale(2)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(2)++){
                            ROS_WARN("CoarseExpand0.113");

                            chk_it2 = start_p + chk_it1;
                            if(!InsideMap(chk_it2)) ROS_ERROR("CoarseExpand");
                            ROS_WARN("CoarseExpand0.1134");//
                            Eigen::Vector3d p = IdtoPos(chk_it2);
                            int blockid = GetBlockId(p);
                            cout<<p.transpose()<<"--"<<chk_it2.transpose()<<" ;;; "<<blockid<<endl;

                            if(IsFeasible(chk_it2) != VoxelState::free){
                                ROS_WARN("CoarseExpand0.1135");
                                expand[dim*2 + int((1-dir)/2)] = false;
                            }
                            ROS_WARN("CoarseExpand0.114");
                        }
                    }
                }

                expandable |= expand[dim*2 + int((1-dir)/2)];

                if(expand[dim*2 + int((1-dir)/2)]){
                    corners[int((1-dir)/2)](dim) += dir;
                    expanded[dim*2 + int((1-dir)/2)] += 1;
                    expand[dim*2 + int((1-dir)/2)] = (expanded[dim*2 + int((1-dir)/2)] < corridor_exp_r_(dim)) ? 1 : 0;
                }

            }
        }
        
        if(!expandable) break;
    }
    coridx_end = corners[0];
    coridx_start = corners[1];
    ROS_WARN("CoarseExpand1");

}

void LowResMap::FineExpand(Eigen::Vector3i &coridx_start, Eigen::Vector3i &coridx_end,  Eigen::Vector3d &up_corner, Eigen::Vector3d &down_corner){
    vector<Eigen::Vector3d> corners(2);
    vector<bool> expand(6, true);
    vector<double> expanded(6, 0.0);
    corners[0] = (coridx_end.cast<double>() + Eigen::Vector3d::Ones()).cwiseProduct(node_scale_) + origin_ - Eigen::Vector3d::Ones() * 1e-4;
    corners[1] = (coridx_start.cast<double>()).cwiseProduct(node_scale_) + origin_ + Eigen::Vector3d::Ones() * 1e-4;

    while(1){
        bool expandable = false;
        for(int dim = 0; dim < 3; dim++){
            for(int dir = -1; dir <= 1; dir += 2){
                if(!expand[dim*2 + int((1-dir)/2)]) continue;
                Eigen::Vector3d corridor_scale = corners[0] - corners[1];
                Eigen::Vector3d corridor_center = (corners[0] + corners[1]) * 0.5;
                Eigen::Vector3d chk_it1, chk_it2;
                Eigen::Vector3d start_p = corners[1] + Eigen::Vector3d::Ones() * resolution_ * 0.5;
                start_p(dim) = corridor_center(dim) + dir * 0.5 * (corridor_scale(dim) + resolution_);
                if(!InsideMap(start_p)) expand[dim*2 + int((1-dir)/2)] = false;
                for(chk_it1(0) = 0.0; chk_it1(0) < (dim == 0 ? resolution_ : corridor_scale(0)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(0) += resolution_){
                    for(chk_it1(1) = 0.0; chk_it1(1) < (dim == 1 ? resolution_ : corridor_scale(1)) - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(1) += resolution_){
                        for(chk_it1(2) = 0.0; chk_it1(2) < (dim == 2 ? resolution_ : corridor_scale(2))  - 1e-4 && expand[dim*2 + int((1-dir)/2)]; chk_it1(2) += resolution_){
                            chk_it2 = start_p + chk_it1;
                            // debug_pts.push_back(chk_it2);
                            if(map_->GetVoxState(chk_it2) != VoxelState::free){
                                // ROS_ERROR("occ");
                                expand[dim*2 + int((1-dir)/2)] = false;
                            }
                        }
                    }
                }

                expandable |= expand[dim*2 + int((1-dir)/2)];
                if(expand[dim*2 + int((1-dir)/2)]){
                    corners[int((1-dir)/2)](dim) += resolution_ * dir;
                    expanded[dim*2 + int((1-dir)/2)] += resolution_;
                    expand[dim*2 + int((1-dir)/2)] = (expanded[dim*2 + int((1-dir)/2)] < expand_r_(dim) * 0.5 - 1e-4) ? 1 : 0;
                }
            }
        }
        if(!expandable) break;
    }
    up_corner = corners[0];
    down_corner = corners[1];
}

void LowResMap::UpdateFOV(){
    Eigen::Vector3d nor1, nor2, nor3, nor4, xdir; 
    double d1, d2, d3, d4;
    Eigen::Matrix3d rot = Robot_pose_.block(0, 0, 3, 3);
    xdir = Robot_pose_.block(0, 0, 3, 1);
    //up
    nor1 = rot * Eigen::Vector3d(cos(ver_up_dir_), 0, sin(ver_up_dir_));
    nor1 = (- nor1 + nor1.dot(xdir) * xdir).normalized();
    d1 = nor1.dot(Robot_pos_);
    FOV_ieqs_.block(0, 0, 3, 1) = -nor1;
    FOV_ieqs_(3, 0) = d1;
    //down
    nor2 = rot * Eigen::Vector3d(cos(ver_down_dir_), 0, -sin(ver_down_dir_));
    nor2 = (- nor2 + nor2.dot(xdir) * xdir).normalized();
    d2 = nor2.dot(Robot_pos_);
    FOV_ieqs_.block(0, 1, 3, 1) = -nor2;
    FOV_ieqs_(3, 1) = d2;
    //left
    nor3 = rot * Eigen::Vector3d(cos(hor_left_dir_), sin(hor_left_dir_), 0);
    nor3 = (- nor3 + nor3.dot(xdir) * xdir).normalized();
    d3 = nor3.dot(Robot_pos_);
    FOV_ieqs_.block(0, 2, 3, 1) = -nor3;
    FOV_ieqs_(3, 2) = d3;
    //right
    nor4 = rot * Eigen::Vector3d(cos(hor_right_dir_), -sin(hor_right_dir_), 0);
    nor4 = (- nor4 + nor4.dot(xdir) * xdir).normalized();
    d4 = nor4.dot(Robot_pos_);
    FOV_ieqs_.block(0, 3, 3, 1) = -nor4;
    FOV_ieqs_(3, 3) = d4;
}

bool LowResMap::PrunePath(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length){
    length = 0;
    if(path.size() == 0) return false;
    else if(path.size() == 1){
        pruned_path = path;
        return true;
    }
    pruned_path.clear();
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    Eigen::Vector3i p_i;
    list<Eigen::Vector3d> std_off_path;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    bool free_ray;
    RayCaster rc;

    for(auto &p : path){
        PostoId3(p, p_i);
        std_off_path.push_back(IdtoPos(p_i) + Eigen::Vector3d::Ones() * 1e-4);
    }

    pruned_path.push_back(std_off_path.front());
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    for(list<Eigen::Vector3d>::iterator ps_it = std_off_path.begin(); pe_it != std_off_path.end(); pe_it++){
        pe_it = ps_it;
        double seg_length = 0;
        for(list<Eigen::Vector3d>::iterator pf_it = pe_it; pe_it != std_off_path.end() && seg_length < prune_seg_length_; pe_it++) {
            seg_length += ((*pf_it) - (*pe_it)).norm();
            pf_it = pe_it;
        }
        if(pe_it != std_off_path.end()) pe_it--;
        pe_it--;

        while (1)
        {
            if(ps_it == pe_it) return false;
            free_ray = true;
            rc.setInput((*ps_it - origin_).cwiseProduct(inv_res), (*pe_it - origin_).cwiseProduct(inv_res));
            while(rc.step(ray_iter)){
                ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
                if(!IsFeasible(ray_iter)) {
                    free_ray = false;
                    break;
                }
            }
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsFeasible(ray_iter)) {
                free_ray = false;
            }

            if(free_ray) {
                length += (pruned_path.back() - (*pe_it)).norm();
                pruned_path.push_back(*pe_it);
                break;
            }
            pe_it--;
        }
        ps_it = pe_it;
    }
    length += (pruned_path.back() - path.back()).norm();
    length += (pruned_path.front() - path.front ()).norm();
    pruned_path.push_back(path.back());
    pruned_path.push_front(path.front());
    return true;
}

void LowResMap::GetSafePath(list<Eigen::Vector3d> &danger_path, list<Eigen::Vector3d> &safe_path, 
            list<Eigen::Vector3d> &unknown_path, const double &max_dist, const double &unknown_dist){
    safe_path.clear();
    double dist = 0;
    double u_dist = 0;
    if(!danger_path.empty()){
        safe_path.emplace_back(danger_path.front());
    }
    else{
        ROS_ERROR("empty/infeasible path");
        return;
    }
    RayCaster rc;
    for(auto pt : danger_path){
        if(!InsideMap(pt)){
            ROS_ERROR("GetSafePath out!!!");
            return;
        }
        if((pt - safe_path.back()).norm() > 1e-6){
            Eigen::Vector3d rc_s, rc_e, it;
            rc_s = safe_path.back() - origin_;
            rc_e = pt - origin_;

            for(int dim = 0; dim < 3; dim ++){
                rc_s(dim) /= node_scale_(dim);
                rc_e(dim) /= node_scale_(dim);
            }

            rc.setInput(rc_s, rc_e);
            while (rc.step(it)) {
                it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
                dist += (it - safe_path.back()).norm();
                if(dist < 1e-6) continue;
                if(IsFeasible(it) && dist < max_dist){
                    safe_path.emplace_back(it);
                    // cout<<"dang push1:"<<it.transpose()<<endl;
                }
                else{
                    unknown_path.emplace_back(it);
                    // cout<<"dang unknown_path1:"<<it.transpose()<<endl;
                    return;
                }
            }
            it = it.cwiseProduct(node_scale_) + origin_ + node_scale_/2;
            dist += (it - safe_path.back()).norm();
            if(IsFeasible(it) && dist < max_dist){
                safe_path.emplace_back(it);
                // cout<<"dang push2:"<<it.transpose()<<endl;
            }
            else{
                unknown_path.emplace_back(it);
                // cout<<"dang unknown_path2:"<<it.transpose()<<endl;
                return;
            }
        }
    }
}

bool LowResMap::PrunePathTempt(const list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &pruned_path, double &length){
    length = 0;
    if(path.size() == 0) return false;
    else if(path.size() == 1){
        pruned_path = path;
        return true;
    }
    pruned_path.clear();
    Eigen::Vector3d inv_res, ray_iter;
    Eigen::Vector3d half_res = 0.5 * node_scale_;
    Eigen::Vector3i p_i;
    list<Eigen::Vector3d> std_off_path;
    list<Eigen::Vector3d>::iterator ps_it, pe_it;
    bool free_ray;
    RayCaster rc;

    for(auto &p : path){
        PostoId3(p, p_i);
        std_off_path.push_back(IdtoPos(p_i) + Eigen::Vector3d::Ones() * 1e-4);
    }

    pruned_path.push_back(std_off_path.front());
    for(int dim = 0; dim < 3; dim++) inv_res(dim) = 1.0 / node_scale_(dim);
    for(list<Eigen::Vector3d>::iterator ps_it = std_off_path.begin(); pe_it != std_off_path.end(); pe_it++){
        pe_it = ps_it;
        double seg_length = 0;
        for(list<Eigen::Vector3d>::iterator pf_it = pe_it; pe_it != std_off_path.end() && seg_length < prune_seg_length_; pe_it++) {
            seg_length += ((*pf_it) - (*pe_it)).norm();
            pf_it = pe_it;
        }
        if(pe_it != std_off_path.end()) pe_it--;
        pe_it--;

        while (1)
        {
            if(ps_it == pe_it) return false;
            free_ray = true;
            rc.setInput((*ps_it - origin_).cwiseProduct(inv_res), (*pe_it - origin_).cwiseProduct(inv_res));
            while(rc.step(ray_iter)){
                ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
                if(!IsLocalFeasible(ray_iter)) {
                    free_ray = false;
                    break;
                }
            }
            ray_iter = ray_iter.cwiseProduct(node_scale_) + origin_ + half_res;
            if(!IsLocalFeasible(ray_iter)) {
                free_ray = false;
            }

            if(free_ray) {
                length += (pruned_path.back() - (*pe_it)).norm();
                pruned_path.push_back(*pe_it);
                break;
            }
            pe_it--;
        }
        ps_it = pe_it;
    }
    length += (pruned_path.back() - path.back()).norm();
    length += (pruned_path.front() - path.front ()).norm();
    pruned_path.push_back(path.back());
    pruned_path.push_front(path.front());
    return true;
}


void LowResMap::LoadShowList(){
    mtx_.lock();
    Eigen::Vector3i startblck, endblck, searchidx;
    bool add;
    startblck(0) = max(0.0, floor(local_low_idx_(0)/block_size_(0))-1);
    startblck(1) = max(0.0, floor(local_low_idx_(1)/block_size_(1))-1);
    startblck(2) = max(0.0, floor(local_low_idx_(2)/block_size_(2))-1);
    endblck(0) = min(block_num_(0)-1.0, ceil(local_up_idx_(0)/block_size_(0))+1);
    endblck(1) = min(block_num_(1)-1.0, ceil(local_up_idx_(1)/block_size_(1))+1);
    endblck(2) = min(block_num_(2)-1.0, ceil(local_up_idx_(2)/block_size_(2))+1);

    for(searchidx(0) = startblck(0); searchidx(0) <= endblck(0); searchidx(0)+=1){
        for(searchidx(1) = startblck(1); searchidx(1) <= endblck(1); searchidx(1)+=1){
            for(searchidx(2) = startblck(2); searchidx(2) <= endblck(2); searchidx(2)+=1){
                int idx = GetBlockId(searchidx);
                if(idx != -1){
                    add = true;
                    for(list<int>::iterator slit = Showblocklist_.begin(); slit != Showblocklist_.end(); slit++){
                        if(*slit == idx){
                            add = false;
                            break;
                        }
                    }
                    if(add){
                        Showblocklist_.push_front(idx);
                    }
                }
            }
        }
    }
    mtx_.unlock();
}

void LowResMap::ShowGridLocal(const ros::TimerEvent& e){
    mtx_.lock();
    if(Showblocklist_.size() > 0 && node_pub_.getNumSubscribers() > 0){
        visualization_msgs::MarkerArray MKArray;
        MKArray.markers.resize(Showblocklist_.size()*2);
        //load makers
        int i = 0;
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            MKArray.markers[i].action = visualization_msgs::Marker::ADD;
            MKArray.markers[i].pose.orientation.w = 1.0;
            MKArray.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;      //nodes
            MKArray.markers[i].scale.x = resolution_/2;
            MKArray.markers[i].scale.y = resolution_/2;
            MKArray.markers[i].scale.z = resolution_/2;
            MKArray.markers[i].header.frame_id = "world";
            MKArray.markers[i].header.stamp = ros::Time::now();
            MKArray.markers[i].id = (*idit)*2;
            i++;
            MKArray.markers[i] = MKArray.markers[i-1];
            MKArray.markers[i].id++;
            MKArray.markers[i].type = visualization_msgs::Marker::CUBE_LIST;        //Xnodes

            MKArray.markers[i].scale.x *= 2;
            MKArray.markers[i].scale.y *= 2;
            MKArray.markers[i].scale.z *= 2;
            // MKArray.markers[i].color.r = 0.2;
            // MKArray.markers[i].color.g = 0.25;
            // MKArray.markers[i].color.b = 0.65;
            // MKArray.markers[i].color.a = 0.5;
            i++;
        }
        i = 0;
        Eigen::Vector3d pos;
        Eigen::Vector3i iterp;
        geometry_msgs::Point pt;
        int nodeid;
        std_msgs::ColorRGBA localcolor, globalcolor, Xcolor, Ecolor;
        Ecolor.a = 1.0;
        Ecolor.g = 1.0;

        Xcolor.a = 1.0;
        Xcolor.r = 1.0;

        localcolor.a = 1.0;
        localcolor.b = 0.3;
        localcolor.g = 0.5;
        localcolor.r = 0.8;
        globalcolor.a = 1.0;
        globalcolor.b = 0.8;
        globalcolor.g = 0.5;
        globalcolor.r = 0.3;
        //publish nodes
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            if(gridBLK_[*idit] == NULL){
                MKArray.markers[i].color.a = 0.2;
                MKArray.markers[i].type = visualization_msgs::Marker::CUBE;
                MKArray.markers[i].action = visualization_msgs::Marker::DELETE;
            }
            else{
                //clear flag
                // gridBLK_[*idit]->show_flag_ = false;
                //load points in block
                for(iterp(0) = 0; iterp(0) < gridBLK_[*idit]->block_size_(0); iterp(0)++){
                    for(iterp(1) = 0; iterp(1) < gridBLK_[*idit]->block_size_(1); iterp(1)++){
                        for(iterp(2) = 0; iterp(2) < gridBLK_[*idit]->block_size_(2); iterp(2)++){
                            nodeid = iterp(2)*gridBLK_[*idit]->block_size_(0)*gridBLK_[*idit]->block_size_(1)+
                                iterp(1)*gridBLK_[*idit]->block_size_(0) + iterp(0);
                            
                            if(gridBLK_[*idit]->local_grid_[nodeid] != NULL){
                                pt.x = (iterp(0)+gridBLK_[*idit]->origin_(0)+0.5)*node_scale_(0)+origin_(0);
                                pt.y = (iterp(1)+gridBLK_[*idit]->origin_(1)+0.5)*node_scale_(1)+origin_(1);
                                pt.z = (iterp(2)+gridBLK_[*idit]->origin_(2)+0.5)*node_scale_(2)+origin_(2);
                                // if(gridBLK_[*idit]->local_grid_[nodeid] == Expandnode_){//debug
                                //     MKArray.markers[i].colors.push_back(Ecolor);
                                //     MKArray.markers[i].points.push_back(pt);
                                // }
                                // else if(gridBLK_[*idit]->local_grid_[nodeid]->flags_[0] && gridBLK_[*idit]->local_grid_[nodeid]->flags_[3]){//debug
                                //     MKArray.markers[i+1].points.push_back(pt);
                                //     MKArray.markers[i+1].colors.push_back(Ecolor);
                                // }
                                // else if(gridBLK_[*idit]->local_grid_[nodeid]->flags_[0]){//debug
                                //     MKArray.markers[i+1].points.push_back(pt);
                                //     MKArray.markers[i+1].colors.push_back(Xcolor);
                                // }
                                // else 
                                if(!gridBLK_[*idit]->local_grid_[nodeid]->flags_[0]){
                                    MKArray.markers[i].colors.push_back(localcolor);
                                    MKArray.markers[i].points.push_back(pt);
                                }
                                // else{
                                //     MKArray.markers[i].colors.push_back(globalcolor);
                                //     MKArray.markers[i].points.push_back(pt);
                                // }
                                i++;
                                if(show_dtg_){
                                    pt.y -= resolution_ * int(gridBLK_[*idit]->local_grid_[nodeid]->ties_.size()); 
                                    pt.z -= resolution_; 
                                    for(auto &tie : gridBLK_[*idit]->local_grid_[nodeid]->ties_){
                                        pt.y += resolution_ * 2;
                                        MKArray.markers[i].colors.push_back(CM_->Id2Color(int(tie.root_id_) % drone_num_, 0.5));
                                        MKArray.markers[i].points.push_back(pt);
                                    }
                                }
                                i--;
                            }
                        }
                    }
                }
            }
            i+=2;
        }        
        //publish FOV, to do
        i = 0;
        for(list<int>::iterator idit = Showblocklist_.begin(); idit != Showblocklist_.end(); idit++){
            if(MKArray.markers[i].points.size() == 0){
                MKArray.markers[i].color.a = 0.2;
                MKArray.markers[i].type = visualization_msgs::Marker::CUBE;
                MKArray.markers[i].action = visualization_msgs::Marker::DELETE;
            }
            if(MKArray.markers[i+1].points.size() == 0){
                MKArray.markers[i+1].color.a = 0.2;
                MKArray.markers[i+1].type = visualization_msgs::Marker::CUBE;
                MKArray.markers[i+1].action = visualization_msgs::Marker::DELETE;
            }
            i+=2;
        }  
        Showblocklist_.clear();
        node_pub_.publish(MKArray);
    }
    // if(debug_){
    //     vector<Eigen::Vector4d> pts;
    //     Debug(pts);
    // }
    mtx_.unlock();
}

void LowResMap::DebugTie(const Eigen::Vector3d &p){
    shared_ptr<LR_node> node = GetNode(p);
    cout<<"debug p:"<<p.transpose()<<endl;
    if(node == NULL){
        ROS_WARN("NULL TIE");
        return;
    }
    else{
        bool success = false;
        for(auto & tie : node->ties_){
            if(tie.in_dir_ == 128) success = true;
            cout<<"tie id:"<<int(tie.root_id_)<<" in:"<<int(tie.in_dir_)<<" out:"<<int(tie.out_dir_)<<endl;
        }
        // if(!success){
        //     ROS_ERROR("debug error");
        //     ros::shutdown();
        // }
    }
}

void LowResMap::Debug(vector<Eigen::Vector4d> &pts){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::CUBE_LIST;      //nodes
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.color.a = 0.7;
    mk.color.g = 1.0;
    for(auto & blk : gridBLK_){
        if(blk != NULL){
            for(int x = 0; x < blk->block_size_(0); x++)
                for(int y = 0; y < blk->block_size_(1); y++)
                    for(int z = 0; z < blk->block_size_(2); z++){
                shared_ptr<LR_node> node = blk->local_grid_[z*blk->block_size_(0)*blk->block_size_(1) + y*blk->block_size_(0) + x];
                if(node != NULL && node->flags_[0]){
                    geometry_msgs::Point pt;
                    pt.x = blk->origin_(0) * node_scale_(0) + node_scale_(0) * x + node_scale_(0) / 2 + origin_(0);
                    pt.y = blk->origin_(1) * node_scale_(1) + node_scale_(1) * y + node_scale_(1) / 2 + origin_(1);
                    pt.z = blk->origin_(2) * node_scale_(2) + node_scale_(2) * z + node_scale_(2) / 2 + origin_(2);
                    mk.points.emplace_back(pt);
                }
            }
        }
    }
    debug_pub_.publish(mk);

}

void LowResMap::Debug(vector<Eigen::Vector3d> &pts){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;      //nodes
    mk.scale.x = 0.15;
    mk.scale.y = 0.15;
    mk.scale.z = 0.15;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.color.a = 0.7;
    mk.color.r = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for(int i = 0; i < pts.size(); i++){
        pt.x = pts[i](0);
        pt.y = pts[i](1);
        pt.z = pts[i](2);
        mk.points.push_back(pt);
    }
    debug_pub_.publish(mk);
}

void LowResMap::Debug2(vector<Eigen::Vector3d> &pts){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;      //nodes
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -1;
    mk.color.a = 0.6;
    mk.color.b = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for(int i = 0; i < pts.size(); i++){
        pt.x = pts[i](0) ;
        pt.y = pts[i](1) ;
        pt.z = pts[i](2) ;
        mk.points.push_back(pt);
    }
    debug_pub_.publish(mk);
}


void LowResMap::Debug2(list<Eigen::Vector3d> &pts){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::POINTS;      //nodes
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 1;
    mk.color.a = 0.6;
    mk.color.b = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for(auto &p : pts){
        pt.x = p(0) ;
        pt.y = p(1) ;
        pt.z = p(2) ;
        mk.points.push_back(pt);
    }
    debug_pub_.publish(mk);
}

void LowResMap::Debug(const uint32_t &id){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::LINE_LIST;      //nodes
    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.color.a = 0.7;
    mk.color.g = 1.0;

    geometry_msgs::Point pt;
    int have_tie = 0;
    for(auto &bk : gridBLK_){
        if(bk == NULL) continue;
        Eigen::Vector3i it = bk->origin_;
        Eigen::Vector3i pt_s, pt_e;
        double l;
        for(it(0) = bk->origin_(0); it(0) < bk->block_size_(0) + bk->origin_(0); it(0)++)
            for(it(1) = bk->origin_(1); it(1) < bk->block_size_(1) + bk->origin_(1); it(1)++)
                for(it(2) = bk->origin_(2); it(2) < bk->block_size_(2) + bk->origin_(2); it(2)++){
            int l_id = GetNodeId(it, bk);
            if(bk->local_grid_[l_id] == NULL) continue;
            // if(!bk->local_grid_[l_id]->flags_[0]) have_tie++;
            for(auto &tie : bk->local_grid_[l_id]->ties_){
                if(tie.root_id_ == id){
                    pt_s = it;
                    pt_e = it;
                    if(TopoDirIter(tie.in_dir_, pt_e, l)){
                        pt.x = (pt_s(0)+0.5) * node_scale_(0) + origin_(0);
                        pt.y = (pt_s(1)+0.5) * node_scale_(1) + origin_(1);
                        pt.z = (pt_s(2)+0.5) * node_scale_(2) + origin_(2);
                        mk.points.emplace_back(pt);
                        pt.x = (pt_e(0)+0.5) * node_scale_(0) + origin_(0);
                        pt.y = (pt_e(1)+0.5) * node_scale_(1) + origin_(1);
                        pt.z = (pt_e(2)+0.5) * node_scale_(2) + origin_(2);
                        mk.points.emplace_back(pt);
                    }
                    break;
                }
            }
        }
    }
    debug_pub_.publish(mk);
}


void LowResMap::Debug2(list<list<Eigen::Vector3d>> &paths){
    visualization_msgs::Marker mk;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.type = visualization_msgs::Marker::LINE_LIST;      //nodes
    mk.scale.x = 0.02;
    mk.scale.y = 0.02;
    mk.scale.z = 0.02;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -1;
    mk.color.a = 0.3;
    mk.color.b = 1.0;
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for(auto &path : paths){
        vector<Eigen::Vector3d> pth;
        if(path.size() <= 1) continue;
        for(auto &p : path) pth.emplace_back(p);
        for(int i = 1; i < pth.size(); i++){
            pt.x = pth[i-1](0);
            pt.y = pth[i-1](1);
            pt.z = pth[i-1](2);
            mk.points.emplace_back(pt);
            pt.x = pth[i](0);
            pt.y = pth[i](1);
            pt.z = pth[i](2);
            mk.points.emplace_back(pt);
        }
    }
    debug_pub_.publish(mk);
}
}
