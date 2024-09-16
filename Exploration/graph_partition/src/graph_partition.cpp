#include <graph_partition/graph_partition.h>
using namespace DTG;

void GraphVoronoiPartition::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){

    double par_freq;
    std::string ns = ros::this_node::getName();
    nh_private.param(ns + "/GVD/show_gvd", show_gvd_, false);
    nh_private.param(ns + "/GVD/lambda", lambda_, 0.2);
    nh_private.param(ns + "/GVD/allowance", allowance_, 0.1);
    
    nh_private.param(ns + "/GVD/tau", tau_, 0.3);
    nh_private.param(ns + "/GVD/partition_frequency", par_freq, 0.3);

    debug_pub_ = nh.advertise<visualization_msgs::Marker>(ns + "/GVD/Debug", 5);
    if(SDM_->is_ground_){
        show_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/GVD/GraphPartition", 5);
    }

    job_timer_ = nh.createTimer(ros::Duration(0.25), &GraphVoronoiPartition::JobTimerCallback, this);
    state_timer_ = nh.createTimer(ros::Duration(0.1), &GraphVoronoiPartition::StateTimerCallback, this);
    GVP_timer_ = nh.createTimer(ros::Duration(1.0 / par_freq), &GraphVoronoiPartition::PartitionTimerCallback, this);

    last_new_job_.resize(SDM_->drone_num_, ros::WallTime::now().toSec() - 1000.0);
    swarm_job_f_dist_.resize(SDM_->drone_num_, 9999.0);
    swarm_job_f_.resize(SDM_->drone_num_, -1);
    swarm_job_h_dist_.resize(SDM_->drone_num_, 9999.0);
    swarm_job_h_.resize(SDM_->drone_num_, -1);


    swarm_part_f_dist_.resize(SDM_->drone_num_);
    swarm_part_f_id_.resize(SDM_->drone_num_);
    swarm_connect_f_dist_.resize(SDM_->drone_num_);
    swarm_connect_f_id_.resize(SDM_->drone_num_);
    swarm_connect_h_dist_.resize(SDM_->drone_num_);
    swarm_connect_h_.resize(SDM_->drone_num_);
    last_state_pub_t_ = ros::WallTime::now().toSec() - 100.0;
}

void GraphVoronoiPartition::LocalGVP(){
    list<f_ptr> partition_fn;
    list<h_ptr> partition_hn;// = MDTG_->local_h_list_;
    list<h_ptr> fake_hn_s;
    LoadLocalData(partition_hn, partition_fn, fake_hn_s);

    /* GraphVoronoiPartition */
    prio_D pr_l;
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    h_ptr cur_hn, exp_hn;
    f_ptr exp_fn;
    for(auto &hns : fake_hn_s){
        hns->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, hns->id_, hns->pos_);
        cur_n = hns->sch_node_;
        cur_n->flag_ = 1;
        cur_n->id_ = hns->id_;
        cur_n->root_id_ = hns->id_;
        pr_l.push(cur_n);
    }

    while(!pr_l.empty()){
        cur_n = pr_l.top();
        pr_l.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        
        if((cur_n->flag_ & 1) && cur_n->id_ <= SDM_->drone_num_){   // fake start hn
            for(auto &hn : fake_hn_s){
                if(hn->id_ == cur_n->id_){
                    cur_hn = hn;
                    break;
                }
            }
        }
        else if(cur_n->flag_ & 1){                                  // mid hn
            if(!MDTG_->FindHnode(cur_n->pos_, cur_n->id_, cur_hn)){
                ROS_ERROR("error get hnode GetClosestGlobalTarget1");
                ros::shutdown();
                return;
            }
        }
        if(!(cur_n->flag_ & 1)) continue;

        /* expand Hneighbours */
        for(auto &hhe : cur_hn->hh_edges_){
            if(!(hhe->e_flag_ & 8)) continue;

            if(hhe->head_n_ == cur_hn)
                exp_hn = hhe->tail_n_;
            else
                exp_hn = hhe->head_n_;
            if(!(exp_hn->h_flags_ & 2)) continue;
            if(exp_hn->sch_node_ == NULL){
                /* new node */
                exp_hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_s_, 0.0, exp_hn->id_, exp_hn->pos_);
                nei_n = exp_hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->root_id_ = cur_n->root_id_;
                nei_n->flag_ = 1;
                pr_l.push(nei_n);
            }
            else{
                /* in close list */
                if(exp_hn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = exp_hn->sch_node_;
                if(cur_n->g_ + hhe->length_s_ + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    exp_hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_s_, 0.0, exp_hn->id_, exp_hn->pos_);
                    exp_hn->sch_node_->parent_ = cur_n;
                    exp_hn->sch_node_->root_id_ = cur_n->root_id_;
                    exp_hn->sch_node_->flag_ = 1;
                    pr_l.push(exp_hn->sch_node_);
                }
            }
        }

        /* expand Fneighbours */
        for(auto &hfe : cur_hn->hf_edges_){
            if(!(hfe->e_flag_ & 24)) continue;
            exp_fn = hfe->tail_n_;            

            if(exp_fn->sch_node_ == NULL){
                /* new node */
                exp_fn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hfe->length_, 0.0, exp_fn->id_, exp_fn->center_);
                nei_n = exp_fn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->root_id_ = cur_n->root_id_;
                nei_n->id_ = exp_fn->id_;
                nei_n->g_ = cur_n->g_ + hfe->length_;
                nei_n->flag_ = 0;
                pr_l.push(nei_n);
            }
            else{
                /* in close list */
                if(exp_fn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = exp_fn->sch_node_;
                if(cur_n->g_ + hfe->length_ + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    exp_fn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hfe->length_, 0.0, exp_fn->id_, exp_fn->center_);
                    exp_fn->sch_node_->parent_ = cur_n;
                    exp_fn->sch_node_->root_id_ = cur_n->root_id_;
                    exp_fn->sch_node_->flag_ = 0;
                    pr_l.push(exp_fn->sch_node_);
                }
            }
        }
    }

    HandleLocalData(partition_hn, partition_fn, fake_hn_s);

}

void GraphVoronoiPartition::GlobalGVP(){
    list<h_ptr> fake_hn_s;
    list<h_ptr> searched_hn;

    LoadGlobalData(fake_hn_s);
    /* GraphVoronoiPartition */
    prio_D pr_l;
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    h_ptr cur_hn, exp_hn;
    for(auto &hns : fake_hn_s){
        hns->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, hns->id_, hns->pos_);
        cur_n = hns->sch_node_;
        cur_n->flag_ = 1;
        cur_n->id_ = hns->id_;
        cur_n->root_id_ = hns->id_;
        pr_l.push(cur_n);
    }

    while(!pr_l.empty()){
        cur_n = pr_l.top();
        pr_l.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;
        if(cur_n->id_ <= SDM_->drone_num_){   // fake start hn
            for(auto &hn : fake_hn_s){
                if(hn->id_ == cur_n->id_){
                    cur_hn = hn;
                    break;
                }
            }
        }
        else{                                  // mid hn
            if(!MDTG_->FindHnode(cur_n->pos_, cur_n->id_, cur_hn)){
                ROS_ERROR("error get hnode GetClosestGlobalTarget2");
                ros::shutdown();
                return;
            }
            searched_hn.emplace_back(cur_hn);
        }

        /* expand Hneighbours */
        for(auto &hhe : cur_hn->hh_edges_){
            if(!(hhe->e_flag_ & 32)) continue;
            
            if(hhe->head_n_ == cur_hn)
                exp_hn = hhe->tail_n_;
            else
                exp_hn = hhe->head_n_;

            if(exp_hn->sch_node_ == NULL){
                /* new node */
                exp_hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_s_, 0.0, exp_hn->id_, exp_hn->pos_);
                nei_n = exp_hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->root_id_ = cur_n->root_id_;
                nei_n->flag_ = 1;
                pr_l.push(nei_n);
            }
            else{
                /* in close list */
                if(exp_hn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = exp_hn->sch_node_;
                if(cur_n->g_ + hhe->length_s_ + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    exp_hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_s_, 0.0, exp_hn->id_, exp_hn->pos_);
                    exp_hn->sch_node_->parent_ = cur_n;
                    exp_hn->sch_node_->root_id_ = cur_n->root_id_;
                    exp_hn->sch_node_->flag_ = 1;
                    pr_l.push(exp_hn->sch_node_);
                }
            }
        }
    }
    HandleGlobalData(searched_hn);
}

void GraphVoronoiPartition::LoadLocalData(list<h_ptr> &hn_l, list<f_ptr> &fn_l, list<h_ptr> &fake_hn_l){
    /* load fn and hn */

    hn_l = MDTG_->local_h_list_;
    for(auto &hn : hn_l){
        hn->h_flags_ |= 2;
        for(auto &hfe : hn->hf_edges_){
            // if(hfe->e_flag_ & 16){
                hfe->tail_n_->f_flag_ |= 2;
                fn_l.emplace_back(hfe->tail_n_);
            // }
        }
    }


    /* load start */
    swarm_connect_h_[SDM_->self_id_ - 1].clear();
    for(auto &hn : hn_l) swarm_connect_h_[SDM_->self_id_ - 1].push_back({MDTG_->GetVoxId(hn->pos_), hn->id_}); 
    swarm_connect_h_dist_[SDM_->self_id_ - 1].clear();
    for(auto &hn_d : MDTG_->local_h_dist_list_) swarm_connect_h_dist_[SDM_->self_id_ - 1].emplace_back(hn_d); 
    swarm_connect_f_id_[SDM_->self_id_ - 1].clear();
    for(auto &fn : MDTG_->local_f_list_) swarm_connect_f_id_[SDM_->self_id_ - 1].push_back(fn->id_); 
    swarm_connect_f_dist_[SDM_->self_id_ - 1].clear();
    for(auto &fn_d : MDTG_->local_f_dist_list_) swarm_connect_f_dist_[SDM_->self_id_ - 1].emplace_back(fn_d); 

    // cout<<"MDTG_->local_f_list_:"<<MDTG_->local_f_list_.size()<<"  MDTG_->local_h_list_:"<<MDTG_->local_h_list_.size()<<endl;//debug
    for(uint8_t i = 0; i < SDM_->drone_num_; i++){
    // for(int i = 0; i < swarm_h_id_.size(); i++){  
        h_ptr hs = NULL;            
        // hnodes invaders and self
        list<pair<uint16_t, uint32_t>>::iterator h_it = swarm_connect_h_[i].begin();
        list<double>::iterator he_it = swarm_connect_h_dist_[i].begin();
        for(;h_it != swarm_connect_h_[i].end(); h_it++, he_it++){
            for(auto &hn : hn_l){
                if(hn->id_ == h_it->second){
                    if(hs == NULL){
                        CreateFakeHnode(i + 1, hs);
                        fake_hn_l.emplace_back(hs);
                    }
                    CreateFakeEdge(hs, hn, *he_it);
                }
            }
        }

        // fnodes invaders and self
        list<uint16_t>::iterator f_it = swarm_connect_f_id_[i].begin();
        list<double>::iterator fe_it = swarm_connect_f_dist_[i].begin();
        for(; f_it != swarm_connect_f_id_[i].end(); f_it++, fe_it++){
            for(auto &fn:  fn_l){
                if(*f_it == fn->id_){
                    if(hs == NULL){
                        CreateFakeHnode(i + 1, hs);
                        fake_hn_l.emplace_back(hs);
                    }
                    CreateFakeEdge(hs, fn, *fe_it);
                }
            }
        }
    // }
    }
}

void GraphVoronoiPartition::LoadGlobalData(list<h_ptr> &fake_hn_l){
    for(int i = 0; i < SDM_->drone_num_; i++){
        // if(!(swarm_job_state_[i] & 1) && i+1 != SDM_->self_id_) continue;
        h_ptr hs, hn;            
        CreateFakeHnode(i + 1, hs);
        list<pair<uint16_t, uint32_t>>::iterator h_it = swarm_connect_h_[i].begin();
        list<double>::iterator he_it = swarm_connect_h_dist_[i].begin();
        for(; h_it != swarm_connect_h_[i].end(); h_it++, he_it++){
            if(!MDTG_->FindHnode(h_it->first, h_it->second, hn)){
                ROS_WARN("id:%d LoadGlobalData. dont have hnode %d", SDM_->self_id_, h_it->second);
                continue;
            }
            CreateFakeEdge(hs, hn, *he_it);
        }
        fake_hn_l.emplace_back(hs);
    }
}

void GraphVoronoiPartition::HandleLocalData(list<h_ptr> &hn_l, list<f_ptr> &fn_l, list<h_ptr> &fake_hn_l){
    list<pair<uint32_t, uint32_t>> h_send;
    dist_to_f_.clear();
    /* erase old flags */
    for(auto &f_id : (*local_fn_)){
        MDTG_->F_depot_[f_id]->f_flag_ &= 249;
        FG_->ClearOwner(f_id); 
    }
    for(auto &hn_id : (*local_hn_)){
        h_ptr hn;
        if(MDTG_->FindHnode(hn_id.first, hn_id.second, hn)){
            hn->h_flags_ &= 253;
        }
    }
    (*local_fn_).clear();
    (*local_hn_).clear();
    
    uint8_t temp_owner;

    int debug_count = 1;
    for(auto &fn : fn_l){
        if(fn->sch_node_ == NULL) continue;
        if(fn->sch_node_->root_id_ == SDM_->self_id_){

            FG_->ClearOwner(fn->id_); 
            FG_->ChangeOwner(fn->id_, SDM_->self_id_, fn->sch_node_->g_, temp_owner); 
            (*local_fn_).emplace_back(fn->id_);
            dist_to_f_.emplace_back(fn->sch_node_->g_);
            fn->f_flag_ |= 2;

            // if(fn->hf_edge_ != NULL){
            //     bool insert_hn = true;
            //     for(auto &hn : local_hn_){
            //         if(hn->id_ == fn->hf_edge_->tail_){
            //             insert_hn = false;
            //             break;
            //         }
            //     }
            //     if(insert_hn){
            //         h_send.push_back({fn->hf_edge_->head_, LRM_->PostoId(fn->hf_edge_->head_n_->pos_)});
            //         dist_to_h.emplace_back(fn->hf_edge_->head_n_->sch_node_->g_);
            //         local_hn_.emplace_back(fn->hf_edge_->head_n_);
            //     }
            // }
            // ROS_WARN("id:%d HandleLocalData2.2 %f  %d  %d  %d", SDM_->self_id_, fn->sch_node_->g_, temp_owner, (fn->hf_edge_==NULL), fn->id_);
            // cout<<int(fn->sch_node_==NULL)<<"  "<<(fn->hf_edge_==NULL)<<"  fne:"<<int(fn->f_flag_)<<"  "
            //     <<fn->hf_edge_->length_<<"   "<<int(fn->hf_edge_->e_flag_)<<endl;
        }
        else{
            fn->f_flag_ &= 253;
        }
        debug_count++;
        fn->sch_node_ = NULL;
    }


    for(auto &hn : hn_l){
        // if(hn->sch_node_->root_id_ == SDM_->self_id_){
        //     // local_hn_.emplace_back(hn);
        //     // bool push_hn = true;
        //     // for(auto lhn : local_hn_){
        //     //     if(lhn->id_ == hn->id_){
        //     //         push_hn = false;
        //     //         break;
        //     //     }
        //     // }
        //     // if(push_hn) {
        //         h_send.push_back({hn->id_, LRM_->PostoId(hn->pos_)});
        //         dist_to_h.emplace_back(hn->sch_node_->g_);
        //         local_hn_.emplace_back(hn);
        //         hn->h_flags_ |= 2;
        //     // }
        // }
        // else{
        //     hn->h_flags_ &= 253;
        // }
        if(hn->sch_node_->root_id_ == SDM_->self_id_){
            hn->h_flags_ |= 2;
            (*local_hn_).push_back({MDTG_->GetVoxId(hn->pos_), hn->id_});
        }
        hn->sch_node_ = NULL;
    }

}

void GraphVoronoiPartition::HandleGlobalData(list<h_ptr> &hn_l){
    list<double> dist_to_f;
    list<double> dist_to_h;
    // list<pair<uint32_t, uint32_t>> h_send;

    // for(auto &f_id : (*local_fn_)){
    //     MDTG_->F_depot_[f_id]->f_flag_ &= 253;
    // }
    for(auto &hn_id : (*GVP_hn_)){
        h_ptr hn;
        if(MDTG_->FindHnode(hn_id.first, hn_id.second, hn)){
            hn->h_flags_ &= 251;
        }
    }
    // (*local_fn_).clear();
    (*GVP_hn_).clear();
    
    if(SDM_->is_ground_) ShowPartition(hn_l);

    for(auto &hn : hn_l){
        if(hn->sch_node_->root_id_ == SDM_->self_id_){
            (*GVP_hn_).push_back({MDTG_->GetVoxId(hn->pos_), hn->id_});
            // dist_to_h_.emplace_back(hn->sch_node_->g_);
            hn->h_flags_ |= 4;
        }
        else{
            hn->h_flags_ &= 251;
        }
        hn->sch_node_ = NULL;
    }
}

void GraphVoronoiPartition::GetLocalFNodes(const Eigen::Vector4d &c_state, const Eigen::Vector3d &c_vel, list<Eigen::Vector3d> &path, double &exp_length, 
                            Eigen::Vector4d &t_state, pair<int, int> &f_v, int &h_id, int &exp_state){
    list<pair<double, pair<int, int>>> d_f_v;

    list<Eigen::Vector3d> targets;
    list<double> dist_l;
    list<pair<uint16_t, uint8_t>> vp_l;
    Eigen::Vector3d vp;
    list<list<Eigen::Vector3d>> paths, pruned_paths;
    list<list<Eigen::Vector3d>> f_paths;
    // list<list<Eigen::Vector3d>> h_paths;

    h_id = -1;
    f_v.first = -1;
    /* load vps */
    for(auto &f_id : (*local_fn_)){
        if(FG_->f_grid_[f_id].f_state_ == 1 && (MDTG_->F_depot_[f_id]->f_flag_ & 2)){
            for(uint8_t i = 0; i < FG_->f_grid_[f_id].local_vps_.size(); i++){
                if(FG_->f_grid_[f_id].local_vps_[i] == 1){
                    FG_->GetVpPos(f_id, i, vp);
                    vp_l.push_back({f_id, i});
                    targets.emplace_back(vp);
                    // if(!LRM_->IsFeasible(vp)){
                    //     ROS_WARN("infeasible vp");
                    //     cout<<vp.transpose()<<endl;
                    // }
                }
            }
        }
    }

    /* load hnodes */
    Eigen::Vector3d lowbd = c_state.block(0, 0, 3, 1) - Eigen::Vector3d::Ones() * MDTG_->eurange_;
    Eigen::Vector3d upbd = c_state.block(0, 0, 3, 1) + Eigen::Vector3d::Ones() * MDTG_->eurange_;
    list<h_ptr> hn_l;
    MDTG_->GetHnodesBBX(upbd, lowbd, hn_l);
    for(auto &hn : hn_l) {
        targets.emplace_back(hn->pos_);
    }
    cout<<"id:"<<int(SDM_->self_id_)<<"hn_l:"<<hn_l.size()<<"  lowbd:"<<lowbd.transpose()<<"   upbd:"<<upbd.transpose()<<endl;

    /* local Djkstra search */
    if(!LRM_->DjkstraLocalDist(c_state.block(0, 0, 3, 1), paths, pruned_paths, targets, dist_l)) {
        ROS_WARN("id:%d GetLocalFNodes fail djk", SDM_->self_id_);
        cout<<"id:"<<int(SDM_->self_id_)<<c_state.block(0, 0, 3, 1).transpose()<<"  "<<endl;
        exp_state = 2;
        return;
    }

    /* find local available fn */
    list<pair<uint16_t, uint8_t>>::iterator vp_it = vp_l.begin();
    list<list<Eigen::Vector3d>>::iterator path_it = paths.begin();
    list<list<Eigen::Vector3d>>::iterator p_path_it = pruned_paths.begin();
    // list<Eigen::Vector3d> debug_points;
    list<double>::iterator d_it = dist_l.begin();
    Eigen::Vector4d vp_pose;
    for(;vp_it != vp_l.end(); d_it++, vp_it++, path_it++, p_path_it++){
        if(*d_it < 0) continue;
        double v_cost = 0;
        double yaw_cost = YawP_->GetMinT(0, M_PI) * 1.5;
        double path_cost = (*d_it) / max_v_;
        list<Eigen::Vector3d>::reverse_iterator p_it = p_path_it->rbegin();
        while(p_it != p_path_it->rend()){
            if(((*p_it) - c_state.head(3)).norm() > 0.5){
                v_cost = (((*p_it) - c_state.head(3)).normalized() * max_v_ - c_vel).norm() / max_a_;
                // debug_points.emplace_back(*p_it);
                break;
            }
            p_it++;
        }
        if(FG_->GetVp(vp_it->first, vp_it->second, vp_pose)){
            yaw_cost = YawP_->GetMinT(c_state(3), vp_pose(3)) * 1.5;
        }
        d_f_v.push_back({max(yaw_cost, path_cost) + v_cost, *vp_it});

        f_paths.emplace_back(*path_it);
    }
    // Debug(debug_points, visualization_msgs::Marker::SPHERE_LIST, 0);

    /* find the closest fn */
    if(d_f_v.size() > 0){
        int best_f, best_v;
        best_f = -1;
        while(!d_f_v.empty()){
            /* get closest vp */

            double best_d = 99999.0;
            for(auto &it : d_f_v){
                if(it.first < best_d){
                    best_d = it.first;
                    best_f = it.second.first;
                    best_v = it.second.second;
                }
            }

            if(best_d > 99998.0) {
                ROS_ERROR("how???");
                exp_state = 2;
                return;
            }


            list<pair<double, pair<int, int>>>::iterator i = d_f_v.begin();
            list<list<Eigen::Vector3d>>::iterator p_it = f_paths.begin();
            if(!FG_->StrongCheckViewpoint(best_f, best_v, true)){   //dead vp, erase
                MDTG_->RemoveVp(FG_->f_grid_[best_f].center_, best_f, best_v, true);

                for(; i != d_f_v.end(); i++, p_it++){
                    if(i->second.first == best_f && i->second.second == best_v){
                        d_f_v.erase(i);
                        f_paths.erase(p_it);
                        break;
                    }
                }
                best_f = -1;
            }
            else{ // success vp
                f_v.first = best_f;
                f_v.second = best_v;

                for(; i != d_f_v.end(); i++, p_it++){
                    if(i->second.first == best_f && i->second.second == best_v) {
                        path = *p_it;
                    }
                }
                exp_length = best_d;
                f_ptr fn = MDTG_->F_depot_[f_v.first];
                if(fn->hf_edge_ == NULL) h_id = 0;
                else h_id = fn->hf_edge_->head_;
                FG_->GetVp(f_v.first, f_v.second, t_state);
                exp_state = 0;
                return;
            }
        }
    }

    /* find local available hn, create edges */
    cout<<"id:"<<int(SDM_->self_id_)<<"create fake hn, local global"<<endl;
    h_ptr fake_hn;
    CreateFakeHnode(SDM_->self_id_, fake_hn);
    list<h_ptr>::iterator h_it = hn_l.begin();
    for(;h_it != hn_l.end(); d_it++, h_it++, path_it++){
        if(*d_it < 0) continue;
        CreateFakeEdge(fake_hn, *h_it, *d_it);
        fake_hn->hh_edges_.back()->path_ = *path_it;
    }

    /* global search */
    cout<<"id:"<<int(SDM_->self_id_)<<"local global"<<endl;
    vector<pair<double, list<Eigen::Vector3d>>> d_p;
    vector<h_ptr> hn_t;
    for(auto &hn_id : (*local_hn_)) {
        h_ptr hn;
        if(MDTG_->FindHnode(hn_id.first, hn_id.second, hn)){
            hn_t.emplace_back(hn);
        }
    }
    // vector<list<h_ptr>> debug_paths;
    // list<h_ptr> b_debug_path;
    MDTG_->GetGlobalTarget(fake_hn, d_p, hn_t/*, debug_paths*/);

    /* find the closest fn */
    pair<double, list<Eigen::Vector3d>> best_exp;
    best_exp.first = 99999.0;
    // list<Eigen::Vector3d> debug_path1, debug_path2;
    cout<<"id:"<<int(SDM_->self_id_)<<"hn_t:"<<hn_t.size()<<endl;//debug
    for(int i = 0; i < hn_t.size(); i++){
        if(d_p[i].first < 0.0) continue;
        // cout<<"  hnt:"<<int(hn_t[i]->id_)<<"hn_t[i]-hf_edges_:"<<hn_t[i]->hf_edges_.size()<<endl;//debug
        for(auto &hfe : hn_t[i]->hf_edges_){
            if(!(hfe->e_flag_ & 16)) continue;

                if(MDTG_->F_depot_[hfe->tail_]->f_flag_ & 2){
                    double edge_length = hfe->length_;
                    cout<<"edge_length_s:"<<edge_length<<endl;//debug
                    // if(hfe->e_flag_ & 16)
                        // edge_length = hfe->length_;
                    // cout<<"edge_length_:"<<edge_length<<"  d_p[i].first:"<<d_p[i].first<<"  flag:"<<int(hfe->e_flag_)<<endl;//debug
                    if(edge_length + d_p[i].first < best_exp.first){
                        best_exp.second = d_p[i].second;
                        best_exp.second.insert(best_exp.second.end(), hfe->path_.begin(), hfe->path_.end());
                        // debug_path2 = hfe->path_;
                        // debug_path1 = d_p[i].second;
                        // b_debug_path = debug_paths[i];
                        best_exp.first = edge_length + d_p[i].first;
                        f_v.first = hfe->tail_;
                        f_v.second = hfe->tail_n_->vp_id_;
                        FG_->GetVp(f_v.first, f_v.second, t_state);
                        // if(FG_->GetVp(f_v.first, f_v.second, t_state)) {
                        //     cout<<t_state.transpose()<<"get local global!!!!!!!!!!"<<int(hn_t[i]->id_)<<endl;
                        // }
                        // else cout<<f_v.first<<"get local global?????????"<<f_v.second<<"  flag:"<<int(hfe->e_flag_)<<endl;
                        path = best_exp.second;
                        h_id = hn_t[i]->id_;
                    }
                }
        }
    }
    if(best_exp.first < 99998.0) {
        exp_state = 1;
        // Debug(debug_path1, visualization_msgs::Marker::LINE_STRIP, 1);
        // Debug(debug_path2, visualization_msgs::Marker::LINE_STRIP, 2);
        // for(auto &p : path) cout<<"path:"<<p.transpose()<<endl;
        // for(auto &p : debug_path1) cout<<"p:"<<p.transpose()<<endl;
        // for(auto &p : debug_path2) cout<<"p_end:"<<p.transpose()<<endl;
        // for(auto &h : b_debug_path) cout<<"hid:"<<int(h->id_)<<"  p:"<<h->pos_.transpose()<<endl;
        // cout<<"c_state:"<<c_state.transpose()<<endl;
        // cout<<"t_state:"<<t_state.transpose()<<endl;
    }
    else exp_state = 2;

    return;
}

void GraphVoronoiPartition::GetGlobalFNodes(const Eigen::Vector4d &c_state, list<Eigen::Vector3d> &path, double &exp_length, 
                            Eigen::Vector4d &t_state, pair<int, int> &f_v, int &h_id, int &exp_state){
    /** load local hnode **/
    f_ptr best_f;
    list<Eigen::Vector3d> targets;
    list<list<Eigen::Vector3d>> paths, pruned_paths;
    list<double> dist_l;
    bool have_target = false;

    Eigen::Vector3d lowbd = c_state.block(0, 0, 3, 1) - Eigen::Vector3d::Ones() * MDTG_->eurange_;
    Eigen::Vector3d upbd = c_state.block(0, 0, 3, 1) + Eigen::Vector3d::Ones() * MDTG_->eurange_;
    list<h_ptr> hn_l;
    cout<<"id:"<<int(SDM_->self_id_)<<"hn_l:"<<hn_l.size()<<"  lowbd:"<<lowbd.transpose()<<"   upbd:"<<upbd.transpose()<<endl;

    MDTG_->GetHnodesBBX(upbd, lowbd, hn_l);
    for(auto &hn : hn_l) targets.emplace_back(hn->pos_);

    /* local Djkstra search, get UAV to hn */
    if(!LRM_->DjkstraLocalDist(c_state.block(0, 0, 3, 1), paths, pruned_paths, targets, dist_l)) {
        ROS_WARN("id:%d GetGlobalFNodes fail djk", SDM_->self_id_);
        return;
    }

    /* clear infeasible hns */
    h_ptr fake_hn;
    list<h_ptr>::iterator h_it = hn_l.begin();
    list<double>::iterator d_it = dist_l.begin();
    list<list<Eigen::Vector3d>>::iterator path_it = paths.begin();
    CreateFakeHnode(SDM_->self_id_, fake_hn);
    for(;h_it != hn_l.end(); d_it++, h_it++, path_it++){
        if(*d_it < 0) continue;
        CreateFakeEdge(fake_hn, *h_it, *d_it);
        fake_hn->hh_edges_.back()->path_ = *path_it;
    }
    // cout<<"id:"<<int(SDM_->self_id_)<<"fake edges"<<fake_hn->hh_edges_.size()<<endl;

    /** local DTG search **/
    list<pair<double, list<Eigen::Vector3d>>> d_p;
    list<h_ptr> hn_t;
    MDTG_->GlobalLocalSearch(fake_hn, d_p, hn_t);

    // cout<<"id:"<<int(SDM_->self_id_)<<"global hn target:"<<hn_t.size()<<endl;
    /* get best hn and fn */
    if(GetBestTarget(d_p, hn_t, best_f, path, t_state, f_v, h_id)){
        // cout<<"id:"<<int(SDM_->self_id_)<<"have_target1"<<endl;
        have_target = true;
    }

    /** search in whole DTG **/
    if(!have_target){
        // cout<<"id:"<<int(SDM_->self_id_)<<"  FullSearch"<<endl;
        MDTG_->FullSearch(fake_hn, d_p, hn_t);
            /* get best hn and fn */
        // cout<<"id:"<<int(SDM_->self_id_)<<"  FullSearch target:"<<hn_t.size()<<endl;
        if(GetBestTarget(d_p, hn_t, best_f, path, t_state, f_v, h_id)){
            // cout<<"id:"<<int(SDM_->self_id_)<<"  have_target2"<<endl;
            have_target = true;
        }
    }

    if(have_target){
        /** try to find shorter path **/
        list<Eigen::Vector3d> shorter_path;
        Eigen::Vector3d ps, safe_pt;
        ps = c_state.head(3);
        safe_pt = t_state.head(3);

        // for(auto &pt : path){
        //     if(!LRM_->IsFeasible(pt)){
        //         break;
        //     }
        //     safe_pt = pt;
        // }

        if(LRM_->GetPath(ps, safe_pt, shorter_path, false, 3000)){
            /* find safe path */
            path = shorter_path;
            if(ExploreOrFollow(best_f)) exp_state = 0;
            else exp_state = -2;
        }
        else{
            /* use dangerous path */
            if(ExploreOrFollow(best_f)) exp_state = 1;
            else exp_state = -1;
        }
    }
    else{
        /* no path */
        exp_state = 2;
    }
}

bool GraphVoronoiPartition::LocalExplorable(){
    for(auto &hn_id : (*local_hn_)){
        h_ptr hn;
        if(!MDTG_->FindHnode(hn_id.first, hn_id.second, hn)) continue;
        for(auto &hfe : hn->hf_edges_){
            if((hfe->e_flag_ & 16) && hfe->tail_n_->cf_->f_state_ == 1 && (hfe->tail_n_->f_flag_ & 2)){
                return true;
            }
        }
    }
    return false;
}

void GraphVoronoiPartition::LocalExplorableDebug(){
    for(auto &hn_id : (*local_hn_)){
        h_ptr hn;
        if(!MDTG_->FindHnode(hn_id.first, hn_id.second, hn)) continue;
        for(auto &hfe : hn->hf_edges_){
            if((hfe->e_flag_ & 16) && hfe->tail_n_->cf_->f_state_ == 1 && (hfe->tail_n_->f_flag_ & 2)){
                cout<<"f:"<<int(hfe->tail_)<<"  pos:"<<hfe->tail_n_->center_.transpose()<<
                    "  owner:"<<int(hfe->tail_n_->cf_->owner_)<<"  e flag:"<<int(hfe->e_flag_)<<"  tarh:"<<int(hn->id_)<<endl;
                return;
            }
        }
    }
}

bool GraphVoronoiPartition::GlobalExplorable(){
    for(auto &hn : MDTG_->H_list_){
        for(auto &hfe : hn->hf_edges_){
            if((hfe->e_flag_ & 16) && hfe->tail_n_->cf_->f_state_ == 1){
                return true;
            }
        }
    }
    return false;
}

bool GraphVoronoiPartition::ExploreOrFollow(f_ptr &f_target){
    if(f_target->exploring_id_ != 0 && f_target->exploring_id_ != SDM_->self_id_) return false;
    else return true;
}

bool GraphVoronoiPartition::GetBestTarget(list<pair<double, list<Eigen::Vector3d>>> &d_p, list<h_ptr> &hn_l, f_ptr &fn_t,
                            list<Eigen::Vector3d> &path, Eigen::Vector4d &t_state, pair<int, int> &f_v, int &h_id){
    do {
        list<pair<double, list<Eigen::Vector3d>>>::iterator d_p_it = d_p.begin();
        list<h_ptr>::iterator hn_it = hn_l.begin();
        double best_gain = 1e-8;
        h_ptr best_h;
        fn_t = NULL;    
        ROS_WARN("id:%d GetBestTarget!", SDM_->self_id_);
        for(; hn_it != hn_l.end(); hn_it++, d_p_it++){
            double g;   //to modify
            double f_num = 0;
            double f_dist = 99999.0;
            f_ptr cur_best_f;
            hfe_ptr best_e;
            for(auto &hfe : (*hn_it)->hf_edges_){
                if(hfe->tail_n_->cf_->f_state_ == 1 && (hfe->e_flag_ & 16)){
                    f_num += 1.0;
                    if(f_dist > hfe->length_){
                        f_dist = hfe->length_;
                        cur_best_f = hfe->tail_n_;
                        best_e = hfe;
                    }
                }
            }
            // cout<<"f_num:"<<f_num<<"   d_p_it->first:"<<d_p_it->first<<"  f_dist:"<<f_dist<<endl;
            int work_num = 0;
            double t = d_p_it->first / max_v_;
            g = f_num;
            for(int i = 0; i < swarm_job_h_.size(); i++){
                if(i+1 == SDM_->self_id_) continue;
                double dt = t - (swarm_job_h_dist_[i] - last_new_job_[i]);
                if(swarm_job_h_[i] == (*hn_it)->id_ && dt > 0){
                    g -= tau_ * dt;
                    work_num++;
                }
            }
            g = (max(g, 0.0) + (f_num * allowance_)) / (work_num + 1);

            g = g * exp(-d_p_it->first * lambda_);
            // cout<<"g1:"<<g<<"   best_gain:"<<best_gain<<"  cur_best_f==NULL "<<(cur_best_f == NULL)<<endl;

            if(g > best_gain){
                best_gain = g;
                best_h = (*hn_it);
                fn_t = cur_best_f;
                path = d_p_it->second;
                path.insert(path.end(), best_e->path_.begin(), best_e->path_.end());
                f_v.first = fn_t->id_;
                f_v.second = fn_t->vp_id_;
                h_id = (*hn_it)->id_;
                FG_->GetVp(f_v.first, f_v.second, t_state);
            }
        }
        if(fn_t != NULL && FG_->StrongCheckViewpoint(fn_t->id_, fn_t->vp_id_, true)){ // good target
            // cout<<" success fn:"<<int(fn_t->id_)<<"   vp:"<<int(fn_t->vp_id_)<<" flag"<<int(fn_t->f_flag_)<<" hfe flag:"<<int(fn_t->hf_edge_->e_flag_)<<endl;
            break;
        }
        else if(fn_t != NULL){  // vp low gain
            // cout<<"remove fn:"<<int(fn_t->id_)<<"   vp:"<<int(fn_t->vp_id_)<<endl;
            if(fn_t->vp_id_ == -1) {
                ros::shutdown();
                return false;
            }
            MDTG_->RemoveVp(FG_->f_grid_[fn_t->id_].center_, fn_t->id_, fn_t->vp_id_, true);
            // cout<<" fn:"<<int(fn_t->id_)<<"   new vp:"<<int(fn_t->vp_id_)<<" flag"<<int(fn_t->f_flag_)<<" hfe flag:"<<int(fn_t->hf_edge_->e_flag_)<<endl;
        }
        // else cout<<"NULL fn:"<<endl;
    } while(fn_t != NULL);
    // cout<<"return:"<<(fn_t != NULL)<<endl;
    if(fn_t != NULL) return true;
    else return false;
}


void GraphVoronoiPartition::JobTimerCallback(const ros::TimerEvent &e){
    for(uint8_t i = 0; i < SDM_->drone_num_; i++){
        if(i + 1 == SDM_->self_id_) continue;
        for(auto &job : SDM_->jobs_[i]){
            LoadSwarmJob(job);
        }
        SDM_->jobs_[i].clear();
    }
}

void GraphVoronoiPartition::StateTimerCallback(const ros::TimerEvent &e){
    for(uint8_t i = 0; i < SDM_->drone_num_; i++){
        if(i + 1 == SDM_->self_id_) continue;
        LoadSwarmState(SDM_->states_[i]);
        // SDM_->states_[i].clear();
    }
    /* state send */
    double cur_t = ros::WallTime::now().toSec();
    if(cur_t - last_state_pub_t_ > 1.0 && !SDM_->is_ground_){

        list<uint16_t> local_fn_send;
        list<double> dist_to_local_fn_send;
        for(auto &f : (*local_fn_)){
            if(!(MDTG_->F_depot_[f]->f_flag_ & 4) && FG_->f_grid_[f].f_state_ == 1){
                local_fn_send.emplace_back(f);
                dist_to_local_fn_send.emplace_back(FG_->f_grid_[f].owner_dist_);
                MDTG_->F_depot_[f]->f_flag_ |= 4;
            }
        }

        swarm_connect_h_[SDM_->self_id_ - 1].clear();
        for(auto &hn : MDTG_->local_h_list_) swarm_connect_h_[SDM_->self_id_ - 1].push_back({MDTG_->GetVoxId(hn->pos_), hn->id_}); 
        swarm_connect_h_dist_[SDM_->self_id_ - 1].clear();
        for(auto &hn_d : MDTG_->local_h_dist_list_) swarm_connect_h_dist_[SDM_->self_id_ - 1].emplace_back(hn_d); 
        swarm_connect_f_id_[SDM_->self_id_ - 1].clear();
        for(auto &fn : MDTG_->local_f_list_) swarm_connect_f_id_[SDM_->self_id_ - 1].push_back(fn->id_); 
        swarm_connect_f_dist_[SDM_->self_id_ - 1].clear();
        for(auto &fn_d : MDTG_->local_f_dist_list_) swarm_connect_f_dist_[SDM_->self_id_ - 1].emplace_back(fn_d); 

        SDM_->SetState(1, local_fn_send, dist_to_local_fn_send, swarm_connect_f_id_[SDM_->self_id_ - 1], swarm_connect_f_dist_[SDM_->self_id_ - 1], 
                        swarm_connect_h_[SDM_->self_id_ - 1], swarm_connect_h_dist_[SDM_->self_id_ - 1]);
        last_state_pub_t_ = cur_t;

    }
}

void GraphVoronoiPartition::PartitionTimerCallback(const ros::TimerEvent &e){
    cout<<"\033[0;44m id:"<<int(SDM_->self_id_)<<" Partition\033[0m"<<endl;
    if(SDM_->statistic_) SDM_->CS_.StartTimer(0);
    if(!SDM_->is_ground_){
        ROS_WARN("id:%d LocalGVP!", SDM_->self_id_);
         LocalGVP();
    }

    GlobalGVP();
    if(SDM_->statistic_) SDM_->CS_.EndTimer(0);
    // ROS_WARN("id:%d PartitionTimerCallback!", SDM_->self_id_);
    // cout<<"local_fn_:"<<local_fn_->size()<<"  local_hn_:"<<local_hn_->size()<<"  GVP_hn_:"<<GVP_hn_->size()<<endl;
    /* partition send */
    list<uint16_t> local_fn_send;
    list<double> dist_to_local_fn_send;
    for(auto &f : (*local_fn_)){
        if(!(MDTG_->F_depot_[f]->f_flag_ & 4) && FG_->f_grid_[f].f_state_ == 1){
            local_fn_send.emplace_back(f);
            dist_to_local_fn_send.emplace_back(FG_->f_grid_[f].owner_dist_);
            MDTG_->F_depot_[f]->f_flag_ |= 4;
        }
    }

    if(!SDM_->is_ground_){
        swarm_connect_h_[SDM_->self_id_ - 1].clear();
        for(auto &hn : MDTG_->local_h_list_) swarm_connect_h_[SDM_->self_id_ - 1].push_back({MDTG_->GetVoxId(hn->pos_), hn->id_}); 
        swarm_connect_h_dist_[SDM_->self_id_ - 1].clear();
        for(auto &hn_d : MDTG_->local_h_dist_list_) swarm_connect_h_dist_[SDM_->self_id_ - 1].emplace_back(hn_d); 
        swarm_connect_f_id_[SDM_->self_id_ - 1].clear();
        for(auto &fn : MDTG_->local_f_list_) swarm_connect_f_id_[SDM_->self_id_ - 1].push_back(fn->id_); 
        swarm_connect_f_dist_[SDM_->self_id_ - 1].clear();
        for(auto &fn_d : MDTG_->local_f_dist_list_) swarm_connect_f_dist_[SDM_->self_id_ - 1].emplace_back(fn_d); 
        SDM_->SetState(0, local_fn_send, dist_to_local_fn_send, swarm_connect_f_id_[SDM_->self_id_ - 1], swarm_connect_f_dist_[SDM_->self_id_ - 1], 
                            swarm_connect_h_[SDM_->self_id_ - 1], swarm_connect_h_dist_[SDM_->self_id_ - 1]);
    }
}

void GraphVoronoiPartition::ShowPartition(list<h_ptr> &hn_l){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 0;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.35;
    mka.markers[0].scale.y = 0.35;
    mka.markers[0].scale.z = 0.35;

    geometry_msgs::Point pt;

    for(auto &hn : hn_l){
        pt.x = hn->pos_(0);
        pt.y = hn->pos_(1);
        pt.z = hn->pos_(2);
        mka.markers[0].points.emplace_back(pt);
        mka.markers[0].colors.emplace_back(CM_->Id2Color(hn->sch_node_->root_id_, 1.0));
    }
    show_pub_.publish(mka);
}

void GraphVoronoiPartition::Debug(list<Eigen::Vector3d> &fl, int type, int pub_id){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = pub_id;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = type;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 0.7;
    if(pub_id == 1){
        mk.color.b = 0.9;
        mk.color.r = 0.6;
    }
    else{
        mk.color.g = 0.6;
        mk.color.r = 0.9;
    }
    geometry_msgs::Point pt;
    for(auto &f : fl){
        // for(auto &v : f.second){
        //     pt.x = v.second(0);
        //     pt.y = v.second(1);
        //     pt.z = v.second(2);
        //     mk.points.emplace_back(pt);
        // }
        pt.x = f(0);
        pt.y = f(1);
        pt.z = f(2);
        mk.points.emplace_back(pt);
    }
    if(mk.points.size() != 0)
        debug_pub_.publish(mk);
    // fl.clear();
}
