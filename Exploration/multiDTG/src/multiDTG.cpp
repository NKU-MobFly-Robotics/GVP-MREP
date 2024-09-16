#include <multiDTG/multiDTG.h>
using namespace DTG;
void MultiDTG::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    double topo_range;
    std::string ns = ros::this_node::getName();
    nh_private.param(ns + "/Exp/minX", origin_(0), 0.0);
    nh_private.param(ns + "/Exp/minY", origin_(1), 0.0);
    nh_private.param(ns + "/Exp/minZ", origin_(2), 0.0);
    nh_private.param(ns + "/Exp/maxX", map_upbd_(0), 0.0);
    nh_private.param(ns + "/Exp/maxY", map_upbd_(1), 0.0);
    nh_private.param(ns + "/Exp/maxZ", map_upbd_(2), 0.0);
    nh_private.param(ns + "/MR_DTG/Toporange", topo_range, 6.0);
    nh_private.param(ns + "/MR_DTG/H_thresh", H_thresh_, 5.0);
    nh_private.param(ns + "/MR_DTG/update_FF", update_FF_, false);
    nh_private.param(ns + "/MR_DTG/resX", vox_scl_(0), 1.0);
    nh_private.param(ns + "/MR_DTG/resY", vox_scl_(1), 1.0);
    nh_private.param(ns + "/MR_DTG/resZ", vox_scl_(2), 1.0);
    nh_private.param(ns + "/MR_DTG/show_edge_details", show_e_details_, false);
    nh_private.param(ns + "/Exp/UAV_id", uav_id_, 10);
    nh_private.param(ns + "/Exp/drone_num", drone_num_, 1);
    nh_private.param(ns + "/block_map/sensor_max_range", sensor_range_, 5.0);


    // FG_ = NULL;
    for(int dim = 0; dim < 3; dim++){
        vox_num_(dim) = ceil((map_upbd_(dim) - origin_(dim)) / vox_scl_(dim));
    }
    H_depot_.resize(vox_num_(0) * vox_num_(1) * vox_num_(2));
    F_depot_.resize(vox_num_(0) * vox_num_(1) * vox_num_(2));
    cout<<"F_depot"<<F_depot_.size()<<"  "<<FG_->f_grid_.size()<<endl;
    cout<<vox_num_(0) * vox_num_(1) * vox_num_(2)<<endl;
    cout<<"vox_num_:"<<vox_num_.transpose()<<"  map_upbd_ - origin_:"<<(map_upbd_-origin_).transpose()<<endl;
    vector<uint16_t> id_l;
    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> bounds;
    for(int i = 0; i < FG_->f_grid_.size(); i++){
        F_depot_[i] = make_shared<F_node>();
        F_depot_[i]->cf_ = &(FG_->f_grid_[i]);
        F_depot_[i]->id_ = i;
        F_depot_[i]->center_ = FG_->f_grid_[i].center_;
        id_l.emplace_back(i);
        bounds.push_back({F_depot_[i]->cf_->up_, F_depot_[i]->cf_->down_});
    }
    BM_->InitSwarmBlock(id_l, bounds);
    map_upbd_ = LRM_->map_upbd_;

    root_ = NULL;
    cur_hid_ = uav_id_ + drone_num_;

    eurange_ = topo_range/2 + H_thresh_/2; //heuristic range for local topo search
    LRM_->SetEuRange(eurange_);
    LRM_->SetTopoRange(topo_range);
    LRM_->SetHthresh(H_thresh_); 

    topo_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/MR_DTG/Graph", 10);
    debug_pub_ = nh.advertise<visualization_msgs::Marker>(ns + "/MR_DTG/Debug", 10);
    show_timer_ = nh.createTimer(ros::Duration(0.2), &MultiDTG::ShowAll, this);
    if(drone_num_ > 1){
        use_swarm_ = true;
        swarm_timer_ = nh.createTimer(ros::Duration(SDM_->local_comm_intv_), &MultiDTG::DTGCommunicationCallback, this);
    }

    cout<<"DTG origin:"<<origin_.transpose()<<endl;
    cout<<"DTG topo_range:"<<topo_range<<endl;
    cout<<"DTG H_thresh_:"<<H_thresh_<<endl;
    cout<<"DTG eurange:"<<eurange_<<endl;
    cout<<"DTG update_FF_:"<<update_FF_<<endl;
    cout<<"DTG map_upbd_:"<<map_upbd_.transpose()<<endl;
    cout<<"DTG vox_num_:"<<vox_num_.transpose()<<endl;
    cout<<"DTG vox_scl_:"<<vox_scl_.transpose()<<endl;
    cout<<"DTG uav_id_:"<<uav_id_<<endl;
    cout<<"DTG drone_num_:"<<drone_num_<<endl;
}

void MultiDTG::GetHnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<h_ptr> &H_list){
    Eigen::Vector3i low, up, it;
    list<f_ptr> f_l_temp;
    list<h_ptr> h_l_temp;
    Eigen::Vector3d u, l;
    for(int dim = 0; dim < 3; dim++){
        u(dim) = min(upbd(dim), map_upbd_(dim) - 1e-3);
        l(dim) = max(lowbd(dim), origin_(dim) + 1e-3);
    }
    // if(!InsideMap(upbd) && !InsideMap(lowbd)) return;
    low = GetVoxId3(lowbd);
    up = GetVoxId3(upbd);
    for(int dim = 0; dim < 3; dim++){
        low(dim) = max(0, low(dim));
        up(dim) = min(vox_num_(dim) - 1, up(dim));
    }

    for(it(0) = low(0); it(0) <= up(0); it(0)++)
        for(it(1) = low(1); it(1) <= up(1); it(1)++)
            for(it(2) = low(2); it(2) <= up(2); it(2)++)
    {
        if(GetVox(it, h_l_temp, f_l_temp))
            H_list.insert(H_list.end(), h_l_temp.begin(), h_l_temp.end());
    }
}

void MultiDTG::GetFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd, list<f_ptr> &F_list){
    Eigen::Vector3i low, up, it;
    list<f_ptr> f_l_temp;
    list<h_ptr> h_l_temp;
    Eigen::Vector3d u, l;
    for(int dim = 0; dim < 3; dim++){
        u(dim) = min(upbd(dim), map_upbd_(dim) - 1e-3);
        l(dim) = max(lowbd(dim), origin_(dim) + 1e-3);
    }
    // if(!InsideMap(upbd) && !InsideMap(lowbd)) return;
    low = GetVoxId3(lowbd);
    up = GetVoxId3(upbd);
    for(int dim = 0; dim < 3; dim++){
        low(dim) = max(0, low(dim));
        up(dim) = min(vox_num_(dim) - 1, up(dim));
    }

    for(it(0) = low(0); it(0) <= up(0); it(0)++)
        for(it(1) = low(1); it(1) <= up(1); it(1)++)
            for(it(2) = low(2); it(2) <= up(2); it(2)++)
    {
        if(GetVox(it, h_l_temp, f_l_temp))
            F_list.insert(F_list.end(), f_l_temp.begin(), f_l_temp.end());
    }
}

void MultiDTG::GetHnodesAndFnodesBBX(const Eigen::Vector3d &upbd, const Eigen::Vector3d &lowbd,
    list<h_ptr> &H_list, list<f_ptr> &F_list){
    Eigen::Vector3i low, up, it;
    list<f_ptr> f_l_temp;
    list<h_ptr> h_l_temp;
    // if(!InsideMap(upbd) && !InsideMap(lowbd)) return;
    low = GetVoxId3(lowbd);
    up = GetVoxId3(upbd);
    for(int dim = 0; dim < 3; dim++){
        low(dim) = max(0, low(dim));
        up(dim) = min(vox_num_(dim) - 1, up(dim));
    }

    for(it(0) = low(0); it(0) <= up(0); it(0)++)
        for(it(1) = low(1); it(1) <= up(1); it(1)++)
            for(it(2) = low(2); it(2) <= up(2); it(2)++)
    {
        if(GetVox(it, h_l_temp, f_l_temp)){
        H_list.insert(H_list.end(), h_l_temp.begin(), h_l_temp.end());
        F_list.insert(F_list.end(), f_l_temp.begin(), f_l_temp.end());
        }
    }
}

void MultiDTG::GetLocalFnDetail(const Eigen::Vector3d &p, list<pair<double, pair<int, int>>> &d_f_v){
    Eigen::Vector3d upbd, lowbd;
    list<h_ptr> H_list_temp;
    list<pair<int, list<pair<int, Eigen::Vector3d>>>> frontier_list_;//<f_id, <v_id, viewpoint>>, check if the veiwpoint is in local space
    list<list<Eigen::Vector3d>> paths, pruned_paths;

    list<Eigen::Vector3d> vp_l; 
    list<pair<int, int>> vp_id_l; 
    list<double> vp_d_l; 
    Eigen::Vector3d v_pos;
    for(int dim = 0; dim < 3; dim++){
        upbd(dim) = p(dim) + eurange_;
        lowbd(dim) = p(dim) - eurange_;
    }
    FG_->GetWildGridsBBX(p, upbd - lowbd, frontier_list_);
    for(auto &f : frontier_list_){
        for(auto &vp : f.second){
            vp_l.emplace_back(vp.second);
            vp_id_l.push_back({f.first, vp.first});
        }
    }
    // cout<<"vp_l:"<<vp_l.size()<<endl;
    if(!LRM_->DjkstraLocalDist(p, paths, pruned_paths, vp_l, vp_d_l)) {
        ROS_WARN("id:%d fail djk", SDM_->self_id_);
        return;
    }

    list<pair<int, int>>::iterator vp_it = vp_id_l.begin();
    list<double>::iterator d_it = vp_d_l.begin();
    for(;d_it != vp_d_l.end(); d_it++, vp_it++){
        if(*d_it < 0) continue;
        d_f_v.push_back({*d_it, *vp_it});
    }
}

void MultiDTG::Update(const Eigen::Matrix4d &robot_pose, bool clear_x){
    Eigen::Vector3d robot_pos, upbd, lowbd;
    robot_pos = robot_pose.block(0, 3, 3, 1);
    list<hhe_ptr> hhe_free_list, hhe_occ_list;               
    list<hfe_ptr> hfe_free_list, hfe_occ_list;               
    list<pair<int, pair<int, f_ptr>>> C_F_list;    //connect idx & Fnode
    list<h_ptr> H_list_temp, L_H_list;
    list<f_ptr> F_list_temp;
    list<int> wild_f_list;
    if(!InsideMap(robot_pos)){
        ROS_WARN("id:%d out map! DTG refuse to update!", SDM_->self_id_);
        return;
    }

    LRM_->SetEternalNode(robot_pos);
    for(int dim = 0; dim < 3; dim++){
        upbd(dim) = robot_pos(dim) + sensor_range_;
        lowbd(dim) = robot_pos(dim) - sensor_range_;
    }

    GetHnodesAndFnodesBBX(upbd, lowbd, H_list_temp, F_list_temp);     
    if(FG_ != NULL){
        /* get local fn for djkstra */
        FG_->GetWildGridsBBX(robot_pos, upbd - lowbd, LRM_->frontier_list_);
        
        /* erase dead Fnodes */
        for(auto &f : F_list_temp){
            int idx = f->id_;
            if(f->cf_->f_state_ == 2){
                EraseFnode(f->center_, f->id_);
            }
            else if(f->vp_id_ != -1 && f->cf_->local_vps_[f->vp_id_] != 1){
                // BlockEdge(f->hf_edge_);
                EraseEdge(f->hf_edge_);
            }
        }
    }

    /* check if Hnode is blocked */
    for(auto &h : H_list_temp){
        if(h->state_ == BLOCKED && LRM_->CheckAddHnode(h->pos_, h->id_)){
            h->state_ = L_FREE;
        }
    }

    /* update lrm topo relationships */
    LRM_->prep_idx_ = cur_hid_;
    root_ = NULL;

    if(LRM_->UpdateLocalTopo(robot_pose, BM_->cur_pcl_, clear_x)){
        if(!LRM_->IsFeasible(robot_pos)){//debug
            ROS_WARN("id:%d Infeasible1! DTG refuse to update!", SDM_->self_id_);
            return;
        }
        
        //generate new Hnode
        root_ = CreateHnode(robot_pos);
        if(root_ == NULL){
            ROS_WARN("id:%d Infeasible2! DTG refuse to update!", SDM_->self_id_);
            return;
        }
    }
    else{
        if(!GetVoxBBX(LRM_->cur_root_h_idx_, LRM_->node_scale_, LRM_->cur_root_h_id_, root_)){
            cout<<LRM_->cur_root_h_idx_.transpose()<<":"<<LRM_->cur_root_h_id_<<";"<<LRM_->prep_idx_<<endl;
            ROS_ERROR("error root h!");
            ros::shutdown();
        }
    }

    /* load edges between the robot and frontier viewpoint */
    if(FG_ != NULL){
        Eigen::Vector3d vp_pos;
        shared_ptr<F_node> fn;
        r_f_edges_.clear();
        local_f_list_.clear();
        local_f_dist_list_.clear();

        for(auto &f_p : LRM_->frontier_path_){
            shared_ptr<ffe> r_e = make_shared<ffe>();
            r_e->head_n_ = NULL;
            bool gen_f = true;

            FG_->GetVpPos(f_p.first.first, f_p.second.first, vp_pos);
            fn = F_depot_[f_p.first.first];

            fn->vp_id_ = f_p.second.first;
            r_e->tail_n_ = fn;
            r_e->tail_ = fn->id_;
            r_e->path_ = f_p.second.second;
            r_e->path_.emplace_front(robot_pos);
            r_e->path_.emplace_back(vp_pos);
            local_f_list_.emplace_back(fn);
            local_f_dist_list_.emplace_back(f_p.first.second);
            r_f_edges_.emplace_back(r_e);

            //Get local Fnodes
            int c_id = LRM_->PostoId(vp_pos);
            //debug
            if(!LRM_->IsFeasible(vp_pos) || (LRM_->GetNode(c_id)==NULL)){
                cout<<LRM_->IsFeasible(vp_pos)<<"  "<<(LRM_->GetNode(c_id) == NULL)<<"   "<<(LRM_->GetNode(vp_pos) == NULL)<<endl;
                ROS_ERROR("error f vp");
                ros::shutdown();
                return;
            }
            C_F_list.push_back({c_id, {f_p.second.first, fn}});
        }
    }

    /* clear occ edges */
    for(auto &hid : LRM_->h_id_clear_){
        h_ptr h_clear;
        if(FindHnode(hid, h_clear)){
            for(auto &e : h_clear->hh_edges_){
                if(!(e->e_flag_ & 1)){
                    e->e_flag_ |= 1;
                    if(LRM_->PathCheck(e->path_)){
                        hhe_free_list.emplace_back(e);
                    }
                    else{
                        // ROS_WARN("id:%d clear edge1", SDM_->self_id_);
                        // cout<<"h:"<<e->head_<<"  t:"<<e->tail_<<endl;
                        hhe_occ_list.emplace_back(e);
                    }
                }
            }
            for(auto &e : h_clear->hf_edges_){
                if(!(e->e_flag_ & 1)){
                    e->e_flag_ |= 1;
                    if(LRM_->PathCheck(e->path_, true)){
                        hfe_free_list.emplace_back(e);
                    }
                    else{
                        // ROS_WARN("id:%d clear edge2", SDM_->self_id_);
                        // cout<<"h:"<<e->head_<<"  t:"<<e->tail_<<"PN:"<<e->path_.size()<<endl;
                        hfe_occ_list.emplace_back(e);
                    }
                }
            }
            if(!LRM_->IsFeasible(h_clear->pos_)){
                h_clear->state_ = BLOCKED;
            }
        }
        else{
            ROS_ERROR("h missed %d", int(hid));
        }
    }

    for(auto &e : hhe_free_list) e->e_flag_ &= 254;
    for(auto &e : hfe_free_list) e->e_flag_ &= 254;
    for(auto &e : hhe_occ_list) BlockEdge(e);
    for(auto &e : hfe_occ_list) BlockEdge(e);
    // for(auto &e : hhe_occ_list) EraseEdge(e);
    // for(auto &e : hfe_occ_list) EraseEdge(e);

    /* get local hn (inside djkstra area) */
    h_ptr c_h;
    for(auto &h : LRM_->id_Hpos_dist_){
        if(GetVoxBBX(h.second, LRM_->node_scale_, h.first, c_h)){
            L_H_list.push_back(c_h);
        }
        else{//debug
            cout<<h.second.transpose()<<":"<<int(h.first)<<endl;
            ROS_ERROR("error lhnode!");
            ros::shutdown();
        }
    }

    /* connect root and local hnodes */
    list<Eigen::Vector3d> path, path_h1, path_h2, path_prune;
    double length, length_h1, length_h2;
    local_h_list_.clear();
    local_h_dist_list_.clear();
    local_h_list_.emplace_back(root_);
    local_h_dist_list_.emplace_back(LRM_->GetRootHnCost());

    for(auto &h : L_H_list){
        if(LRM_->RetrieveHPath(h->pos_, root_->id_, path, length) && LRM_->PrunePath(path, path_prune, length)){
            reverse(path_prune.begin(), path_prune.end());
            if(root_ == h){
                ROS_ERROR("error connect1 %d %d", root_->id_, h->id_);
                ros::shutdown();
            }
            ConnectHH(root_, h, path_prune, length);
            // if((root_->pos_ - path_prune.front()).norm() > 1e-3) ROS_ERROR("error connect1");
            // if((h->pos_ - path_prune.back()).norm() > 1e-3) ROS_ERROR("error connect2");
            local_h_list_.emplace_back(h);
            local_h_dist_list_.emplace_back(length);
            // local_h_list_
        }
        else{
            ROS_ERROR("Error hpath0!");
            ros::shutdown();
        }
    }

    /* connect hands-shaking hnodes */
    for(auto &CH : LRM_->id_idx_dist_){
        if(LRM_->RetrieveHPath(CH.second.first, CH.first, path_h1, length_h1)){
            if(LRM_->RetrieveHPath(CH.second.first, root_->id_, path_h2, length_h2)){
                length = length_h1 + length_h2;
                path = path_h1;
                path.pop_front();
                for(auto &p : path_h2){
                    path.emplace_front(p);
                }
                if(LRM_->PrunePath(path, path_prune, length)){
                    if(GetVoxBBX(path.back(), LRM_->node_scale_, CH.first, c_h)){//debug
                        if(root_ == c_h){
                            ROS_ERROR("error connect2 %d %d", root_->id_, c_h->id_);
                            ros::shutdown();
                        }
                        ConnectHH(root_, c_h, path_prune, length);
                        // if((root_->pos_ - path_prune.front()).norm() > 1e-3) ROS_ERROR("error connect3");
                        // if((c_h->pos_ - path_prune.back()).norm() > 1e-3) ROS_ERROR("error connect4");
                    }
                    else{
                        if(FindHnode(CH.first, c_h)){
                            cout<<"id:"<<CH.first<<"pos:"<<c_h->pos_.transpose()<<endl;
                            cout<<path.front().transpose()<<"p:"<<path.back().transpose()<<endl;
                        }
                        ROS_ERROR("Error Hnode!");
                        ros::shutdown();
                    }
                }
                else{
                    ROS_ERROR("error hpath1.5!");
                    ros::shutdown();
                }
            }
            else{//debug
                cout<<"root:"<<int(root_->id_)<<"  h:"<<int(CH.first)<<endl;
                cout<<LRM_->id_idx_dist_.size()<<"  "<<CH.second.first<<endl;
                ROS_ERROR("error hpath2!");
                ros::shutdown();
            }
        }
        else{//debug
            ROS_ERROR("error hpath1!");
            ros::shutdown();
        }
    }

    /* connect fnodes and rootH */
    list<Eigen::Vector3d> path_f;
    double length_f;
    for(auto &CF : C_F_list){
        if(LRM_->RetrieveHPath(CF.first, root_->id_, path_f, length_f) && LRM_->PrunePath(path_f, path_prune, length_f)){

            reverse(path_prune.begin(), path_prune.end());
            Eigen::Vector3d vp_pos;
            //debug
            if(!FG_->GetVpPos(CF.second.second->id_, CF.second.first, vp_pos)){
                ROS_ERROR("error fpath2!");
                ros::shutdown();
            }
            path_prune.push_back(vp_pos);
            ConnectHF(root_, CF.second.second, CF.second.first, path_prune, length_f);
        }
        else{//debug
            ROS_ERROR("error fpath!");
            ros::shutdown();
        }
    }
}

void MultiDTG::RemoveVp(const Eigen::Vector3d &center, int const &f_id, int const &v_id, bool broad_cast){
    f_ptr fn;
    /* remove vp in fronter grid */
    FG_->RemoveVp(f_id, v_id, broad_cast);
    if(SDM_->is_ground_) return;

    /* remove local edge */
    if(f_id < F_depot_.size()) fn = F_depot_[f_id];
    // if(!FindFnode(center, f_id, fn)) return;
    if(v_id == fn->vp_id_){
        for(list<ffe_ptr>::iterator it = r_f_edges_.begin(); it != r_f_edges_.end(); it++){
            if((*it)->tail_ == f_id){
                (*it)->e_flag_ &= 239;
                r_f_edges_.erase(it);
                break;
            }
        }
        BlockEdge(fn->hf_edge_);
    }
    else return;

    double best_l, cur_l;
    int best_v_id;
    uint32_t target_h_id, best_h_id;
    list<Eigen::Vector3d> best_path;
    Eigen::Vector3d vp_pos, best_vp_pos;
    best_l = 99999.0;
    for(int v_id = 0; v_id < fn->cf_->local_vps_.size(); v_id++){
        if(fn->cf_->local_vps_[v_id] == 1 && FG_->GetVpPos(f_id, v_id, vp_pos) && 
                LRM_->ShortestLength2Root(vp_pos, target_h_id, cur_l) && cur_l < best_l){
            best_l = cur_l;
            best_h_id = target_h_id;
            best_v_id = v_id;
            best_vp_pos = vp_pos;
        }
    }


    list<Eigen::Vector3d> path, path_prune;
    double length_f;
    if(best_l < 99998.0 && LRM_->RetrieveHPath(best_vp_pos, best_h_id, path, length_f) && LRM_->PrunePath(path, path_prune, length_f)){
        reverse(path_prune.begin(), path_prune.end());
        h_ptr hn;
        if(!FindHnode(path_prune.front(), best_h_id, hn)){
            ROS_ERROR("Error RemoveVp Hnode!");
            cout<<"vp:"<<best_vp_pos.transpose()<<endl;

            cout<<"hid:"<<best_h_id<<"  pp pos:"<<path_prune.front().transpose()<<endl;
            cout<<"hid:"<<best_h_id<<"  p pos:"<<path.front().transpose()<<"   "<<path.back().transpose()<<endl;
            LRM_->DebugTie(path_prune.front());
            if(FindHnode(best_h_id, hn)){
                cout<<"h pos:"<<hn->pos_.transpose()<<"   "<<LRM_->IsFeasible(hn->pos_)<<endl;
                LRM_->DebugTie(hn->pos_);
            }
            ros::shutdown();
            return;
        }
        Eigen::Vector3d vp_pos;
        FG_->GetVpPos(f_id, best_v_id, vp_pos);
        path_prune.push_back(vp_pos);
        ConnectHF(hn, fn, best_v_id, path_prune, length_f);   
        // ROS_WARN("rewire h:%d f:%d flag:%d", hn->id_, fn->id_, fn->hf_edge_->e_flag_);
    }
    else if(broad_cast){
        SDM_->SetDTGFn(f_id, F_depot_[f_id]->cf_->local_vps_, 1, F_depot_[f_id]->cf_->f_state_ != 2);
    }
}

ffe_ptr MultiDTG::GetClosestLocalTarget(){
    ffe_ptr best_e = NULL;
    list<Eigen::Vector3d> pruned_path;
    double best_l, l;
    best_l = 99999.0;
    for(auto &e : r_f_edges_){
        if(e == NULL){
            ROS_ERROR("GetClosestLocalTarget: e == NULL");
            ros::shutdown();
        }
        if(LRM_->PrunePath(e->path_, pruned_path, l)){
            if(l < best_l){
                best_e = e;
                best_l = l;
            }
        }
        else{
            continue;
        }
    }
    return best_e;
}

void MultiDTG::GetAllLocalFroVps(list<int> &f_l){
    f_l.clear();
    for(auto &rfe : r_f_edges_){
        int f_id = rfe->tail_n_->id_;
        if(f_id < 0 || f_id >= F_depot_.size()) continue;
        if(F_depot_[f_id]->cf_->f_state_ == 1) f_l.emplace_back(f_id);
    }
}

void MultiDTG::DTGCommunicationCallback(const ros::TimerEvent &e){
    // ROS_WARN("id:%d DTGCommunicationCallback", SDM_->self_id_);
    /* update Fn */
    vector<uint8_t> vp_states;
    while (!SDM_->swarm_sub_fn_.empty())
    {
        auto &f_msg = SDM_->swarm_sub_fn_.front();
        if(0 > f_msg.f_id || F_depot_.size() <= f_msg.f_id){
            cout<<"invalid fn:"<<int(f_msg.f_id)<<" max:"<<F_depot_.size()<<endl;
        }
        else{
            if(F_depot_[f_msg.f_id]->cf_->f_state_ != 2){
                if(f_msg.alive){
                    SDM_->GetFnvp(vp_states, f_msg.vp_flags);
                    if(F_depot_[f_msg.f_id]->cf_->f_state_ == 0) F_depot_[f_msg.f_id]->cf_->f_state_ = 1;
                    for(int i = 0; i < vp_states.size(); i++){
                        if(vp_states[i] == 2 && F_depot_[f_msg.f_id]->cf_->local_vps_[i] == 1){
                            RemoveVp(F_depot_[f_msg.f_id]->cf_->center_, f_msg.f_id, i, false);
                        }
                        else if(vp_states[i] == 2 && F_depot_[f_msg.f_id]->cf_->local_vps_[i] == 0){
                            RemoveVp(F_depot_[f_msg.f_id]->cf_->center_, f_msg.f_id, i, false);
                        }
                        else if(vp_states[i] == 1 && F_depot_[f_msg.f_id]->cf_->local_vps_[i] == 0){
                            F_depot_[f_msg.f_id]->cf_->local_vps_[i] = 1;
                        }
                    }
                    int alive_num = 0;
                    if(F_depot_[f_msg.f_id]->cf_->f_state_ != 2){
                        for(auto &vs : F_depot_[f_msg.f_id]->cf_->local_vps_) if(vs != 2) alive_num++;
                        if(alive_num < FG_->min_vp_num_){
                            
                            FG_->SetExplored(f_msg.f_id);
                            if(!SDM_->is_ground_) SDM_->SetDTGFn(f_msg.f_id, F_depot_[f_msg.f_id]->cf_->local_vps_, 1, false);
                        }
                        else FG_->AddShow(f_msg.f_id);
                    }
                    
                    if(f_msg.need_help && F_depot_[f_msg.f_id]->cf_->f_state_ == 1 && 
                        F_depot_[f_msg.f_id]->hf_edge_ != NULL && F_depot_[f_msg.f_id]->hf_edge_->e_flag_ & 16){
                        vector<uint32_t> path;
                        int p_idx;
                        for(auto &p : F_depot_[f_msg.f_id]->hf_edge_->path_){
                            p_idx = LRM_->PostoId(p);
                            if(path.empty() || p_idx != path.back())
                                path.push_back(p_idx);
                        }
                        SDM_->SetDTGHFEdge(F_depot_[f_msg.f_id]->hf_edge_->head_, 
                                F_depot_[f_msg.f_id]->hf_edge_->tail_, F_depot_[f_msg.f_id]->vp_id_, path);
                    }

                }
                else{
                    // cout<<"kill:"<<int(f_msg.f_id)<<endl;
                    if(f_msg.need_help){
                        BM_->SendSwarmBlockMap(f_msg.f_id, false);
                    }
                    FG_->SetExplored(f_msg.f_id);
                    // debug_pts_.emplace_back(F_depot_[f_msg.f_id]->cf_->center_);
                    // Debug(debug_pts_);
                    EraseFnode(F_depot_[f_msg.f_id]->cf_->center_, f_msg.f_id);
                }
            }
        }
        SDM_->swarm_sub_fn_.pop_front();
    }

    /* update Hn */
    h_ptr hn;
    while (!SDM_->swarm_sub_hn_.empty()){
        auto &h_msg = SDM_->swarm_sub_hn_.front();
        Eigen::Vector3d pos = LRM_->IdtoPos(h_msg.pos_idx);
        if(FindHnode(pos, h_msg.h_id, hn)){
            ROS_WARN("id:%d exists%d", SDM_->self_id_, h_msg.h_id);
        }
        else{
            CreateSwarmHnode(pos, h_msg.h_id);
        }
        SDM_->swarm_sub_hn_.pop_front();
    }

    /* update HFe */
    for(list<swarm_exp_msgs::DtgHFEdge>::iterator hfe_it = SDM_->swarm_sub_hfe_.begin(); hfe_it != SDM_->swarm_sub_hfe_.end(); hfe_it++){
        if(hfe_it->f_id < F_depot_.size()){
            list<Eigen::Vector3d> path;
            h_ptr hn_head;
            f_ptr fn_tail = F_depot_[hfe_it->f_id];
            Eigen::Vector3d h_pos, t_pos;
            h_pos = LRM_->IdtoPos(hfe_it->points_idx.front());
            t_pos = fn_tail->cf_->center_;
            if(FindHnode(h_pos, hfe_it->h_id, hn_head) && hfe_it->f_id < F_depot_.size()){
                // if(!hfe_it->erase){
                list<Eigen::Vector3d> path;
                for(auto &p : hfe_it->points_idx) path.emplace_back(LRM_->IdtoPos(p));
                if(!ConnectSwarmHF(hn_head, fn_tail, path, hfe_it->vp_id)){
                    // ROS_ERROR("id:%d error swarm connect hf", SDM_->self_id_);
                }
                // }
                // else{
                //     EraseEdge(fn_tail->hf_edge_, false);
                // }
                list<swarm_exp_msgs::DtgHFEdge>::iterator erase_it = hfe_it;
                hfe_it--;
                SDM_->swarm_sub_hfe_.erase(erase_it);
            }
            else{
                ROS_WARN("id:%d fail find hn fn, dont connect", SDM_->self_id_);
                cout<<int(hfe_it->h_id)<<"  pos:"<<h_pos.transpose()<<"  "<<FindHnode(h_pos, hfe_it->h_id, hn_head)<<endl;
                cout<<int(hfe_it->f_id)<<endl;
                cout<<int(fn_tail->cf_->f_state_)<<" gt pos:"<<fn_tail->cf_->center_.transpose()<<endl;
            }
        }
        else{
            list<swarm_exp_msgs::DtgHFEdge>::iterator erase_it = hfe_it;
            hfe_it--;
            SDM_->swarm_sub_hfe_.erase(erase_it);
        }
    }

    /* update HHe */
    for(list<swarm_exp_msgs::DtgHHEdge>::iterator hhe_it = SDM_->swarm_sub_hhe_.begin(); hhe_it != SDM_->swarm_sub_hhe_.end(); hhe_it++){
        h_ptr hn_head, hn_tail;
        Eigen::Vector3d h_pos, t_pos;
        h_pos = LRM_->IdtoPos(hhe_it->points_idx.front());
        t_pos = LRM_->IdtoPos(hhe_it->points_idx.back());
        if(FindHnode(h_pos, hhe_it->head_h_id, hn_head) && FindHnode(t_pos, hhe_it->tail_h_id, hn_tail)){
            list<Eigen::Vector3d> path;
            // if(!hhe_it->erase){
            for(auto &p : hhe_it->points_idx) path.emplace_back(LRM_->IdtoPos(p));
            if(!ConnectSwarmHH(hn_head, hn_tail, path)){
                // ROS_ERROR("fail hh");
            }
            // else{
            //     // ROS_WARN("success connect %d, %d", hn_head->id_, hn_tail->id_);
            // }
            // }
            // else{
            //     for(auto &e : hn_head->hh_edges_){
            //         if((e->head_ == hn_head->id_ && e->tail_ == hn_tail->id_) ||
            //             (e->tail_ == hn_head->id_ && e->head_ == hn_tail->id_)){
            //             EraseEdge(e, false);
            //         }
            //     }
            // }
            list<swarm_exp_msgs::DtgHHEdge>::iterator erase_it = hhe_it;
            hhe_it--;
            SDM_->swarm_sub_hhe_.erase(erase_it);
        }
        else{
            ROS_WARN("id:%d fail find hn hn, dont connect", SDM_->self_id_);
            cout<<int(hhe_it->head_h_id)<<"  pos:"<<h_pos.transpose()<<FindHnode(h_pos, hhe_it->head_h_id, hn_head)<<endl;
            cout<<int(hhe_it->tail_h_id)<<"  pos:"<<t_pos.transpose()<<FindHnode(t_pos, hhe_it->tail_h_id, hn_tail)<<endl;
        }
    }
    if(SDM_->swarm_sub_hhe_.size() > 0){//debug
        ROS_ERROR("error hhe swarm");
    }


}

void MultiDTG::ShowAll(const ros::TimerEvent &e){
    Show();
}

void MultiDTG::Show(){
    visualization_msgs::MarkerArray mka;
    geometry_msgs::Point p1, p2;
    list<hhe_ptr> hh_e_list;
    list<hfe_ptr> hf_e_list;
    list<ffe_ptr> ff_e_list;
    mka.markers.resize(6);

    // for(auto &f_l : F_depot_){
    //     f_n_list.insert(f_n_list.end(), f_l.begin(), f_l.end());
    // }

    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 0;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.25;
    mka.markers[0].scale.y = 0.25;
    mka.markers[0].scale.z = 0.25;
    mka.markers[0].color.a = 0.8;
    // cout<<"show????????"<<H_list_.size()<<endl;

    for(auto &h : H_list_){
        p1.x = h->pos_(0);
        p1.y = h->pos_(1);
        p1.z = h->pos_(2);
        // cout<<"show!!!!!!!!!!!!!!!!!!!"<<h->pos_.transpose()<<endl;
        if(h->h_flags_ & 4)
            mka.markers[0].colors.push_back(CM_->Id2Color(SDM_->self_id_, 1.0));
        else 
            mka.markers[0].colors.push_back(CM_->Id2Color(0, 0.1));
        mka.markers[0].points.emplace_back(p1);
        for(auto &e : h->hh_edges_){
            if(e->e_flag_ & 2) continue;
            else {
                e->e_flag_ |= 2;
                hh_e_list.emplace_back(e);
            }
        }
        // for(auto &e : h->hf_edges_){
            
            hf_e_list.insert(hf_e_list.end(), h->hf_edges_.begin(), h->hf_edges_.end()); 
        // }
    }

    mka.markers[1].header.frame_id = "world";
    mka.markers[1].header.stamp = ros::Time::now();
    mka.markers[1].id = 1;
    mka.markers[1].action = visualization_msgs::Marker::ADD;
    mka.markers[1].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[1].scale.x = 0.25;
    mka.markers[1].scale.y = 0.25;
    mka.markers[1].scale.z = 0.25;
    mka.markers[1].color.a = 0.4;
    mka.markers[1].color.b = 0.8;
    mka.markers[1].color.g = 0.2;
    mka.markers[1].color.r = 0.2;
    for(auto &f : hf_e_list){
        // p1.x = f->tail_n_->center_(0);
        // p1.y = f->tail_n_->center_(1);
        // p1.z = f->tail_n_->center_(2);
        // mka.markers[1].points.emplace_back(p1);
        // for(auto &e : f->tail_n_->edges_){
        //     if(e->flag_ & 2) continue;
        //     else {
        //         e->flag_ |= 2;
        //         ff_e_list.emplace_back(e);
        //     }
        // }
    }

    mka.markers[2].header.frame_id = "world";
    mka.markers[2].header.stamp = ros::Time::now();
    mka.markers[2].id = 2;
    mka.markers[2].action = visualization_msgs::Marker::ADD;
    mka.markers[2].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[2].scale.x = 0.03;
    mka.markers[2].scale.y = 0.03;
    mka.markers[2].scale.z = 0.03;
    mka.markers[2].color.a = 0.4;
    mka.markers[2].color.r = 0.6;
    mka.markers[2].color.g = 0.2;
    mka.markers[2].color.b = 0.5;

    // cout<<"show!!!!!!!!!!!!!!   "<<hh_e_list.size()<<endl;
    for(auto &e : hh_e_list){
        e->e_flag_ &= 253;
        // if(e->flag_ & 32)
        // if(show_e_details_){
        //     if(e->path_.size() <= 1) continue;
        //     for(list<Eigen::Vector3d>::iterator p_it = e->path_.begin(); p_it != e->path_.end(); p_it++){
        //         p1.x = p_it->x();
        //         p1.y = p_it->y();
        //         p1.z = p_it->z();
        //         // cout<<(*p_it).transpose()<<endl;
        //         p_it++;
        //         p2.x = p_it->x();
        //         p2.y = p_it->y();
        //         p2.z = p_it->z();
        //         // cout<<(*p_it).transpose()<<endl;
        //         mka.markers[2].points.emplace_back(p1);
        //         mka.markers[2].points.emplace_back(p2);
        //         p_it++;
        //         if(p_it == e->path_.end()) break;
        //         p_it--;
        //         p_it--;
        //     }
        // }
        // else{
        //     if(e->path_.size() <= 1) continue;
        //     p1.x = e->path_.front().x();
        //     p1.y = e->path_.front().y();
        //     p1.z = e->path_.front().z();
        //     p2.x = e->path_.back().x();
        //     p2.y = e->path_.back().y();
        //     p2.z = e->path_.back().z();
        //     mka.markers[2].points.emplace_back(p1);
        //     mka.markers[2].points.emplace_back(p2);
        // }
    }

    mka.markers[3].header.frame_id = "world";
    mka.markers[3].header.stamp = ros::Time::now();
    mka.markers[3].id = 3;
    mka.markers[3].action = visualization_msgs::Marker::ADD;
    mka.markers[3].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[3].scale.x = 0.03;
    mka.markers[3].scale.y = 0.03;
    mka.markers[3].scale.z = 0.03;
    mka.markers[3].color.a = 0.2;
    mka.markers[3].color.r = 0.7;
    mka.markers[3].color.g = 0.5;
    mka.markers[3].color.b = 0.2;
    // cout<<"debug show"<<endl;
    // tr1::unordered_map<int, int> debug_dict;
    for(auto &e : hf_e_list){
        // if(debug_dict.find(e->tail_) == debug_dict.end())
        //     debug_dict.insert({e->tail_, e->tail_});
        // else {
        //     ROS_ERROR("id:%d dup edge fn:%d", SDM_->self_id_, e->tail_);
        //     ros::shutdown();
        //     return;
        // }
        if(!(e->e_flag_ & 16)) continue;
        // cout<<"h:"<<e->head_<<" f:"<<e->tail_<<endl;
        // e->flag_ &= 253;


        if(show_e_details_){
            // if(e->path_.size() <= 1) continue;
            // for(list<Eigen::Vector3d>::iterator p_it = e->path_.begin(); p_it != e->path_.end(); p_it++){
            //     p1.x = p_it->x();
            //     p1.y = p_it->y();
            //     p1.z = p_it->z();
            //     p_it++;
            //     p2.x = p_it->x();
            //     p2.y = p_it->y();
            //     p2.z = p_it->z();
            //     p_it--;
            //     mka.markers[3].points.emplace_back(p1);
            //     mka.markers[3].points.emplace_back(p2);
            // }
            // p1.x = e->path_.back().x();
            // p1.y = e->path_.back().y();
            // p1.z = e->path_.back().z();
            p1.x = e->path_.front().x();
            p1.y = e->path_.front().y();
            p1.z = e->path_.front().z();
            p2.x = e->tail_n_->center_.x();
            p2.y = e->tail_n_->center_.y();
            p2.z = e->tail_n_->center_.z();
            mka.markers[3].points.emplace_back(p1);
            mka.markers[3].points.emplace_back(p2);
        }
        else{
            p1.x = e->path_.front().x();
            p1.y = e->path_.front().y();
            p1.z = e->path_.front().z();

            p2.x = e->path_.back().x();
            p2.y = e->path_.back().y();
            p2.z = e->path_.back().z();

            mka.markers[3].points.emplace_back(p1);
            mka.markers[3].points.emplace_back(p2);

        }

    }

    mka.markers[4].header.frame_id = "world";
    mka.markers[4].header.stamp = ros::Time::now();
    mka.markers[4].id = 4;
    mka.markers[4].action = visualization_msgs::Marker::ADD;
    mka.markers[4].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[4].scale.x = 0.03;
    mka.markers[4].scale.y = 0.03;
    mka.markers[4].scale.z = 0.03;
    mka.markers[4].color.a = 0.6;
    mka.markers[4].color.r = 1.0;
    mka.markers[4].color.g = 0.5;
    mka.markers[4].color.b = 0.0;
    for(auto &e : hh_e_list){
        if(e->path_.size() <= 1) continue;
        p1.x = e->path_.front().x();
        p1.y = e->path_.front().y();
        p1.z = e->path_.front().z();
        p2.x = e->path_.back().x();
        p2.y = e->path_.back().y();
        p2.z = e->path_.back().z();
        mka.markers[4].points.emplace_back(p1);
        mka.markers[4].points.emplace_back(p2);

    }

    mka.markers[5].header.frame_id = "world";
    mka.markers[5].header.stamp = ros::Time::now();
    mka.markers[5].id = 5;
    mka.markers[5].action = visualization_msgs::Marker::ADD;
    mka.markers[5].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[5].scale.x = 0.03;
    mka.markers[5].scale.y = 0.03;
    mka.markers[5].scale.z = 0.03;
    mka.markers[5].color.a = 0.80;
    mka.markers[5].color.r = 0.25;
    mka.markers[5].color.g = 0.55;
    mka.markers[5].color.b = 0.30;

    for(auto &re : r_f_edges_){
        // if(show_e_details_){
        //     if(re->path_.size() <= 1) continue;
        //     for(list<Eigen::Vector3d>::iterator p_it = re->path_.begin(); p_it != re->path_.end(); p_it++){
        //         p1.x = p_it->x();
        //         p1.y = p_it->y();
        //         p1.z = p_it->z();
        //         p_it++;
        //         p2.x = p_it->x();
        //         p2.y = p_it->y();
        //         p2.z = p_it->z();
        //         p_it--;
        //         mka.markers[5].points.emplace_back(p1);
        //         mka.markers[5].points.emplace_back(p2);
        //     }
        // }
        // else{
        //     p1.x = re->path_.front().x();
        //     p1.y = re->path_.front().y();
        //     p1.z = re->path_.front().z();
        //     p2.x = re->path_.back().x();
        //     p2.y = re->path_.back().y();
        //     p2.z = re->path_.back().z();
        //     mka.markers[5].points.emplace_back(p1);
        //     mka.markers[5].points.emplace_back(p2);
        // }
        // p1.x = re->path_.back().x();
        // p1.y = re->path_.back().y();
        // p1.z = re->path_.back().z();
        // p2.x = re->tail_n_->center_.x();
        // p2.y = re->tail_n_->center_.y();
        // p2.z = re->tail_n_->center_.z();
        // mka.markers[5].points.emplace_back(p1);
        // mka.markers[5].points.emplace_back(p2);
    }
    int i = 0;
    for(auto &mk : mka.markers){
        if(mk.points.size() == 0) mk.action = visualization_msgs::Marker::DELETE;
        i++;
    }
    topo_pub_.publish(mka);
}

void MultiDTG::Debug(){
    for(auto &h_l : H_depot_){
        for(auto &h : h_l){
            for(auto &e : h->hh_edges_){
                h_ptr t;
                if(e->head_n_ == h){
                    t = e->tail_n_;
                }
                else{
                    t = e->head_n_;
                }
                bool have_edge = false;
                for(auto &e2 : t->hh_edges_){
                    if(e2 == e) {
                        have_edge = true;
                        break;
                    }
                }
                if(!have_edge){
                    ROS_ERROR("error edge");
                    cout<<e->head_<<" "<<e->tail_<<endl;
                }
            }
        }
    }
}

void MultiDTG::Debug(list<Eigen::Vector3d> &fl){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.color.a = 0.8;
    mk.color.b = 0.8;
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
