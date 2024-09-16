#include <multiDTG/multiDTG.h>
using namespace DTG;

void MultiDTG::RetrieveHFPath(f_ptr &tar_f, list<h_ptr> &h_path, list<Eigen::Vector3d> &path, double &length){
    path.clear();
    h_path.clear();
    length = 0;
    h_ptr cur_h = tar_f->hf_edge_->head_n_;
    h_ptr par_h;
    path.insert(path.end(), tar_f->hf_edge_->path_.begin(), tar_f->hf_edge_->path_.end());
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            ROS_ERROR("error get hnode RetrieveHFPath");
            ros::shutdown();
            return;
        }
        for(auto &hhe : cur_h->hh_edges_){
            if(hhe->head_n_ == par_h){
                for (auto pit = hhe->path_.rbegin(); pit != hhe->path_.rend(); ++pit) {
                    path.emplace_front(*pit);
                }
                break;
            }
            else if(hhe->tail_n_ == par_h){
                for (auto pit = hhe->path_.begin(); pit != hhe->path_.end(); ++pit) {
                    path.emplace_front(*pit);
                }
                break;
            }
        }
        h_path.emplace_front(cur_h);
        cur_h = par_h;
    }
    h_path.emplace_front(cur_h);
}

void MultiDTG::RetrieveHPath(h_ptr &start_h, h_ptr &tar_h, list<Eigen::Vector3d> &path, double &length){
    path.clear();
    length = tar_h->sch_node_->g_;
    h_ptr cur_h = tar_h;
    h_ptr par_h;
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            if(start_h->id_ != cur_h->sch_node_->parent_->id_){
                ROS_ERROR("error get hnode RetrieveHPath");
                ros::shutdown();
                return;
            }
            else{
                par_h = start_h;
            }
        }
        bool debug_flag = true;
        for(auto &hhe : par_h->hh_edges_){
            if(hhe->tail_n_ == cur_h){
                for (auto pit = hhe->path_.rbegin(); pit != hhe->path_.rend(); ++pit) {
                    path.emplace_front(*pit);
                }
                debug_flag = false;
                break;
            }
            else if(hhe->head_n_ == cur_h){
                for (auto pit = hhe->path_.begin(); pit != hhe->path_.end(); ++pit) {
                    path.emplace_front(*pit);
                }
                debug_flag = false;
                break;
            }
        }
        if(debug_flag){
            ROS_ERROR("kidding??");
        }
        cur_h = par_h;
    }
}

void MultiDTG::RetrieveHDebug(h_ptr &start_h, h_ptr &tar_h, list<h_ptr> &h_path){
    h_path.clear();
    h_ptr cur_h = tar_h;
    h_ptr par_h;
    while(cur_h->sch_node_->parent_ != NULL){
        if(!FindHnode(cur_h->sch_node_->parent_->pos_, cur_h->sch_node_->parent_->id_, par_h)){
            if(start_h->id_ != cur_h->sch_node_->parent_->id_){
                ROS_ERROR("error get hnode RetrieveHPath");
                ros::shutdown();
                return;
            }
            else{
                par_h = start_h;
            }
        }
        h_path.emplace_front(cur_h);
        cout<<"cur_h->id_:"<<int(cur_h->id_)<<endl;
        cur_h = par_h;
    }
}

void MultiDTG::ClearSearched(list<h_ptr> &h_l){
    for(auto &h : h_l) h->sch_node_ = NULL;
}

void MultiDTG::ClearSearched(list<f_ptr> &f_l){
    for(auto &f : f_l) f->sch_node_ = NULL;
}

bool MultiDTG::GetClosestGlobalTarget(list<Eigen::Vector3d> &path, list<h_ptr> &H_path, int &f_id, int &v_id, double &length){
    if(root_ == NULL) return false;
    list<h_ptr> searched_h;
    list<f_ptr> searched_f;
    h_ptr hc, hn;
    f_ptr fn;

    prio_D empty_set;
    open_D_.swap(empty_set);
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    searched_h.emplace_back(root_);
    root_->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, root_->id_, root_->pos_);
    cur_n = root_->sch_node_;
    cur_n->flag_ = 1;
    open_D_.push(cur_n);
    while(!open_D_.empty()){
        cur_n = open_D_.top();
        // if(cur_n->g_ + 0.011 < debug_d_) {
        //     ROS_ERROR("error g%f!  %f", cur_n->g_ + 0.1,debug_d_);
        //     cout<<int(cur_n->flag_)<<endl;
        // }
        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        // debug_d_ = max(debug_d_, cur_n->g_);
        cur_n->flag_ |= 2;
        if((cur_n->flag_ & 1) && !FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode GetClosestGlobalTarget");
            ros::shutdown();
            return false;
        }

        /* find h with f edges, success */
        if(!(cur_n->flag_ & 1)){
            double length_e = 99999.0;
            f_ptr tar_f = F_depot_[cur_n->id_];
            // if(!FindFnode(cur_n->pos_, cur_n->id_, tar_f)){
            //     cout<<cur_n->pos_.transpose()<<"     "<< cur_n->id_<<endl;
            //     ROS_ERROR("how fail finding f???");
            // }

            f_id = tar_f->id_;
            v_id = tar_f->vp_id_;
            // cout<<"global g:"<<cur_n->g_<<endl;
            // while(!open_D_.empty()){
            //     cur_n = open_D_.top();
            //     open_D_.pop();
            //     if(cur_n->g_ + 0.11 < debug_d_) {
            //         ROS_ERROR("finish error g%f!  %f", cur_n->g_, debug_d_);
            //         cout<<int(cur_n->flag_)<<endl;
            //     }
            // }
            RetrieveHFPath(tar_f, H_path, path, length_e);
            ClearSearched(searched_h);
            ClearSearched(searched_f);
            return true;
        }
        // if(hc->hf_edges_.size() > 0){
        //     double length_e = 99999.0;
        //     f_ptr tar_f;
        //     for(auto &hfe : hc->hf_edges_){
        //         if(hfe->length_ < length_e){
        //             f_id = hfe->tail_;
        //             v_id = hfe->tail_n_->vp_id_;
        //             tar_f = hfe->tail_n_;
        //             length_e = hfe->length_;
        //         }
        //     }
        //     if(length_e < 99998.0){
        //         /* get the whole path */
        //         RetrieveHFPath(tar_f, H_path, path, length_e);
        //         ClearSearched(searched_h);
        //         return true;
        //     }
        //     else{
        //         ROS_ERROR("error get fnode GetClosestGlobalTarget");
        //         ros::shutdown();
        //         return false;
        //     }
        // }

        // ROS_WARN("exp");
        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            // if(!(hhe->flag_ & 8)) continue;

            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;
            // cout<<"h --> h"<<int(hc->id_)<<"  "<<int(hn->id_)<<"  "<<hhe->length_<<endl;
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;

                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + hhe->length_ + 1e-3 < nei_n->g_){
                    // nei_n->parent_ = cur_n;
                    // nei_n->g_ = cur_n->g_ + hhe->length_;
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hhe->length_, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }

        /* expand Fneighbours */
        for(auto &hfe : hc->hf_edges_){
            // if(!(hfe->flag_ & 8)) continue;
            fn = hfe->tail_n_;            
            // cout<<"h --> f"<<int(hc->id_)<<"  "<<int(fn->id_)<<"  "<<hfe->length_<<endl;

            if(fn->sch_node_ == NULL){
                /* new node */
                fn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + hfe->length_, 0.0, fn->id_, fn->center_);
                nei_n = fn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->id_ = fn->id_;
                nei_n->g_ = cur_n->g_ + hfe->length_;
                nei_n->flag_ = 0;
                searched_f.push_back(fn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(fn->sch_node_->flag_ & 2) continue;
                ROS_WARN("id:%d how double f GetClosestGlobalTarget", SDM_->self_id_);
                /* try to change parent */
                nei_n = fn->sch_node_;
                if(cur_n->g_ + hfe->length_ + 1e-3 < nei_n->g_){
                    nei_n->parent_ = cur_n;
                    nei_n->g_ = cur_n->g_ + hfe->length_;
                }
            }
        }

    }

    return false;
}

void MultiDTG::GetGlobalTarget(h_ptr &s_hn, vector<pair<double, list<Eigen::Vector3d>>> &paths, vector<h_ptr> &t_hn/*,  vector<list<h_ptr>> &debug_paths*/){
    h_ptr hc, hn;
    f_ptr fn;
    list<h_ptr> searched_h;
    list<f_ptr> searched_f;
    int reach_count = 0;
    // debug_paths.resize(t_hn.size());
    paths.resize(t_hn.size());
    for(auto &p : paths) p.first = -1.0;

    prio_D empty_set;
    open_D_.swap(empty_set);
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    searched_h.emplace_back(s_hn);
    s_hn->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, s_hn->id_, s_hn->pos_);
    cur_n = s_hn->sch_node_;
    cur_n->flag_ = 1;
    open_D_.push(cur_n);

    while(!open_D_.empty()){
        cur_n = open_D_.top();
        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;

        if(cur_n->id_ > SDM_->drone_num_ && (cur_n->flag_ & 1) && !FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode GetGlobalTarget");
            cout<<"cur_n->id_:"<<int(cur_n->id_)<<endl;
            ros::shutdown();
            return;
        }
        else if(cur_n->id_ == s_hn->id_){
            hc = s_hn;
        }

        /* reach target? */
        for(int i = 0; i < t_hn.size(); i++){
            if(t_hn[i]->id_ == hc->id_){
                reach_count++;
                /* retrieve path */
                RetrieveHPath(s_hn, hc, paths[i].second, paths[i].first);
                // RetrieveHDebug(s_hn, hc, debug_paths[i]);
                if(reach_count == t_hn.size()){
                    ClearSearched(searched_h);
                    return;
                }
                else break;
            }
        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            // if(!(hhe->flag_ & 4)) continue;

            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;

            double edge_length = hhe->length_s_;
            if(hhe->e_flag_ & 16)
                edge_length = hhe->length_;
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;
                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + edge_length + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }
    }
    ClearSearched(searched_h);
}


void MultiDTG::GlobalLocalSearch(h_ptr &s_hn, list<pair<double, list<Eigen::Vector3d>>> &paths, list<h_ptr> &t_hn){
    h_ptr hc, hn;
    f_ptr fn;
    list<h_ptr> searched_h;
    list<f_ptr> searched_f;
    int reach_count = 0;
    t_hn.clear();
    paths.clear();
    // paths.resize(t_hn.size());
    // for(auto &p : paths) p.first = -1.0;

    prio_D empty_set;
    open_D_.swap(empty_set);
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    searched_h.emplace_back(s_hn);
    s_hn->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, s_hn->id_, s_hn->pos_);
    cur_n = s_hn->sch_node_;
    cur_n->flag_ = 1;
    open_D_.push(cur_n);

    while(!open_D_.empty()){
        cur_n = open_D_.top();
        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;

        if(cur_n->id_ > SDM_->drone_num_ && (cur_n->flag_ & 1) && !FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode GlobalLocalSearch");
            ros::shutdown();
            return;
        }
        else if(cur_n->id_ == s_hn->id_){
            hc = s_hn;
        }

        /* explorable target? */
        for(auto &hfe : hc->hf_edges_){
            if((hfe->e_flag_ & 16) && hfe->tail_n_->cf_->f_state_ == 1){
                if(hfe->tail_n_->vp_id_ == -1){ //debug
                    ROS_ERROR("alive -1 vp %d!!", hfe->tail_n_->id_);
                    ros::shutdown();
                }
                pair<double, list<Eigen::Vector3d>> d_p;
                RetrieveHPath(s_hn, hc, d_p.second, d_p.first);
                t_hn.emplace_back(hc);
                paths.emplace_back(d_p);
                break;
            }
        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            // if(!(hhe->flag_ & 32)) continue;

            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;

            if(!(hn->h_flags_ & 4)) {
                continue;
            }
            double edge_length = hhe->length_s_;
            if(hhe->e_flag_ & 16)
                edge_length = hhe->length_;
            if(edge_length > 1e3){//debug
                ROS_ERROR("id:%d GlobalLocalSearch!", SDM_->self_id_);
                cout<<"h1:"<<int(hhe->head_)<<"  h2:"<<int(hhe->head_)<<"  flag:"<<int(hhe->e_flag_)<<" length_s_:"<<hhe->length_s_<<"  length:"<<hhe->length_<<endl;
                ros::shutdown();
                return;
            }
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;
                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + edge_length + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }
    }
    ClearSearched(searched_h);
}

void MultiDTG::FullSearch(h_ptr &s_hn, list<pair<double, list<Eigen::Vector3d>>> &paths, list<h_ptr> &t_hn){
    h_ptr hc, hn;
    f_ptr fn;
    list<h_ptr> searched_h;
    list<f_ptr> searched_f;
    int reach_count = 0;
    t_hn.clear();
    paths.clear();
    // paths.resize(t_hn.size());
    // for(auto &p : paths) p.first = -1.0;

    prio_D empty_set;
    open_D_.swap(empty_set);
    shared_ptr<DTG_sch_node> cur_n, nei_n;
    searched_h.emplace_back(s_hn);
    s_hn->sch_node_ = make_shared<DTG_sch_node>(0.0, 0.0, s_hn->id_, s_hn->pos_);
    cur_n = s_hn->sch_node_;
    cur_n->flag_ = 1;
    open_D_.push(cur_n);

    while(!open_D_.empty()){
        cur_n = open_D_.top();
        open_D_.pop();
        if(cur_n->flag_ & 2) continue;
        cur_n->flag_ |= 2;

        if(cur_n->id_ > SDM_->drone_num_ && (cur_n->flag_ & 1) && !FindHnode(cur_n->pos_, cur_n->id_, hc)){
            ROS_ERROR("error get hnode FullSearch");
            ros::shutdown();
            return;
        }
        else if(cur_n->id_ == s_hn->id_){
            hc = s_hn;
        }
        
        /* explorable target? */
        for(auto &hfe : hc->hf_edges_){
            if(hfe->tail_n_->cf_->f_state_ == 1 && (hfe->e_flag_ & 16)){
                pair<double, list<Eigen::Vector3d>> d_p;
                RetrieveHPath(s_hn, hc, d_p.second, d_p.first);
                t_hn.emplace_back(hc);
                paths.emplace_back(d_p);
                break;
            }
        }

        /* expand Hneighbours */
        for(auto &hhe : hc->hh_edges_){
            // if(!(hhe->flag_ & 8)) continue;

            if(hhe->head_n_ == hc)
                hn = hhe->tail_n_;
            else
                hn = hhe->head_n_;
                
            double edge_length = hhe->length_s_;
            if(hhe->e_flag_ & 16)
                edge_length = hhe->length_;
            if(hn->sch_node_ == NULL){
                /* new node */
                hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
                nei_n = hn->sch_node_;
                nei_n->parent_ = cur_n;
                nei_n->flag_ = 1;
                searched_h.push_back(hn);
                open_D_.push(nei_n);
            }
            else{
                /* in close list */
                if(hn->sch_node_->flag_ & 2) continue;
                /* try to change parent */
                nei_n = hn->sch_node_;
                if(cur_n->g_ + edge_length + 1e-3 < nei_n->g_){
                    nei_n->flag_ |= 2;
                    hn->sch_node_ = make_shared<DTG_sch_node>(cur_n->g_ + edge_length, 0.0, hn->id_, hn->pos_);
                    hn->sch_node_->parent_ = cur_n;
                    hn->sch_node_->flag_ = 1;
                    open_D_.push(hn->sch_node_);
                }
            }
        }
    }
    ROS_WARN("id:%d Full searched %ld, exist:%ld", SDM_->self_id_, searched_h.size(), H_list_.size());
    // if(searched_h.size() < H_list_.size()){
    //     ROS_ERROR("id:%d Full searched %ld, exist:%ld", SDM_->self_id_, searched_h.size(), H_list_.size());
    //     for(auto &hn : searched_h){
    //         cout<<"searched hn: "<<int(hn->id_)<<"   "<<(hn->sch_node_ == NULL)<<endl;
    //     }

    //     for(auto &hn : H_list_){
    //         cout<<"hn: "<<int(hn->id_)<<"   "<<(hn->sch_node_ == NULL)<<endl;
    //         for(auto &hhe : hn->hh_edges_){
    //             cout<<"hhe:"<<int(hhe->head_)<<"   "<<int(hhe->tail_)<<"  "<<int(hhe->e_flag_)<<endl;
    //         }
    //     }
    //     ros::shutdown();
    // }
    ClearSearched(searched_h);

}
