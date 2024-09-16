#include <swarm_data/swarm_data.h>

void SwarmDataManager::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){

    std::string ns = ros::this_node::getName();
    int drone_num, self_id;
    nh_private.param(ns + "/Exp/drone_num", drone_num, 1);
    nh_private.param(ns + "/Exp/UAV_id", self_id, 1);
    nh_private.param(ns + "/Exp/have_ground", have_ground_, false);
    nh_private.param(ns + "/Exp/local_comm_freq", local_comm_intv_, 1.0);
    nh_private.param(ns + "/Exp/global_comm_freq", global_comm_intv_, 1.0);
    nh_private.param(ns + "/Exp/local_dist_thresh", local_dist_thresh_, 5.0);
    nh_private.param(ns + "/Exp/use_answer", use_answer_, false);
    nh_private.param(ns + "/Exp/wait_t", wait_t_, 3.0);
    nh_private.param(ns + "/Exp/show_swarm_traj", show_swarm_traj_, false);
    nh_private.param(ns + "/Exp/statistic", statistic_, false);
    nh_private.param(ns + "/Exp/finish_thresh", finish_thresh_, 0.5);

    if(statistic_) CS_.init(nh, nh_private);
    req_flag_ = false;
    finish_num_ = 0;
    finish_list_.resize(drone_num, false);
    local_comm_intv_ = 1.0 / local_comm_intv_;
    global_comm_intv_ = 1.0 / global_comm_intv_;
    bag_id_ = 0;
    if(drone_num > 255){
        ROS_ERROR("too many!");
        ros::shutdown();
    }
    drone_num_ = drone_num;
    self_id_ = self_id;
    is_ground_ = (self_id == 0);
    ground_id_ = 0;
    flags_ = 0;
    // if(is_ground_){
    //     debug_l_.open("/home/charliedog/rosprojects/MURDER/debug_d.txt", std::ios::out);
    // }
    double t = ros::WallTime::now().toSec() - 100.0;
    pose_pub_g_t_ = job_pub_g_t_ = t;
    ba_.id = self_id_;

    trajs_.resize(drone_num);
    Poses_.resize(drone_num);
    Pose_t_.resize(drone_num);
    jobs_.resize(drone_num);
    states_.resize(drone_num);
    swarm_root_.resize(drone_num);
    for(int i = 0; i < drone_num; i++) start_t_.emplace_back(ros::WallTime::now().toSec() - 2000.0);
    for(int i = 0; i < drone_num; i++) {
        states_[i].pub_t = ros::WallTime::now().toSec() - 2000.0;
        states_[i].from_uav = i + 1;
    }

    for(auto &t : Pose_t_) t = ros::WallTime().now().toSec() - 1000.0;

    if(show_swarm_traj_)
        show_pub_ = nh.advertise<visualization_msgs::Marker>("/drone_" + to_string(self_id_) + "/swarm_vis", 100);
    if(is_ground_){
        pose_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/pose_rec", 100, &SwarmDataManager::PoseMSGCallback, this);
        DTG_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/DTG_rec", 100, &SwarmDataManager::DTGMSGCallback, this);
        traj_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/traj_rec", 100, &SwarmDataManager::TrajMSGCallback, this);
        job_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/job_rec", 100, &SwarmDataManager::JobMSGCallback, this);
        state_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/state_rec", 100, &SwarmDataManager::StateMSGCallback, this);
        if(use_answer_) ans_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/DTGAns_rec", 100, &SwarmDataManager::DTGAnsMSGCallback, this);
        map_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/map_rec", 1000, &SwarmDataManager::MapMSGCallback, this);

        mapreq_pub_ = nh.advertise<exp_comm_msgs::MapReqC>("/drone_" + to_string(self_id_) + "/mapreq_send", 100);

        // pose_timer_ = nh.createTimer(ros::Duration(local_comm_intv_), &SwarmDataManager::PoseTimerCallback, this);
        DTG_timer_ = nh.createTimer(ros::Duration(local_comm_intv_), &SwarmDataManager::DTGTimerCallback, this);
        traj_timer_ = nh.createTimer(ros::Duration(local_comm_intv_), &SwarmDataManager::TrajTimerCallback, this);
        map_timer_ = nh.createTimer(ros::Duration(5.0), &SwarmDataManager::MapTimerCallback, this);
        // job_timer_ = nh.createTimer()
        if(use_answer_) ans_timer_ = nh.createTimer(ros::Duration(wait_t_ / 2), &SwarmDataManager::DTGAnsTimerCallback, this);
    }
    else{
        pose_pub_ = nh.advertise<swarm_exp_msgs::IdPose>("/drone_" + to_string(self_id_) + "/pose_send", 100);
        DTG_pub_ = nh.advertise<swarm_exp_msgs::DtgBag>("/drone_" + to_string(self_id_) + "/DTG_send", 100);
        traj_pub_ = nh.advertise<swarm_exp_msgs::SwarmTraj>("/drone_" + to_string(self_id_) + "/traj_send", 100);
        job_pub_ = nh.advertise<exp_comm_msgs::SwarmJobC>("/drone_" + to_string(self_id_) + "/job_send", 100);
        state_pub_ = nh.advertise<exp_comm_msgs::SwarmStateC>("/drone_" + to_string(self_id_) + "/state_send", 100);
        map_pub_ = nh.advertise<exp_comm_msgs::MapC>("/drone_" + to_string(self_id_) + "/map_send", 100);
        cout<<"use_answer_:"<<use_answer_<<endl;
        if(use_answer_) ans_pub_ = nh.advertise<exp_comm_msgs::DtgBagAnswer>("/drone_" + to_string(self_id_) + "/DTGAns_send", 100);

        pose_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/pose_rec", 100, &SwarmDataManager::PoseMSGCallback, this);
        DTG_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/DTG_rec", 100, &SwarmDataManager::DTGMSGCallback, this);
        traj_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/traj_rec", 100, &SwarmDataManager::TrajMSGCallback, this);
        job_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/job_rec", 100, &SwarmDataManager::JobMSGCallback, this);
        state_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/state_rec", 100, &SwarmDataManager::StateMSGCallback, this);
        mapreq_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/mapreq_rec", 1000, &SwarmDataManager::MapReqMSGCallback, this);
        if(use_answer_) ans_sub_ = nh.subscribe("/drone_" + to_string(self_id_) + "/DTGAns_rec", 100, &SwarmDataManager::DTGAnsMSGCallback, this);



        pose_timer_ = nh.createTimer(ros::Duration(local_comm_intv_ / 2), &SwarmDataManager::PoseTimerCallback, this);
        DTG_timer_ = nh.createTimer(ros::Duration(local_comm_intv_), &SwarmDataManager::DTGTimerCallback, this);
        traj_timer_ = nh.createTimer(ros::Duration(local_comm_intv_), &SwarmDataManager::TrajTimerCallback, this);
        // job_timer_ = nh.createTimer()
        if(use_answer_) ans_timer_ = nh.createTimer(ros::Duration(wait_t_ / 2), &SwarmDataManager::DTGAnsTimerCallback, this);
    }
}

void SwarmDataManager::PoseTimerCallback(const ros::TimerEvent &e){

    if(!(flags_ & 96)) return;
    swarm_exp_msgs::IdPose IDP_l, IDP_g;
    double cur_t = ros::WallTime::now().toSec();

    /* pub local */
    IDP_l.id = self_id_;
    IDP_l.pose = Poses_[self_id_-1];
    IDP_l.pub_t = cur_t;
    IDP_g = IDP_l;
    for(uint8_t i = 0; i < drone_num_ + 1; i++){
        if(i == self_id_) continue;
        if(i == 0 || IsEucLocal(Poses_[i - 1], Poses_[self_id_ - 1])){
            IDP_l.to_uavs.emplace_back(i);
        }
    }

    if(flags_ & 32){
        pose_pub_.publish(IDP_l);
        flags_ &= 223;
    }

    /* pub global */
    if(cur_t - pose_pub_g_t_ > global_comm_intv_){
        pose_pub_g_t_ = cur_t;
        for(uint8_t i = 1; i < drone_num_ + 1; i++){
            if(i == self_id_) continue;
            // bool pub = true;
            // for(auto &l_id : self_neighbor_hns_){
            //     if(l_id == i){
            //         pub = false;
            //         break;
            //     }
            // }
            if(!IsEucLocal(Poses_[i - 1], Poses_[self_id_ - 1])){
                // cout<<"!!!!!!!!!!!!!!!!swarm pose to:"<<int(i)<<endl;
                IDP_g.to_uavs.emplace_back(i);
            }
        }
        flags_ &= 191;
        pose_pub_.publish(IDP_g);
    }
}

void SwarmDataManager::TrajTimerCallback(const ros::TimerEvent &e){
    for(list<uint8_t>::iterator uav_it = traj_msg_.first.begin(); uav_it != traj_msg_.first.end(); uav_it++){
        if(*uav_it == 0 || IsEucLocal(Poses_[self_id_ - 1], Poses_[*uav_it - 1])){
            traj_msg_.second.to_uavs.emplace_back(*uav_it);
            list<uint8_t>::iterator erase_it = uav_it;
            uav_it--;
            traj_msg_.first.erase(erase_it);
        }
    }
    if(!traj_msg_.second.to_uavs.empty()){
        traj_pub_.publish(traj_msg_.second);
        traj_msg_.second.to_uavs.clear();
    }
}

// void Swarp

void SwarmDataManager::DTGTimerCallback(const ros::TimerEvent &e){
    if(!(flags_ & 4)) return;
    /* pub local */
    swarm_exp_msgs::DtgBag DTGB_msg;
    // DTGB_msg.FFedges = swarm_pub_ffe_;
    DTGB_msg.Fnodes = swarm_pub_fn_;
    DTGB_msg.HFedges = swarm_pub_hfe_;
    DTGB_msg.HHedges = swarm_pub_hhe_;
    DTGB_msg.Hnodes = swarm_pub_hn_;
    DTGB_msg.id = bag_id_;
    DTGB_msg.from_uav = self_id_;
    bag_id_++;
    for(uint8_t i = 0; i < drone_num_ + 1; i++){
        if(i == self_id_) continue;
        DTGB_msg.to_uavs.emplace_back(i);
    }

    if(use_answer_){
        vector<bool> ba;
        for(int i = 0; i < drone_num_; i++){
            if(i+1 == self_id_) ba.emplace_back(true);
            else{
                ba.emplace_back(false);
            }
        }
        bag_answer_.push_back({ros::WallTime::now().toSec(), DTGB_msg});
    }
    swarm_pub_ffe_.clear();
    swarm_pub_fn_.clear();
    swarm_pub_hfe_.clear();
    swarm_pub_hhe_.clear();
    swarm_pub_hn_.clear();
    if(!is_ground_)
        DTG_pub_.publish(DTGB_msg);
    flags_ &= 251;
}

// void DTGTimerCallback(const ros::TimerEvent &e);
// void JobTimerCallback(const ros::TimerEvent &e);
void SwarmDataManager::MapTimerCallback(const ros::TimerEvent &e){
    req_flag_ = true;
}

void SwarmDataManager::DTGAnsTimerCallback(const ros::TimerEvent &e){
    double cur_t = ros::WallTime::now().toSec();
    for(list<pair<double, swarm_exp_msgs::DtgBag>>::iterator ba = bag_answer_.begin(); ba != bag_answer_.end(); ba++){
        if(ba->second.to_uavs.empty()){
            list<pair<double, swarm_exp_msgs::DtgBag>>::iterator erase_it = ba;
            ba--;
            bag_answer_.erase(erase_it);
        }
        if(cur_t - ba->first < wait_t_){
            // ROS_WARN("pub again!!!!!!!!!!!!!!!!%d", ba->second.id);
            ba->second.from_uav = self_id_;
            DTG_pub_.publish(ba->second);
            ba->first = cur_t;
        }
    }
}

void SwarmDataManager::PoseMSGCallback(const exp_comm_msgs::IdPoseCConstPtr &msg){
    if(msg->id == self_id_) return;
    if(statistic_){
        double v = sizeof(*msg);
        CS_.AddVolume(v, 2);
    }
    if(msg->id > drone_num_ + 1 || msg->id <= 0){
        ROS_ERROR("strange id%d, max%d, self%d", msg->id, drone_num_ + 1, self_id_);
        return;
    }
    Poses_[msg->id - 1] = msg->pose;
    Pose_t_[msg->id - 1] = msg->pub_t;
}

void SwarmDataManager::TrajMSGCallback(const exp_comm_msgs::SwarmTrajCConstPtr &msg){
    if(msg->id == self_id_) return;
    if(statistic_){
        double v = sizeof(*msg) + sizeof(float) * msg->t_p.size() + sizeof(geometry_msgs::Point) * msg->coef_p.size();
        CS_.AddVolume(v, 3);
    }
    if(msg->id > drone_num_ + 1 || msg->id <= 0){
        ROS_ERROR("strange id%d, max%d, self%d", msg->id, drone_num_ + 1, self_id_);
        return;
    }
    start_t_[msg->id - 1] = msg->start_t;
    int col = 0;
    int t_idx = 0;
    Eigen::MatrixXd cM(3, 6);
    trajs_[msg->id - 1].clear();

    for(int i = 0; i < msg->coef_p.size(); i++, col++){
        cM(0, col) = msg->coef_p[i].x;
        cM(1, col) = msg->coef_p[i].y;
        cM(2, col) = msg->coef_p[i].z;
        if(col == 5){
            trajs_[msg->id - 1].emplace_back(double(msg->t_p[t_idx]), cM);
            col = -1;
            t_idx++;
        }
    }
    if(show_swarm_traj_) ShowTraj(msg->id);
}

void SwarmDataManager::DTGMSGCallback(const exp_comm_msgs::DtgBagCConstPtr &msg){
    if(msg->from_uav == self_id_) return;
    if(statistic_){
        double v = sizeof(*msg);
        v += sizeof(swarm_exp_msgs::DtgHFEdge) * msg->HFedges.size();
        v += sizeof(swarm_exp_msgs::DtgHHEdge) * msg->HHedges.size();
        v += sizeof(swarm_exp_msgs::DtgHNode) * msg->Hnodes.size();
        v += sizeof(swarm_exp_msgs::DtgFNode) * msg->Fnodes.size();
        for(auto hfe : msg->HFedges) v += sizeof(uint32_t) * hfe.points_idx.size();
        for(auto hhe : msg->HHedges) v += sizeof(uint32_t) * hhe.points_idx.size();
        for(auto fn : msg->Fnodes) v += sizeof(uint8_t) * fn.vp_flags.size();
        CS_.AddVolume(v, 1);
    }

    for(auto &hhe : msg->HHedges){
        swarm_sub_hhe_.emplace_back(hhe);
    }
    
    for(auto &hn : msg->Hnodes){
        swarm_sub_hn_.emplace_back(hn);
        if(is_ground_){
            ROS_WARN("from %d, hn: %d", msg->from_uav, hn.h_id);
        }
    }

    // for(auto &ffe : msg->FFedges){
    //     bool new_e = true;
    //     for(auto &rffe : swarm_sub_ffe_){
    //         if((rffe.head_f_id == ffe.head_f_id && rffe.tail_f_id == ffe.tail_f_id) || 
    //             (rffe.tail_f_id == ffe.head_f_id && rffe.head_f_id == ffe.tail_f_id)){
    //             rffe = ffe;
    //             new_e = false;
    //             break;
    //         }
    //     }
    //     if(new_e) swarm_sub_ffe_.emplace_back(ffe);
    // }

    for(auto &hfe : msg->HFedges){
        swarm_sub_hfe_.emplace_back(hfe);
    }
    for(auto &fn : msg->Fnodes){
        swarm_sub_fn_.emplace_back(fn);
    }

    if(use_answer_) {
        ba_.bag_id = msg->id;     
        ba_.to_uav = msg->from_uav;
        ans_pub_.publish(ba_);
    }
}

void SwarmDataManager::DTGAnsMSGCallback(const exp_comm_msgs::DtgBagAnswerConstPtr &msg){
    if(msg->to_uav != self_id_) return;
    // for(auto &bag_id : msg->bag_id){
        // ROS_WARN("rec ans!!!!!!!!!!!!!!!!%d", msg->bag_id);
        for(auto &ba : bag_answer_){
            if(msg->bag_id == ba.second.id){
                for(vector<uint8_t>::iterator it = ba.second.to_uavs.begin(); it != ba.second.to_uavs.end(); it++){
                    if(*it == msg->id){
                        // ROS_WARN("erase %d", msg->id);
                        ba.second.to_uavs.erase(it);
                        break;
                    }
                }
                break;
            }
        }
    // }
}

void SwarmDataManager::JobMSGCallback(const exp_comm_msgs::SwarmJobCConstPtr &msg){
    if(msg->from_uav == self_id_) return;
    if(statistic_){
        double v = sizeof(*msg);
        CS_.AddVolume(v, 4);
    }
    if((msg->JobState & 2) && !finish_list_[msg->from_uav - 1]){ 
        finish_num_++;
        finish_list_[msg->from_uav - 1] = true;
    }
    if(msg->JobState & 2) return;

    if(msg->from_uav > drone_num_ + 1 || msg->from_uav <= 0){
        ROS_ERROR("strange id%d, max%d, self%d", msg->from_uav, drone_num_ + 1, self_id_);
        return;
    }
    if(msg->JobState & 4){
        jobs_[msg->from_uav - 1].clear();
        trajs_[msg->from_uav - 1].clear();
        start_t_[msg->from_uav - 1] = ros::WallTime::now().toSec() - 100.0;
    }
    jobs_[msg->from_uav - 1].emplace_back(*msg);
}

void SwarmDataManager::StateMSGCallback(const exp_comm_msgs::SwarmStateCConstPtr &msg){
    if(msg->from_uav == self_id_) return;
    if(statistic_){
        double v = sizeof(*msg) + sizeof(uint16_t) * msg->connect_fn.size() + sizeof(uint32_t) * msg->connect_hn.size()
        + sizeof(float) * msg->dist_to_connect_fn.size() + sizeof(float) * msg->dist_to_connect_hn.size() + sizeof(float) * msg->dist_to_local_fn.size()
        + sizeof(uint32_t) * msg->hn_pos_idx.size() + sizeof(uint16_t) * msg->local_fn.size();
        CS_.AddVolume(v, 5);
    }
    if(msg->from_uav > drone_num_ + 1 || msg->from_uav <= 0){
        ROS_ERROR("strange id%d, max%d, self%d", msg->from_uav, drone_num_ + 1, self_id_);
        return;
    }
    if(states_[msg->from_uav - 1].pub_t < msg->pub_t) states_[msg->from_uav - 1] = *msg;
}

void SwarmDataManager::MapMSGCallback(const exp_comm_msgs::MapCConstPtr &msg){
    if(statistic_){
        double v = sizeof(*msg) + msg->flags.size() * sizeof(uint8_t);
        CS_.AddVolume(v, 6);
    }
    // debug_l_<<"f:"<<int(msg->f_id)<<"  b:"<<int(msg->block_id)<<"  state:"<<int(msg->block_state)<<" finish:"<<finish_num_<<endl;
    swarm_sub_map_.emplace_back(*msg);
}

void SwarmDataManager::MapReqMSGCallback(const exp_comm_msgs::MapReqCConstPtr &msg){
    if(is_ground_) return;
    if(statistic_){
        double v = sizeof(*msg) + msg->block_id.size() * sizeof(uint8_t) + msg->f_id.size() * sizeof(uint16_t);
        CS_.AddVolume(v, 7);
    }
    if(msg->flag) finish_list_[self_id_ - 1] = true;
    mreq_ = *msg;
    req_flag_ = true;
}

void SwarmDataManager::ShowTraj(const uint8_t &id){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.scale.x = 0.05;
    mk.scale.y = 0.05;
    mk.scale.z = 0.05;
    mk.color.a = 0.5;
    mk.color.g = 0.8;
    mk.lifetime = ros::Duration(trajs_[id - 1].getTotalDuration());
    for(double delta = 0; delta < trajs_[id - 1].getTotalDuration(); delta += 0.05){
        Eigen::Vector3d p;
        geometry_msgs::Point pt;
        p = trajs_[id - 1].getPos(delta);
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
    }
    if(!mk.points.empty()) show_pub_.publish(mk);
}