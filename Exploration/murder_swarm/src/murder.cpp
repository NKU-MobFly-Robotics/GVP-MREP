#include<murder_swarm/murder.h>


void Murder::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){
    std::string ns = ros::this_node::getName();
    nh_private_.param(ns + "/Exp/traj_length", traj_length_, 10.0);
    nh_private.param(ns + "/Exp/local_path_search", local_max_search_iter_, 1000);
    nh_private.param(ns + "/Exp/global_path_search", global_max_search_iter_, 3000);
    nh_private.param(ns + "/Exp/strong_check_interval", strong_check_interval_, 3.0);
    nh_private.param(ns + "/Exp/replan_duration", replan_duration_, 1.5);
    nh_private.param(ns + "/Exp/check_duration", check_duration_, 3.5);
    nh_private.param(ns + "/Exp/exc_duration", exc_duration_, 0.5);
    nh_private.param(ns + "/Exp/takeoff_x", init_pose_(0), 0.0);
    nh_private.param(ns + "/Exp/takeoff_y", init_pose_(1), 0.0);
    nh_private.param(ns + "/Exp/takeoff_z", init_pose_(2), 1.0);
    nh_private.param(ns + "/Exp/takeoff_yaw", init_pose_(3), 0.0);
    nh_private.param(ns + "/Exp/reach_out_t", reach_out_t_, 0.1);
    nh_private.param(ns + "/Exp/acc_off", acc_off_, -0.3);
    nh_private.param(ns + "/Exp/swarm_check", swarm_check_, false);
    nh_private.param(ns + "/Exp/colli_range", colli_range_, 0.8);
    
    nh_ = nh;
    nh_private_ = nh_private;
    SDM_.init(nh_, nh_private_);

    BM_.SetSwarmDataManager(&SDM_);
    BM_.init(nh_, nh_private_);
    CM_.init(nh_, nh_private_);
    LRM_.SetColorManager(&CM_);


    LRM_.SetMap(&BM_);
    LRM_.init(nh, nh_private);
    TrajOpt_.Init(nh_, nh_private_);

    FG_.SetMap(BM_);
    FG_.SetLowresMap(LRM_);
    FG_.SetColorManager(CM_);
    FG_.SetSwarmDataManager(&SDM_);
    FG_.init(nh_, nh_private_);

    MDTG_.SetFrontierMap(&FG_);
    MDTG_.SetColorManager(&CM_);
    MDTG_.SetBlockMap(&BM_);
    MDTG_.SetLowresMap(&LRM_);
    MDTG_.SetSwarmDataManager(&SDM_);
    MDTG_.local_fn_ = &local_fn_;
    MDTG_.local_hn_ = &local_hn_;
    MDTG_.GVP_hn_ = &GVP_hn_;
    MDTG_.init(nh_, nh_private_);
    
    GVP_.SetLowresMap(&LRM_);
    GVP_.SetDTGMap(&MDTG_);
    GVP_.SetFrontierMap(&FG_);
    GVP_.SetColorManager(&CM_);
    GVP_.SetSwarmDataManager(&SDM_);
    GVP_.local_fn_ = &local_fn_;
    GVP_.local_hn_ = &local_hn_;
    GVP_.GVP_hn_ = &GVP_hn_;
    GVP_.init(nh_, nh_private_);
    YawP_.init(nh, nh_private);

    GVP_.SetYawP(&YawP_);
    GVP_.SetVA(TrajOpt_.upboundVec_[0], TrajOpt_.upboundVec_[1]);

    // CreateVisModel();
    last_safe_ = init_pose_.head(3);
    home_p_ = init_pose_.head(3);

    last_map_update_t_ = ros::WallTime::now().toSec();
    traj_end_t_ = last_map_update_t_ - 0.1;
    have_odom_ = false;
    target_f_id_ = -1;
    target_v_id_ = -1;
    sensor_flag_ = false;
    if(FG_.sensor_type_ == SensorType::CAMERA){
        depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/depth", 10));
        vi_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/vi_odom", 10));
        sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *depth_sub_, *vi_odom_sub_));
        sync_image_odom_->registerCallback(boost::bind(&Murder::ImgOdomCallback, this,  _1, _2));
    }
    else{
        pcl_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/pointcloud", 10));
        vi_odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_, "/vi_odom", 10));
        sync_pointcloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyPCLOdom>(
            SyncPolicyPCLOdom(100), *pcl_sub_, *vi_odom_sub_));
        sync_pointcloud_odom_->registerCallback(boost::bind(&Murder::PCLOdomCallback, this,  _1, _2));
    }
    odom_sub_ = nh_.subscribe("/odom", 1, &Murder::BodyOdomCallback, this);
    show_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/Murder/Show", 5);
    // posevis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "Murder/PoseVis", 1);
    traj_pub_ = nh_.advertise<swarm_exp_msgs::LocalTraj>("/Murder/Traj", 1);
}

void Murder::BroadCastFinish(){
    static bool debug_vp = false;
    if(!debug_vp){
        debug_vp = true;
        for(int i = 0; i < 5; i++){
            ROS_ERROR("id: %d, finish!!!", SDM_.self_id_);
        }
    }
    SDM_.SetJob(2, 0, 0, 0, 0);


}

int Murder::AllowPlan(const double &T){
    /* not satisfy plan interval */
    if(T - plan_t_ < 0){
        return 1;
    }

    /* not in free space */
    if(!LRM_.IsFeasible(p_)){
        return 2;
    }
    last_safe_ = p_;

    /* sensor not update */
    if(!sensor_flag_){
        return 3;
    }

    /* viewpoints not sampled */
    if(!FG_.sample_flag_){
        return 4;
    }


    return 0;
}

void Murder::SetPlanInterval(const double &intv){
    plan_t_ = ros::WallTime::now().toSec() + intv;
    sensor_flag_ = false;
    FG_.sample_flag_ = false;
}

bool Murder::GoHome(){
    Eigen::Vector3d ps, vs, as, pe, ve, ae;
    double ys, yds, ydds, ye, yde, ydde;
    double hand_t = ros::WallTime::now().toSec() + reach_out_t_; 
    double cur_t = ros::WallTime::now().toSec();
    if(hand_t > traj_end_t_){
        hand_t = cur_t;
        ps = p_;
        vs = v_;
        as.setZero();
        ve.setZero();
        ae.setZero();
        ys = yaw_;
        yds = yaw_v_;
        ydds = 0;
        yde = 0; 
        ydde = 0;
    }
    else{
        ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        while(!LRM_.IsFeasible(ps) && hand_t > cur_t){
            hand_t -= reach_out_t_ / 10;
            ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        }
        if(!LRM_.IsFeasible(ps)){
            hand_t = cur_t;
            ps = p_;
        }
        vs = TrajOpt_.traj.getVel(hand_t - traj_start_t_);
        as = TrajOpt_.traj.getAcc(hand_t - traj_start_t_);
        ve.setZero();
        ae.setZero();
        YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        yde = 0; 
        ydde = 0;
    }
    pe = home_p_;
    ye = 0;
    if(TrajPlanB(ps, vs, as, pe, ve, 
                ae, ys, yds, ydds, target_(3), yde, ydde, home_p_ + Eigen::Vector3d(1.0, 0, 0), hand_t)){
        traj_start_t_ = hand_t;
        traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
        replan_t_ = min(replan_duration_, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
        target_f_id_ = -2;
        target_v_id_ = -2;
        PublishTraj(false);
        return true;
    }
    return false;
}

bool Murder::LocalPlan(){
    ROS_WARN("id:%d local plan!", SDM_.self_id_);
    Eigen::Vector3d ps, vs, as, pe, ve, ae;
    double ys, yds, ydds, ye, yde, ydde;
    double hand_t = ros::WallTime::now().toSec() + reach_out_t_; 
    double cur_t = ros::WallTime::now().toSec();

    /* get start status */
    if(hand_t > traj_end_t_){
        hand_t = cur_t;
        ps = p_;
        vs = v_;
        as.setZero();
        ve.setZero();
        ae.setZero();
        if(YawP_.T_.size() != 0) YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        else ys = yaw_;
        yds = yaw_v_;
        ydds = 0;
        yde = 0; 
        ydde = 0;
    }
    else{
        ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        while(!LRM_.IsFeasible(ps) && hand_t > cur_t){
            hand_t -= reach_out_t_ / 10;
            ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        }
        if(!LRM_.IsFeasible(ps)){
            hand_t = cur_t;
            ps = p_;
        }
        vs = TrajOpt_.traj.getVel(hand_t - traj_start_t_);
        as = TrajOpt_.traj.getAcc(hand_t - traj_start_t_);
        ve.setZero();
        ae.setZero();
        YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        yde = 0; 
        ydde = 0;
    }
    if(SDM_.statistic_) SDM_.CS_.StartTimer(1);

    /* sample vps befor planning */
    if(!FG_.sample_flag_){
        FG_.SampleVps();
    }

    /* go to the most promising target */
    Eigen::Vector4d c_state, t_state;
    list<Eigen::Vector3d> path, safe_path, unknown_path; 
    vector<Eigen::Vector3d> s_p;
    pair<int, int> f_v;
    int h_id;
    double path_cost = 9999.0;
    int exp_state;
    c_state.block(0, 0, 3, 1) = p_;
    c_state(3) = yaw_;
    GVP_.GetLocalFNodes(c_state, v_, path, path_cost, t_state, f_v, h_id, exp_state);
    // cout<<"id:"<<int(SDM_.self_id_)<<"exp_state:"<<exp_state<<"---"<<t_state.transpose()<<endl;
    switch (exp_state)
    {
    case 0:{ // free path
        safe_path = path;
        target_ = t_state;
        target_vp_pose_ = t_state;
        dangerous_path_ = false;
        break;
    }
    case 1:{ // dangerous path
        ROS_WARN("id:%d dangerous local plan!", SDM_.self_id_);
        dangerous_path_ = true;
        target_vp_pose_ = t_state;
        LRM_.GetSafePath(path, safe_path, unknown_path, 9999.0, 3.5);
        if(unknown_path.size() > 0){
            // safe_path = path;
            target_(3) = atan2(unknown_path.front()(1) - safe_path.back()(1), unknown_path.front()(0) - safe_path.back()(0));
            target_.block(0, 0, 3, 1) = safe_path.back();
            // ROS_ERROR("!!!");
            // cout<<"target_:"<<target_.transpose()<<"    !!!"<<" safe_num:"<<safe_path.size()<<"  unknown_num:"<<unknown_path.size()<<endl;//debug
            // if(!LRM_.IsFeasible(safe_path.back())){
            //     ROS_ERROR("id:%d dangerous local plan, fail!", SDM_.self_id_);
            //     for(auto &pt : path) cout<<"path:"<<pt.transpose()<<" feas:"<<int(LRM_.IsFeasible(pt))<<endl;
            //     for(auto &pt : safe_path) cout<<"safe_path:"<<pt.transpose()<<" feas:"<<int(LRM_.IsFeasible(pt))<<endl;
            //     for(auto &pt : unknown_path) cout<<"unknown_path:"<<pt.transpose()<<" feas:"<<int(LRM_.IsFeasible(pt))<<endl;
            //     ros::shutdown();
            //     return false;
            // }
        }
        else{
            target_ = t_state;
            // ROS_ERROR("???");
            // cout<<"target_:"<<target_.transpose()<<"    ???"<<endl;//debug
        }
        break;
    }
    default: // no path
        GVP_.SetJob(0, -1, path_cost, -1, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(1);
        return false;                        
        break;
    }

    for(auto &pt : safe_path) s_p.emplace_back(pt);
    // list<Eigen::Vector3d> debug_l;
    // for(auto &it : d_f_v){
    //     Eigen::Vector3d p;
    //     FG_.GetVpPos(it.second.first, it.second.second, p);
    //     // debug_l.emplace_back(p);
    // }
    // for(auto &it : d_f_v){
    //     Eigen::Vector4d vp_pose;
    //     if(!FG_.GetVp(it.second.first, it.second.second, vp_pose)){//vp alive
    //         ROS_WARN("LocalPlan vp dead");
    //         ros::shutdown();
    //         return false;                        
    //     }
    //     double yaw_t = YawP_.GetMinT(yaw_, vp_pose(3));
    //     it.first = max(it.first / TrajOpt_.upboundVec_[0], yaw_t);
    // }
    // Debug(debug_l);

    target_f_id_ = -1;
    target_v_id_ = -1;

    pe = target_.head(3);
    // cout<<"tar:"<<target_.transpose()<<"  pe:"<<pe.transpose()<<endl;
    ye = target_(3);


    // if(dangerous && TrajPlanA(s_p ,ps, vs, as, pe, ve, 
    //         ae, ys, yds, ydds, ye, yde, ydde, FG_.f_grid_[f_v.first].center_)){
    //     target_f_id_ = f_v.second;
    //     target_v_id_ = f_v.first;
    //     traj_start_t_ = hand_t;
    //     traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
    //     replan_t_ = min(replan_duration_, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
    //     cout<<"plan success1!"<<endl;
    //     PublishTraj(false);
    //     return true;
    // }
    // else
    // ve = GetEndV(f_v.first, f_v.second, ps, vs, dangerous_path_);

    if(TrajPlanB(ps, vs, as, pe, ve, 
            ae, ys, yds, ydds, ye, yde, ydde, FG_.f_grid_[f_v.first].center_, hand_t)){
        if(!SwarmFeasiCheck()){
            bool success = false;
            for(int dim = 0; dim < 3; dim++){
                for(int dir = -1; dir <=1; dir += 2){
                    pe = ps;
                    pe(dim) = ps(dim) + dir * 0.5;
                    if(BM_.PosBBXFree(pe, Eigen::Vector3d::Ones() * colli_range_) && LRM_.IsFeasible(pe)){
                        ve.setZero();
                        ae.setZero();
                        ye = ys;
                        yde = 0;
                        ydde = 0;
                        if(TrajPlanB(ps, vs, as, pe, ve, 
                            ae, ys, yds, ydds, ye, yde, ydde, FG_.f_grid_[f_v.first].center_, hand_t)){
                            if(SwarmFeasiCheck()){
                                success = true;
                                break;
                            }
                        }
                    }
                }
            }
            if(!success){
                return false;
            }
        }

        target_f_id_ = f_v.first;
        target_v_id_ = f_v.second;
        traj_start_t_ = hand_t;
        traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
        replan_t_ = min(replan_duration_, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
        // cout<<"plan success1!"<<" s:"<<ps.transpose()<<"  e:"<<pe.transpose()<<" tar:"<<target_vp_pose_.transpose()<<" rt:"<<replan_t_ - traj_start_t_<<
        // " dur:"<<TrajOpt_.traj.getTotalDuration()<<endl;
        PublishTraj(false);        
        GVP_.SetJob(4, f_v.first, path_cost, h_id, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(1);
        return true;
    }
    else{
        cout<<"s:"<<ps.transpose()<<endl;
        cout<<"e:"<<pe.transpose()<<endl;
        ROS_WARN("id:%d LocalPlan fail", SDM_.self_id_);
        MDTG_.RemoveVp(FG_.f_grid_[f_v.first].center_, f_v.first, f_v.second, true);
        GVP_.SetJob(0, -1, path_cost, -1, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(1);
        return false;
    }
}

bool Murder::GlobalPlan(){
    ROS_WARN("id:%d global plan!", SDM_.self_id_);
    // int f_id, v_id;
    double length;
    target_f_id_ = -1;
    target_v_id_ = -1;

    Eigen::Vector3d ps, vs, as, pe, ve, ae;
    double ys, yds, ydds, ye, yde, ydde;
    double hand_t = ros::WallTime::now().toSec() + reach_out_t_; 
    double cur_t = ros::WallTime::now().toSec();
    if(hand_t > traj_end_t_){
        hand_t = ros::WallTime::now().toSec();
        ps = p_;
        vs = v_;
        as.setZero();
        ve.setZero();
        ae.setZero();
        if(YawP_.T_.size() != 0) YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        else ys = yaw_;        // ys = yaw_;
        yds = yaw_v_;
        ydds = 0;
        yde = 0; 
        ydde = 0;
    }
    else{
        ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        while(!LRM_.IsFeasible(ps) && hand_t > cur_t){
            hand_t -= reach_out_t_ / 10;
            ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        }
        if(!LRM_.IsFeasible(ps)){
            hand_t = cur_t;
            ps = p_;
        }
        vs = TrajOpt_.traj.getVel(hand_t - traj_start_t_);
        as = TrajOpt_.traj.getAcc(hand_t - traj_start_t_);
        ve.setZero();
        ae.setZero();
        YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        yde = 0; 
        ydde = 0;
    }
    if(SDM_.statistic_) SDM_.CS_.StartTimer(2);

    /* go to the most promising target */
    Eigen::Vector4d c_state, t_state;
    list<Eigen::Vector3d> path, safe_path, unknown_path, path_temp; 
    vector<Eigen::Vector3d> s_p;
    pair<int, int> f_v;
    int h_id;
    double path_cost = 9999.0;
    int exp_state;
    c_state.block(0, 0, 3, 1) = p_;
    c_state(3) = yaw_;
    GVP_.GetGlobalFNodes(c_state, path, path_cost, t_state, f_v, h_id, exp_state);

    cout<<"id:"<<int(SDM_.self_id_)<<"exp_state:"<<t_state.transpose()<<"  "<<exp_state<<endl;//debug
    switch (exp_state)
    {
    case -2:{ // follow free path
        ROS_WARN("id:%d free global follow plan!", SDM_.self_id_);
        target_vp_pose_ = t_state;
        dangerous_path_ = false;
        GetFollowPath(path, safe_path);
        if(safe_path.size() > 0){
            target_.block(0, 0, 3, 1) = safe_path.back();
            target_(3) = yaw_;
        }
        else{
            target_.block(0, 0, 3, 1) = p_;
            target_(3) = yaw_;
        }
        break;
    }
    case -1:{ // follow dangerous path
        ROS_WARN("id:%d dangerous global follow plan!", SDM_.self_id_);
        dangerous_path_ = true;
        target_vp_pose_ = t_state;
        GetFollowPath(path, path_temp);
        LRM_.GetSafePath(path_temp, safe_path, unknown_path, 9999.0, 3.5);
        if(unknown_path.size() > 0){
            target_(3) = atan2(unknown_path.front()(1) - safe_path.back()(1), unknown_path.front()(0) - safe_path.back()(0));
            target_.block(0, 0, 3, 1) = safe_path.back();
        }
        else{
            target_ = t_state;
        }
        break;
    }
    case 0:{ // free path
        dangerous_path_ = false;
        target_vp_pose_ = t_state;
        target_ = t_state;
        break;
    }
    case 1:{ // dangerous path
        ROS_WARN("id:%d dangerous global plan!", SDM_.self_id_);
        dangerous_path_ = true;
        target_vp_pose_ = t_state;
        LRM_.GetSafePath(path, safe_path, unknown_path, 9999.0, 3.5);
        if(unknown_path.size() > 0){
            target_(3) = atan2(unknown_path.front()(1) - safe_path.back()(1), unknown_path.front()(0) - safe_path.back()(0));
            target_.block(0, 0, 3, 1) = safe_path.back();
        }
        else{
            target_ = t_state;
        }
        break;
    }
    default: // no path
        GVP_.SetJob(5, -1, path_cost, -1, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(2);
        return false;                        
        break;
    }

    if(!FG_.StrongCheckViewpoint(f_v.first, f_v.second, true)){
        ROS_WARN("id:%d GlobalPlan vp StrongCheckViewpoint fail f:%d v:%d", SDM_.self_id_, f_v.first, f_v.second);
        if(f_v.first >= 0 && f_v.first < FG_.f_grid_.size() && 0 <= f_v.second && f_v.second < FG_.f_grid_[f_v.first].local_vps_.size())
            MDTG_.RemoveVp(FG_.f_grid_[f_v.first].center_, f_v.first, f_v.second, true);
        GVP_.SetJob(0, -1, path_cost, -1, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(2);
        return false;           
    }

    pe = target_.head(3);
    ye = target_(3);
    // ve = GetEndV(f_v.first, f_v.second, ps, vs, dangerous_path_);

    // if(!dangerous && TrajPlanA(s_p ,ps, vs, as, pe, ve, 
    //         ae, ys, yds, ydds, ye, yde, ydde, FG_.f_grid_[f_v.first].center_)){
    //     target_f_id_ = f_v.second;
    //     target_v_id_ = f_v.first;
    //     traj_start_t_ = hand_t;
    //     traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
    //     replan_t_ = min(replan_duration_, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
    //     cout<<"plan success1!"<<endl;
    //     PublishTraj(false);
    //     return true;
    // }
    // else 
    if(TrajPlanB(ps, vs, as, pe, ve, 
            ae, ys, yds, ydds, ye, yde, ydde, FG_.f_grid_[f_v.first].center_, hand_t)){
        if(!SwarmFeasiCheck()){
            bool success = false;
            for(int dim = 0; dim < 3; dim++){
                for(int dir = -1; dir <=1; dir += 2){
                    pe = ps;
                    pe(dim) = ps(dim) + dir * 0.5;
                    if(BM_.PosBBXFree(pe, Eigen::Vector3d::Ones() * colli_range_) && LRM_.IsFeasible(pe)){
                        ve.setZero();
                        ae.setZero();
                        ye = ys;
                        yde = 0;
                        ydde = 0;
                        if(TrajPlanB(ps, vs, as, pe, ve, 
                            ae, ys, yds, ydds, ye, yde, ydde, FG_.f_grid_[f_v.first].center_, hand_t)){
                            if(SwarmFeasiCheck()){
                                success = true;
                                break;
                            }
                        }
                    }
                }
            }
            if(!success){
                return false;
            }
        }

        target_f_id_ = f_v.first;
        target_v_id_ = f_v.second;
        traj_start_t_ = hand_t;
        traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
        replan_t_ = min(replan_duration_ * 3.0, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
        // cout<<"plan success2!"<<" s:"<<ps.transpose()<<"  e:"<<pe.transpose()<<" tar:"<<target_vp_pose_.transpose()<<" rt:"<<replan_t_ - traj_start_t_<<
        // " dur:"<<TrajOpt_.traj.getTotalDuration()<<endl;
        PublishTraj(false);
        GVP_.SetJob(5, f_v.first, path_cost, h_id, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(2);
        return true;
    }
    else{
        cout<<"s:"<<ps.transpose()<<endl;
        cout<<"e:"<<pe.transpose()<<endl;
        ROS_WARN("id:%d GlobalPlan fail", SDM_.self_id_);
        MDTG_.RemoveVp(FG_.f_grid_[f_v.first].center_, f_v.first, f_v.second, true);
        GVP_.SetJob(0, -1, path_cost, -1, path_cost);
        if(SDM_.statistic_) SDM_.CS_.EndTimer(2);
        return false;
    }

    // if(MDTG_.GetClosestGlobalTarget(path, H_path, f_id, v_id, length)){
    //     ShowPath(path, -3);
    //     list<Eigen::Vector3d> h_path;
    //     for(auto &h : H_path){
    //         h_path.emplace_back(h->pos_);
    //     }
    //     ShowPath(h_path, -4);
    //     // ros::shutdown();//debug

    //     if(!FG_.StrongCheckViewpoint(f_id, v_id, false)){
    //         ROS_WARN("GlobalPlan vp StrongCheckViewpoint fail f:%d v:%d", f_id, v_id);
    //         if(f_id >= 0 && f_id < FG_.f_grid_.size() && 0 <= v_id && v_id < FG_.f_grid_[f_id].local_vps_.size())
    //             MDTG_.RemoveVp(FG_.f_grid_[f_id].center_, f_id, v_id, true);
    //         return false;           
    //     }
    //     if(!FG_.GetVp(f_id, v_id, target_)){
    //         ROS_ERROR("GlobalPlan fail getvp");
    //         cout<<f_id<<"  "<<v_id<<endl;
    //         cout<<int(FG_.f_grid_[f_id].f_state_)<<endl;
    //         ros::shutdown();
    //         return false;
    //     }

    //     pe = target_.head(3);
    //     ye = target_(3);
    //     // cout<<"ps:"<<ps.transpose()<<" vs:"<<vs.transpose()<<" as:"<<as.transpose()<<" pe:"<<pe.transpose()<<" ys:"<<ys<<" ye:"<<ye<<endl;
    //     if(TrajPlanB(ps, vs, as, pe, ve, 
    //             ae, ys, yds, ydds, target_(3), yde, ydde, FG_.f_grid_[f_id].center_)){
    //         target_f_id_ = f_id;
    //         target_v_id_ = v_id;
    //         traj_start_t_ = hand_t;
    //         traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
    //         replan_t_ = min(replan_duration_, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
    //         PublishTraj(false);
    //         return true;
    //     }
    //     else {
    //             ROS_WARN("GlobalPlan traj fail");
    //             return false;
    //         // }
    //     }
    // }
    // else{
    //     ROS_WARN("GlobalPlan no vp");
    //     return false;
    // }
}

bool Murder::TrajCheck(){
    double cur_t = max(ros::WallTime::now().toSec(), traj_start_t_);

    if(cur_t > replan_t_ - 1e-3) return false;

    double end_t = min(check_duration_, traj_end_t_ - 1e-3 - cur_t);
    cur_t = cur_t - traj_start_t_;
    Eigen::Vector3d last_p = TrajOpt_.traj.getPos(cur_t);
    Eigen::Vector3d p, r_size;
    r_size = LRM_.GetRobotSize() * 0.8;
    if(swarm_check_){
        if(!SwarmFeasiCheck()) return false;
    }
    for(double t = cur_t; t < end_t; t += 0.05){
        p = TrajOpt_.traj.getPos(t);
        for(int dim = 0; dim < 3; dim ++){
            if(abs(p(dim) - last_p(dim)) > BM_.resolution_){
                if(!BM_.PosBBXFree(p, r_size)) return false;
                break;
            }
        }
    }

    return true;
}

bool Murder::SwarmFeasiCheck(){
    double cur_t = ros::WallTime::now().toSec();
    double tc;
    vector<Eigen::Vector3d> swarm_p;
    Eigen::Vector3d cur_p;
    SDM_.GetLocalSwarmPos(swarm_p);
    for(int i = 0; i < SDM_.drone_num_; i++){
        if(SDM_.IsEucLocalUAV(i+1) && i + 1 != SDM_.self_id_){
            // if(i + 1 == SDM_.self_id_) ROS_ERROR("Man!");
            bool static_check = false;
            double total_t = SDM_.trajs_[i].getTotalDuration();
            if(SDM_.trajs_[i].getPieceNum() == 0 || cur_t - SDM_.start_t_[i] > total_t){
                static_check = true;
            }

            for(double t = cur_t; t < check_duration_ + cur_t; t+=0.05){
                if(cur_t > traj_end_t_ - 1e-3) break;
                
                if(cur_t < traj_start_t_) tc = 1e-3;
                else tc = t - traj_start_t_;
                cur_p = TrajOpt_.traj.getPos(tc);

                if(!static_check){
                    tc = t - SDM_.start_t_[i];
                    if(tc < 0) tc = 1e-3;
                    else if(tc > total_t) tc = total_t - 1e-3;
                    swarm_p[i] = SDM_.trajs_[i].getPos(tc);
                }
                if((cur_p - swarm_p[i]).norm() < colli_range_) {
                    cout<<"static_check:"<<static_check<<" tc:"<<tc<<endl;
                    ROS_WARN("id:%d swarm collision %d", SDM_.self_id_, i + 1);
                    return false;
                }
            }
        }
    }
    return true;
}

bool Murder::Recover(){
    double dist = 99999.0;

    for(int x = -1; x <= 1; x += 1)
        for(int y = -1; y <= 1; y += 1)
            for(int z = -1; z <= 1; z += 1){
        if(x == 0 && y == 0 && z == 0) continue;
        Eigen::Vector3d p_chk = p_ + Eigen::Vector3d(x, y, z).cwiseProduct(LRM_.GetLowResolution());
        if(!LRM_.IsFeasible(p_chk)) continue;
        list<Eigen::Vector3d> path;
        bool free = true;
        double cur_d = (p_ - p_chk).norm();
        BM_.GetCastLine(p_, p_chk, path);
        for(auto &p : path){
            if(!BM_.GetVoxState(p) != VoxelState::free) {
                free = false;
                break;
            }
        }
        if(free && dist > cur_d){
            dist = cur_d;
            recover_pose_.block(0, 0, 3, 1) = p_chk;
            recover_pose_(3) = yaw_;
        }
    }
    
    if(dist < 99998.0){
        PublishTraj(true);
        return true;
    }
    else{
        if(LRM_.IsFeasible(last_safe_)){
            recover_pose_.block(0, 0, 3, 1) = last_safe_;
            recover_pose_(3) = yaw_;
            PublishTraj(true);
            return true;
        }
        return false;
    }
}

void Murder::Stay(const Eigen::Vector4d &pose){
    recover_pose_ = pose;
    PublishTraj(true);
}

int Murder::Replan(const bool &new_target, const bool &ignore_duration){
    double cur_t = ros::WallTime::now().toSec();
    if(!ignore_duration && cur_t - traj_start_t_ < exc_duration_) return 0;
    Eigen::Vector4d target_pose;
    Eigen::Vector3d ps, vs, as, pe, ve, ae;
    double ys, yds, ydds, ye, yde, ydde;
    pe = target_.head(3);
    ye = target_(3);
    bool dead_target = (!LRM_.IsFeasible(pe));

    /* replan , dead_target, force go to a new target or target vp dead, change to local or global mode */
    if(!dead_target || cur_t > replan_t_ || new_target || 
        !(FG_.GetVp(target_f_id_, target_v_id_, target_pose) && FG_.f_grid_[target_f_id_].local_vps_[target_v_id_] == 1)){
        return 1;
        // list<int> local_f;
        // MDTG_.GetAllLocalFroVps(local_f);
        // if(local_f.empty()){
        //     if(FG_.SampleVps()){
        //         return 1;
        //     }
        //     else return 2;
        // }
        // if(!local_f.empty()) return 1;
    }
    Eigen::Vector3d target_pos = target_pose.block(0, 0, 3, 1);

    /* try to plan a new traj. If fails, change to plan mode */
    double hand_t = ros::WallTime::now().toSec() + reach_out_t_; 
    if(hand_t > traj_end_t_){
        ps = p_;
        vs = v_;
        as.setZero();
        ve.setZero();
        ae.setZero();
        if(YawP_.T_.size() != 0) YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        else ys = yaw_;        // ys = yaw_;
        yds = yaw_v_;
        ydds = 0;
        yde = 0; 
        ydde = 0;
        // cout<<"new"<<endl;
        // cout<<"feas:"<<LRM_.IsFeasible(ps)<<endl;
    }
    else{
        ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
        while(!LRM_.IsFeasible(ps) && hand_t > cur_t){
            hand_t -= reach_out_t_ / 10;
            ps = TrajOpt_.traj.getPos(hand_t - traj_start_t_);
            // cout<<"handt:"<<hand_t<<endl;
        }
        if(!LRM_.IsFeasible(ps)){
            hand_t = cur_t;
            // cout<<"handct:"<<hand_t<<endl;
            ps = p_;
        }
        // cout<<"re"<<endl;
        // cout<<"feas:"<<LRM_.IsFeasible(ps)<<endl;
        vs = TrajOpt_.traj.getVel(hand_t - traj_start_t_);
        as = TrajOpt_.traj.getAcc(hand_t - traj_start_t_);
        ve.setZero();
        ae.setZero();
        YawP_.GetCmd(hand_t - traj_start_t_, ys, yds, ydds);
        yde = 0; 
        ydde = 0;
    }
    // ve = GetEndV(target_f_id_, target_v_id_, ps, vs, dangerous_path_ || !reach_end_traj_);
    // cout<<"ps:"<<ps.transpose()<<" vs:"<<vs.transpose()<<" as:"<<as.transpose()<<" pe:"<<pe.transpose()<<" ys:"<<ys<<" ye:"<<ye<<endl;
    if(LRM_.IsFeasible(pe) && TrajPlanB(ps, vs, as, pe, ve, 
                ae, ys, yds, ydds, target_(3), yde, ydde, FG_.f_grid_[target_f_id_].center_, hand_t)){
        traj_start_t_ = hand_t;
        traj_end_t_ = TrajOpt_.traj.getTotalDuration() + traj_start_t_;
        replan_t_ = min(replan_duration_, TrajOpt_.traj.getTotalDuration()) + traj_start_t_;
        PublishTraj(false);
        reach_end_traj_ = false;
        return 0;
    }
    else{
        return 1;
        // list<int> local_f;
        // MDTG_.GetAllLocalFroVps(local_f);
        // if(local_f.empty()){
        //     if(FG_.SampleVps()){
        //         return 1;
        //     }
        //     else return 2;
        // }
        // else return 1;
    }
}

int Murder::SwitchMode(){
    // if(!FG_.sample_flag_){
    //     FG_.SampleVps();
    // }
    if(GVP_.LocalExplorable()) return 1;
    else if(GVP_.GlobalExplorable()) return 2;
    else return 0;
}

int Murder::ViewPointsCheck(const double &t){
    list<int> local_f;
    double start_t = ros::WallTime::now().toSec();
    double remain_t = t;
    MDTG_.GetAllLocalFroVps(local_f);
    for(auto &f : local_f){
        if(start_t - FG_.f_grid_[f].last_strong_check_ > strong_check_interval_){
            for(int v = 0; v < FG_.f_grid_[f].local_vps_.size(); v++){
                if(FG_.f_grid_[f].local_vps_[v] != 1) continue;
                if(!FG_.StrongCheckViewpoint(f, v, true)){
                    if(f >= 0 && f < FG_.f_grid_.size() && 0 <= v && v < FG_.f_grid_[f].local_vps_.size() && f != target_f_id_ && v != target_v_id_)
                        MDTG_.RemoveVp(FG_.f_grid_[f].center_, f, v, true);
                }
            }
            FG_.f_grid_[f].last_strong_check_ = start_t + strong_check_interval_;
        }
        remain_t = ros::WallTime::now().toSec() - start_t;//not right
        if(remain_t < 0) break;
    }
    
    Eigen::Vector3d tar_p(target_vp_pose_(0), target_vp_pose_(1), target_vp_pose_(2));
    double dp, dyaw;
    dyaw = abs(yaw_ - target_vp_pose_(3));
    if(abs(dyaw) > 2 * M_PI) dyaw = dyaw - floor(dyaw / (M_PI*2)) * M_PI * 2; 
    dp = (tar_p - p_).norm();
    if(!FG_.StrongCheckViewpoint(target_f_id_, target_v_id_, true) || (dp < 1.0 && dyaw < 0.5)){
        if(target_f_id_ >= 0 && target_f_id_ < FG_.f_grid_.size() && 0 <= target_v_id_ && target_v_id_ < FG_.f_grid_[target_f_id_].local_vps_.size())
            MDTG_.RemoveVp(FG_.f_grid_[target_f_id_].center_, target_f_id_, target_v_id_, true);
        // list<int> local_f;
        // MDTG_.GetAllLocalFroVps(local_f);
        // if(local_f.empty()){
        //     if(FG_.SampleVps()){
        //         return 1;
        //     }
        //     else return 2;
        // }
        return 1;
    }
    return 0;
}

// bool Murder::TrajPlanA(vector<Eigen::Vector3d> &safe_path, const Eigen::Vector3d &ps, const Eigen::Vector3d &vs, const Eigen::Vector3d &as,
//             const Eigen::Vector3d &pe, const Eigen::Vector3d &ve, const Eigen::Vector3d &ae, const double &yps, 
//             const double &yds, const double &ydds, const double &ype, const double &yde, const double &ydde, const Eigen::Vector3d &gazept){
//     vector<Eigen::Vector3d> path_pruned;
//     vector<Eigen::MatrixX4d> h;
//     vector<Eigen::Matrix3Xd> p;
//     // if(LRM_.GetPath(ps, pe, path, false, local_max_search_iter_)){
        
//         if(LRM_.FindCorridors(safe_path, h, p, path_pruned, traj_length_)){
//             Eigen::Matrix3d startpva, endpva;
//             double min_t = YawP_.GetMinT(yps, ype);
//             startpva.setZero();
//             endpva.setZero();
//             startpva.col(0) = ps;
//             startpva.col(1) = vs;
//             startpva.col(2) = as;
//             endpva.col(0) = path_pruned.back();
//             if(TrajOpt_.Optimize(path_pruned, h, p, min_t, startpva, endpva, SDM_.trajs_, )){
//                 Eigen::Vector3d traj_end = TrajOpt_.traj.getPos(TrajOpt_.traj.getTotalDuration());
//                 ShowTraj(safe_path, h);
//                 for(auto &p : safe_path){//debug
//                     if(!LRM_.IsFeasible(p)){
//                         ROS_ERROR("path dead!");
//                         ros::shutdown();
//                         return false;
//                     }
//                 }
//                 double yaw_end = ype;
//                 if((traj_end - safe_path.back()).norm() < 3.5){
//                     if((traj_end - safe_path.back()).norm() > 0.1){
//                         yaw_end = atan2(gazept(1) - traj_end(1), gazept(0) - traj_end(0));
//                     }
//                     PlanYaw(yps, yds, ydds, yaw_end, yde, ydde, gazept, true);
//                 }
//                 else{
//                     yaw_end = atan2(gazept(1) - traj_end(1), gazept(0) - traj_end(0));
//                     PlanYaw(yps, yds, ydds, yaw_end, yde, ydde, gazept, false);

//                 }            
//                 return true;
//             }
//             else{
//                 cout<<"min_t:"<<min_t<<"d:"<<(ps - pe).norm()<<endl;
//                 ROS_ERROR("opt failed");
//                 return false;
//             }
//         }
//         else{
//             ROS_WARN("id:%d fail", SDM_.self_id_);
//             return false;
//         }
//     // }
//     // else{
//     //     cout<<"r:"<<LRM_.IsFeasible(p_)<<endl;
//     //     cout<<"s:"<<LRM_.IsFeasible(ps)<<endl;
//     //     cout<<"e:"<<LRM_.IsFeasible(pe)<<endl;
//     //     cout<<p_.transpose()<<"  "<<ps.transpose()<<endl;
//     //     ROS_ERROR("no path");
//     //     ros::shutdown();
//     //     return false;
//     // }
// }

bool Murder::TrajPlanB(const Eigen::Vector3d &ps, const Eigen::Vector3d &vs, const Eigen::Vector3d &as,
            const Eigen::Vector3d &pe, const Eigen::Vector3d &ve, const Eigen::Vector3d &ae, const double &yps, 
            const double &yds, const double &ydds, const double &ype, const double &yde, const double &ydde, const Eigen::Vector3d &gazept, const double &handt){
    vector<Eigen::Vector3d> path, path_pruned;
    vector<Eigen::MatrixX4d> h;
    vector<Eigen::Matrix3Xd> p;
    vector<Eigen::Vector3d> swarm_p;
    reach_end_traj_ = false;
    if(LRM_.GetPath(ps, pe, path, false, local_max_search_iter_)){

        if(LRM_.FindCorridors(path, h, p, path_pruned, traj_length_)){
            Eigen::Matrix3d startpva, endpva;
            double min_t = YawP_.GetMinT(yps, ype) - 1e-3;
            startpva.setZero();
            endpva.setZero();
            startpva.col(0) = ps;
            startpva.col(1) = vs;
            startpva.col(2) = as;
            endpva.col(0) = path_pruned.back();
            if((path_pruned.back() - path.back()).norm() < 0.3) {
                endpva.col(1) = ve;
                reach_end_traj_ = true;
            }
            SDM_.GetLocalSwarmPos(swarm_p);
            if(TrajOpt_.Optimize(path_pruned, h, p, min_t, startpva, endpva, SDM_.self_id_, SDM_.trajs_, SDM_.start_t_, swarm_p, handt)){
                Eigen::Vector3d traj_end = TrajOpt_.traj.getPos(TrajOpt_.traj.getTotalDuration());
                ShowTraj(path, h);
                // for(auto &p : path){//debug
                //     if(!LRM_.IsFeasible(p)){
                //         ROS_ERROR("path dead!");
                //         ros::shutdown();
                //         return false;
                //     }
                // }
                double yaw_end = ype;
                if((traj_end - path.back()).norm() < 3.5){
                    if((traj_end - path.back()).norm() > 2.0){
                        yaw_end = atan2(gazept(1) - traj_end(1), gazept(0) - traj_end(0));
                    }
                    PlanYaw(yps, yds, ydds, yaw_end, yde, ydde, gazept, true);
                }
                else{
                    yaw_end = atan2(gazept(1) - traj_end(1), gazept(0) - traj_end(0));
                    PlanYaw(yps, yds, ydds, yaw_end, yde, ydde, gazept, false);

                }            
                return true;
            }
            else{
                cout<<"id:"<<int(SDM_.self_id_)<<"min_t:"<<min_t<<"d:"<<(startpva.col(0) - endpva.col(0)).norm()<<"  dyaw:"<<yps-ype<<endl;
                ROS_ERROR("opt failed");
                return false;
            }
        }
        else{
            ROS_WARN("id:%d fail", SDM_.self_id_);
            return false;
        }
    }
    else{
        cout<<"r:"<<LRM_.IsFeasible(p_)<<endl;
        cout<<"s:"<<LRM_.IsFeasible(ps)<<endl;
        cout<<"e:"<<LRM_.IsFeasible(pe)<<endl;
        cout<<p_.transpose()<<"  "<<ps.transpose()<<endl;
        cout<<pe.transpose()<<endl;
        ROS_ERROR("id: %d no path", SDM_.self_id_);
        // ros::shutdown();
        return false;
    }
}

void Murder::PlanYaw(const double &yps, const double &yds, const double &ydds, const double &ype, 
            const double &yde, const double &ydde, const Eigen::Vector3d &gazept, bool gaze){
    Eigen::VectorXd T, yaw_l;
    double total_t = TrajOpt_.traj.getTotalDuration();
    YawP_.SampleT(total_t, T);
    yaw_l.resize(T.size()+1);
    yaw_l(0) = yps;
    double tc = 0.0;
    for(int i = 0; i+1 < T.size(); i++){
        tc += T(i);
        Eigen::Vector3d v = TrajOpt_.traj.getVel(tc);
        Eigen::Vector3d p = TrajOpt_.traj.getPos(tc);
        double yaw = atan2(v(1), v(0));
        double yaw_gaze = atan2(gazept(1) - p(1), gazept(0) - p(0));
        if(i == 0 && gaze && T.size() == 2){     //gaze at the second yaw, if only have 3 yaws
            yaw_l(i+1) = YawP_.GetClosestYaw(T(i), yaw_l(i), yds, yaw_gaze);
            // Debug(p, gazept);
        }
        else if(i == 0)                         //dont gaze
            yaw_l(i+1) = YawP_.GetClosestYaw(T(i), yaw_l(i), yds, yaw);
        else if(T.size() > 2 && i == T.size() - 1){ //gaze at the last second yaw, if have more than 3 yaws 
            yaw_l(i+1) = YawP_.GetClosestYaw(T(i), yaw_l(i), 0, yaw_gaze);
            // Debug(p, gazept);
        }
        else
            yaw_l(i+1) = yaw;
    }

    yaw_l.tail(1)(0) = ype;
    YawP_.Plan(yaw_l, T, yds, 0.0, yde, 0.0);
    double p, v, a;
    YawP_.GetCmd(T.sum(), p, v, a);

}

void Murder::GetFollowPath(list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &path_follow){
    double follow_dist = 0;
    path_follow = path;
    if(path.size() <= 1) return;
    Eigen::Vector3d last_pop = path_follow.back();
    path_follow.pop_back();
    while(!path_follow.empty()){
        follow_dist += (path_follow.back() - last_pop).norm();
        if(follow_dist > FG_.GetSensorRange()) break;
        last_pop = path_follow.back();
        path_follow.pop_back();
    }
}

void Murder::PublishTraj(bool recover){
    swarm_exp_msgs::LocalTraj traj;
    if(recover){        //recover
        traj.state = 1;
        traj.recover_pt.x = recover_pose_.x();
        traj.recover_pt.y = recover_pose_.y();
        traj.recover_pt.z = recover_pose_.z();
    }
    else{               //normal traj
        traj.state = 2;
        traj.start_t = traj_start_t_;
        traj.coef_p.resize(TrajOpt_.traj.getPieceNum() * 6);
        traj.t_p.resize(TrajOpt_.traj.getPieceNum());
        traj.order_p = 5;
        for(int i = 0; i < TrajOpt_.traj.getPieceNum(); i++){
            auto &cur_p = TrajOpt_.traj[i];
            Eigen::MatrixXd cM;
            cM = cur_p.getCoeffMat();
            traj.t_p[i] = cur_p.getDuration();
            for(int j = 0; j < cM.cols(); j++){
                traj.coef_p[j + i * 6].x = cM(0, j);
                traj.coef_p[j + i * 6].y = cM(1, j);
                traj.coef_p[j + i * 6].z = cM(2, j);
            }
        }

        traj.order_yaw = 5;
        traj.t_yaw.resize(YawP_.T_.size());
        traj.coef_yaw.resize(YawP_.A_.size());
        for(int i = 0; i < YawP_.A_.size(); i++){
            traj.coef_yaw[i] = YawP_.A_[i];
        }
        for(int i = 0; i < YawP_.T_.size(); i++){
            traj.t_yaw[i] = YawP_.T_[i];
        }
    }
    SDM_.SetTraj(TrajOpt_.traj, traj_start_t_);
    traj_pub_.publish(traj);
}

void Murder::ImgOdomCallback(const sensor_msgs::ImageConstPtr& img,
                            const nav_msgs::OdometryConstPtr& odom){
    if(ros::WallTime::now().toSec() - last_map_update_t_ < 0.2 || !have_odom_) return;
    last_map_update_t_ = ros::WallTime::now().toSec(); 
    ros::WallTime t0 = ros::WallTime::now();
    BM_.OdomCallback(odom);
    // if(FG_.sensor_type_ == SensorType::CAMERA)
    BM_.InsertImg(img);
    FG_.UpdateFrontier(BM_.newly_register_idx_);
    MDTG_.Update(robot_pose_, !sensor_flag_);
    sensor_flag_ = true;
}

void Murder::PCLOdomCallback(const sensor_msgs::PointCloud2ConstPtr& pcl,
                            const nav_msgs::OdometryConstPtr& odom){
    if(ros::WallTime::now().toSec() - last_map_update_t_ < 0.1 || !have_odom_) return;
    last_map_update_t_ = ros::WallTime::now().toSec(); 
    ros::WallTime t0 = ros::WallTime::now();
    BM_.OdomCallback(odom);
    
    BM_.InsertPcl(pcl);
    // cout<<"dt11111111111111: "<<ros::WallTime::now().toSec() - last_map_update_t_<<endl;
    FG_.UpdateFrontier(BM_.newly_register_idx_);
    MDTG_.Update(robot_pose_, !sensor_flag_);
    sensor_flag_ = true;
}

Eigen::Vector3d Murder::GetEndV(uint16_t f_id, uint8_t v_id, Eigen::Vector3d ps, Eigen::Vector3d vs, bool dangerous){
    if(dangerous) return Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d c = FG_.f_grid_[f_id].center_;
    Eigen::Vector3d vp, vnorm, vit, ve;    
    FG_.GetVpPos(f_id, v_id, vp);
    vnorm = (c - vp).normalized();
    if(!FG_.HaveAliveNeighbour(f_id, vnorm)) return Eigen::Vector3d(0, 0, 0);
    double acc_l = TrajOpt_.upboundVec_[1] + acc_off_;
    double max_dacc_dist_ = TrajOpt_.upboundVec_[0] * TrajOpt_.upboundVec_[0] / 2 / (acc_l);
    double v_m = min(sqrt(2.0 * (ps - vp).norm() * (acc_l) / TrajOpt_.upboundVec_[0]), TrajOpt_.upboundVec_[0] * 0.5);
    double d = 0;
    for(; d < max_dacc_dist_; d += 0.1){
        vit = vp + vnorm * d;
        if(!LRM_.IsFeasible(vit)){
            d = max(d - 0.1, 0.0);
            break;
        }
    }
    ve = vnorm * min(sqrt(2*d*(acc_l)), v_m);
    double acc_t = (ve - vs).norm() / (acc_l);
    if(acc_t < (ps - vp).norm()/(TrajOpt_.upboundVec_[0])){
        ve = vnorm * min(max(0.0, (acc_t - vs.norm() / (acc_l)) * acc_l), v_m);
    }
    return ve;
}

void Murder::BodyOdomCallback(const nav_msgs::OdometryConstPtr& odom){
    Quaterniond qua;
    have_odom_ = true;
    robot_pose_.setZero();
    qua.x() = odom->pose.pose.orientation.x;
    qua.y() = odom->pose.pose.orientation.y;
    qua.z() = odom->pose.pose.orientation.z;
    qua.w() = odom->pose.pose.orientation.w;

    p_(0) = odom->pose.pose.position.x;
    p_(1) = odom->pose.pose.position.y;
    p_(2) = odom->pose.pose.position.z;
    FG_.Robot_pos_ = p_;
    
    robot_pose_.block(0, 0, 3, 3) = qua.toRotationMatrix();
    robot_pose_.block(0, 3, 3, 1) = p_;

    v_(0) = odom->twist.twist.linear.x;
    v_(1) = odom->twist.twist.linear.y;
    v_(2) = odom->twist.twist.linear.z;
    // if(v_.norm() > 5.0) ROS_ERROR("large v!");//debug
    v_ = qua.toRotationMatrix() * v_;
    yaw_ = atan2(qua.matrix()(1, 0), qua.matrix()(0, 0));
    yaw_v_ = odom->twist.twist.angular.z * robot_pose_(2, 2) + odom->twist.twist.angular.y * robot_pose_(2, 1)
                + odom->twist.twist.angular.x * robot_pose_(2, 0);
    SDM_.SetPose(*odom);

    // LoadShowPose(odom->pose.pose);
    // posevis_pub_.publish(vis_model_);
}

void Murder::CreateVisModel(){
    vis_model_.markers.resize(5);
    vis_model_.markers[0].header.frame_id = "world";
    vis_model_.markers[0].header.stamp = ros::Time::now();
    vis_model_.markers[0].id = 0;
    vis_model_.markers[0].action = visualization_msgs::Marker::ADD;
    vis_model_.markers[0].type = visualization_msgs::Marker::SPHERE;
    vis_model_.markers[0].scale.x = 0.30;
    vis_model_.markers[0].scale.y = 0.30;
    vis_model_.markers[0].scale.z = 0.04;
    vis_model_.markers[0].color = CM_.Id2Color(SDM_.self_id_, 1.0);

    vis_model_.markers[1] = vis_model_.markers[0];
    vis_model_.markers[1].id = 1;
    vis_model_.markers[2] = vis_model_.markers[0];
    vis_model_.markers[2].id = 2;
    vis_model_.markers[3] = vis_model_.markers[0];
    vis_model_.markers[3].id = 3;

    vis_model_.markers[4] = vis_model_.markers[0];
    vis_model_.markers[4].id = 4;
    vis_model_.markers[4].type = visualization_msgs::Marker::LINE_LIST;
    vis_model_.markers[4].scale.x = 0.025;
    vis_model_.markers[4].scale.y = 0.025;
    vis_model_.markers[4].scale.z = 0.025;

    geometry_msgs::Point pt;

    // pt.x = 0.225;
    // pt.y = 0.225;
    // pt.z = 0;
    // vis_model_.markers[0].points.emplace_back(pt);
    // pt.y = -0.225;
    // vis_model_.markers[0].points.emplace_back(pt);
    // pt.x = -0.225;
    // vis_model_.markers[0].points.emplace_back(pt);
    // pt.y = 0.225;
    // vis_model_.markers[0].points.emplace_back(pt);

    pt.x = 0.225;
    pt.y = 0.225;
    pt.z = -0.02;
    vis_model_.markers[4].points.emplace_back(pt);
    pt.y = -0.225;
    pt.x = -0.225;
    vis_model_.markers[4].points.emplace_back(pt);
    pt.x = -0.225;
    pt.y = 0.225;
    vis_model_.markers[4].points.emplace_back(pt);
    pt.x = 0.225;
    pt.y = -0.225;
    vis_model_.markers[4].points.emplace_back(pt);
}

void Murder::ShowTraj(vector<Eigen::Vector3d> &path, vector<Eigen::MatrixX4d> &h){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1 + h.size());
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = -1;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    mka.markers[0].scale.x = 0.07;
    mka.markers[0].scale.y = 0.07;
    mka.markers[0].scale.z = 0.07;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.9;
    mka.markers[0].color.g = 0.9;
    mka.markers[0].color.b = 0.9;
    if(dangerous_path_){
        mka.markers[0].color.a = 1.0;
        mka.markers[0].color.r = 0.99;
        mka.markers[0].color.g = 0.0;
        mka.markers[0].color.b = 0.0;
    }

    // for(double delta = 0; delta < TrajOpt_.traj.getTotalDuration(); delta += 0.025){
    //     Eigen::Vector3d p;
    //     geometry_msgs::Point pt;
    //     p = TrajOpt_.traj.getPos(delta);
    //     pt.x = p(0);
    //     pt.y = p(1);
    //     pt.z = p(2);
    //     mka.markers[0].points.emplace_back(pt);
    // }
    for(auto &p : path){
        geometry_msgs::Point pt;
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mka.markers[0].points.emplace_back(pt);
    }
    for(int i = 0; i < h.size(); i++){
        mka.markers[i + 1].pose.position.x = (h[i](1, 3) - h[i](0, 3)) / 2;
        mka.markers[i + 1].pose.position.y = (h[i](3, 3) - h[i](2, 3)) / 2;
        mka.markers[i + 1].pose.position.z = (h[i](5, 3) - h[i](4, 3)) / 2;
        mka.markers[i + 1].pose.orientation.w = 1.0;

        mka.markers[i + 1].scale.x = (- h[i](1, 3) - h[i](0, 3));
        mka.markers[i + 1].scale.y = (- h[i](3, 3) - h[i](2, 3));
        mka.markers[i + 1].scale.z = (- h[i](5, 3) - h[i](4, 3));
        mka.markers[i + 1].type = visualization_msgs::Marker::CUBE;
        mka.markers[i+1].header.frame_id = "world";
        mka.markers[i+1].header.stamp = ros::Time::now();
        mka.markers[i+1].id = 1+i;
        mka.markers[i+1].action = visualization_msgs::Marker::ADD;
        mka.markers[i+1].lifetime = ros::Duration(1.0);
        mka.markers[i + 1].color.a = 0.2;
        mka.markers[i + 1].color.g = 1.0;
        mka.markers[i + 1].color.b = 0.5;
        if(dangerous_path_){
            mka.markers[i + 1].color.a = 0.2;
            mka.markers[i + 1].color.r = 0.9;
            mka.markers[i + 1].color.g = 0.2;
            mka.markers[i + 1].color.b = 0.2;
        }
    }
    show_pub_.publish(mka);
}

void Murder::Debug(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 2;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    mka.markers[0].scale.x = 0.05;
    mka.markers[0].scale.y = 0.05;
    mka.markers[0].scale.z = 0.05;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.9;
    mka.markers[0].color.g = 0.9;
    mka.markers[0].color.b = 0.0;
    
    // for(double delta = 0; delta < TrajOpt_.traj.getTotalDuration(); delta += 0.025){
    //     Eigen::Vector3d p;
    //     geometry_msgs::Point pt;
    //     p = TrajOpt_.traj.getPos(delta);
    //     pt.x = p(0);
    //     pt.y = p(1);
    //     pt.z = p(2);
    //     mka.markers[0].points.emplace_back(pt);
    // }
    mka.markers[0].points.resize(2);
    mka.markers[0].points[0].x = pt1.x();
    mka.markers[0].points[0].y = pt1.y();
    mka.markers[0].points[0].z = pt1.z();
    mka.markers[0].points[1].x = pt2.x()*0.3 + pt1.x()*0.7;
    mka.markers[0].points[1].y = pt2.y()*0.3 + pt1.y()*0.7;
    mka.markers[0].points[1].z = pt2.z()*0.3 + pt1.z()*0.7;
    show_pub_.publish(mka);
}

void Murder::Debug(list<Eigen::Vector3d> &pts){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = -2;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.1;
    mka.markers[0].scale.y = 0.1;
    mka.markers[0].scale.z = 0.1;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.9;
    mka.markers[0].color.g = 0.0;
    mka.markers[0].color.b = 0.5;
    for(auto &pt : pts){
        geometry_msgs::Point p;
        p.x = pt(0);
        p.y = pt(1);
        p.z = pt(2);
        mka.markers[0].points.emplace_back(p);
    }
    show_pub_.publish(mka);
}

void Murder::ShowPath(list<Eigen::Vector3d> &path, int id){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = id;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    mka.markers[0].scale.x = 0.1;
    mka.markers[0].scale.y = 0.1;
    mka.markers[0].scale.z = 0.1;
    mka.markers[0].color.a = 0.8;
    if(id == -4){
        mka.markers[0].pose.position.x = 0.1;
        mka.markers[0].pose.position.y = 0.1;
        mka.markers[0].color.r = 0.9;
        mka.markers[0].color.g = 0.0;
        mka.markers[0].color.b = 0.0;
    }
    if(id == -3){
        mka.markers[0].scale.x = 0.05;
        mka.markers[0].scale.y = 0.05;
        mka.markers[0].scale.z = 0.05;
        mka.markers[0].color.r = 0.2;
        mka.markers[0].color.g = 0.4;
        mka.markers[0].color.b = 0.9;
    }
    for(auto &pt : path){
        geometry_msgs::Point p;
        p.x = pt(0);
        p.y = pt(1);
        p.z = pt(2);
        mka.markers[0].points.emplace_back(p);
    }
    show_pub_.publish(mka);
}