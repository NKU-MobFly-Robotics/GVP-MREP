#include<murder_swarm/ground.h>

void Ground::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){
    std::string ns = ros::this_node::getName();
    // nh_private_.param(ns + "/Exp/traj_length", traj_length_, 10.0);
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

    CreateVisModels();

    traj_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/Trajs", 5);
    pose_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/Poses", 5);
    traj_timer_ = nh_.createTimer(ros::Duration(0.05), &Ground::TrajVisCallback, this);
    pose_timer_ = nh_.createTimer(ros::Duration(0.05), &Ground::PoseVisCallback, this);
}

void Ground::CreateVisModels(){
    vis_models_.markers.resize(5 * SDM_.drone_num_);
    poses_.resize(SDM_.drone_num_);
    for(int i = 0; i < SDM_.drone_num_; i++){
        vis_models_.markers[i*5 + 0].header.frame_id = "world";
        vis_models_.markers[i*5 + 0].header.stamp = ros::Time::now();
        vis_models_.markers[i*5 + 0].id = i*5 + 0;
        vis_models_.markers[i*5 + 0].action = visualization_msgs::Marker::ADD;
        vis_models_.markers[i*5 + 0].type = visualization_msgs::Marker::SPHERE;
        vis_models_.markers[i*5 + 0].scale.x = 0.30;
        vis_models_.markers[i*5 + 0].scale.y = 0.30;
        vis_models_.markers[i*5 + 0].scale.z = 0.04;
        vis_models_.markers[i*5 + 0].color = CM_.Id2Color(i+1, 1.0);
        poses_[i].orientation.w = 1.0;
        poses_[i].orientation.x = 0.0;
        poses_[i].orientation.y = 0.0;
        poses_[i].orientation.z = 0.0;
        poses_[i].position.x = 0.0;
        poses_[i].position.y = 0.0;
        poses_[i].position.z = 0.0;

        vis_models_.markers[i*5 + 1] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 1].id = i*5 + 1;
        vis_models_.markers[i*5 + 2] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 2].id = i*5 + 2;
        vis_models_.markers[i*5 + 3] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 3].id = i*5 + 3;

        vis_models_.markers[i*5 + 4] = vis_models_.markers[i*5 + 0];
        vis_models_.markers[i*5 + 4].id = i*5 + 4;
        vis_models_.markers[i*5 + 4].type = visualization_msgs::Marker::LINE_LIST;
        vis_models_.markers[i*5 + 4].scale.x = 0.025;
        vis_models_.markers[i*5 + 4].scale.y = 0.025;
        vis_models_.markers[i*5 + 4].scale.z = 0.025;

        geometry_msgs::Point pt;

        pt.x = 0.225;
        pt.y = 0.225;
        pt.z = -0.02;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
        pt.y = -0.225;
        pt.x = -0.225;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
        pt.x = -0.225;
        pt.y = 0.225;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
        pt.x = 0.225;
        pt.y = -0.225;
        vis_models_.markers[i*5 + 4].points.emplace_back(pt);
    }
}

void Ground::TrajVisCallback(const ros::TimerEvent &e){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(SDM_.drone_num_);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.1;
    mka.markers[0].scale.y = 0.1;
    mka.markers[0].scale.z = 0.1;
    mka.markers[0].pose.orientation.w = 1;
    mka.markers[0].pose.orientation.x = 0;
    mka.markers[0].pose.orientation.y = 0;
    mka.markers[0].pose.orientation.z = 0;
    mka.markers[0].color.a = 0.7;
    mka.markers[0].color.r = 0.3;
    mka.markers[0].color.g = 0.9;
    mka.markers[0].color.b = 0.2;
    double cur_t = ros::WallTime::now().toSec();
    geometry_msgs::Point pt;
    for(int i = 0; i < SDM_.drone_num_; i++){
        mka.markers[i] = mka.markers[0];
        mka.markers[i].id = i;
        double total_t = SDM_.trajs_[i].getTotalDuration();
        if(cur_t - SDM_.start_t_[i] < total_t) {
            for(double dt = 0; dt < total_t; dt += 0.05){
                Eigen::Vector3d p;
                p = SDM_.trajs_[i].getPos(dt);
                pt.x = p(0);
                pt.y = p(1);
                pt.z = p(2);
                mka.markers[i].points.emplace_back(pt);
            }
        }
        if(mka.markers[i].points.empty()) {
            mka.markers[i].action = visualization_msgs::Marker::DELETE;
        }
    }
    traj_vis_pub_.publish(mka);
}

void Ground::PoseVisCallback(const ros::TimerEvent &e){
    LoadVisModels();
    pose_vis_pub_.publish(vis_models_);
}
