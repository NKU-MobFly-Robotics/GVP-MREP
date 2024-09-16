#include <block_map/block_map.h>

void BlockMap::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){

    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    
    vector<double> CR, CB, CG;
    bool depth;
    Eigen::Quaterniond cam2bodyrot;
    Eigen::Vector3d robots_scale;

    cam2body_.setZero();
    int drone_num, self_id;
    nh_private_.param(ns + "/block_map/HeightcolorR", 
        CR, {});
    nh_private_.param(ns + "/block_map/HeightcolorG", 
        CG, {});
    nh_private_.param(ns + "/block_map/HeightcolorB", 
        CB, {});
    nh_private_.param(ns + "/block_map/minX", 
        origin_.x(), -10.0);
    nh_private_.param(ns + "/block_map/minY", 
        origin_.y(), -10.0);
    nh_private_.param(ns + "/block_map/minZ", 
        origin_.z(), 0.0);
    nh_private_.param(ns + "/block_map/maxX", 
        map_upbd_.x(), 10.0);
    nh_private_.param(ns + "/block_map/maxY", 
        map_upbd_.y(), 10.0);
    nh_private_.param(ns + "/block_map/maxZ", 
        map_upbd_.z(), 0.0);
    nh_private_.param(ns + "/block_map/blockX", 
        block_size_.x(), 5);
    nh_private_.param(ns + "/block_map/blockY", 
        block_size_.y(), 5);
    nh_private_.param(ns + "/block_map/blockZ", 
        block_size_.z(), 3);
    nh_private_.param(ns + "/block_map/resolution", 
        resolution_, 0.2);
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        max_range_, 4.5);
    nh_private_.param(ns + "/block_map/depth", 
        depth_, false);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_x",
        cam2bodyrot.x(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_y",
        cam2bodyrot.y(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_z",
        cam2bodyrot.z(), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_Quater_w",
        cam2bodyrot.w(), 1.0);
    nh_private_.param(ns + "/block_map/CamtoBody_x",
        cam2body_(0, 3), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_y",
        cam2body_(1, 3), 0.0);
    nh_private_.param(ns + "/block_map/CamtoBody_z",
        cam2body_(2, 3), 0.0);
    nh_private_.param(ns + "/block_map/update_freq", 
        update_interval_, 5.0);
    nh_private_.param(ns + "/block_map/show_freq", 
        show_freq_, 2.0);
    nh_private_.param(ns + "/block_map/depth_step", 
        depth_step_, 2);
    nh_private_.param(ns + "/block_map/occ_max", 
        thr_max_, 0.9);
    nh_private_.param(ns + "/block_map/occ_min", 
        thr_min_, 0.1);
    nh_private_.param(ns + "/block_map/pro_hit_occ", 
        pro_hit_, 0.7);
    nh_private_.param(ns + "/block_map/pro_miss_free", 
        pro_miss_, 0.8);    
    nh_private_.param(ns + "/block_map/statistic_v", 
        stat_, false);
    nh_private_.param(ns + "/block_map/min_finish_t", 
        min_finish_t_, 30.0);
    nh_private_.param(ns + "/Exp/maxX", 
        stat_upbd_.x(), -10.0);
    nh_private_.param(ns + "/Exp/maxY", 
        stat_upbd_.y(), -10.0);
    nh_private_.param(ns + "/Exp/maxZ", 
        stat_upbd_.z(), 0.0);
    nh_private_.param(ns + "/Exp/minX", 
        stat_lowbd_.x(), 10.0);
    nh_private_.param(ns + "/Exp/minY", 
        stat_lowbd_.y(), 10.0);
    nh_private_.param(ns + "/Exp/minZ", 
        stat_lowbd_.z(), 0.0);
    nh_private_.param(ns + "/block_map/RobotVisX", 
        robots_scale.x(), 0.5);
    nh_private_.param(ns + "/block_map/RobotVisY", 
        robots_scale.y(), 0.5);
    nh_private_.param(ns + "/block_map/RobotVisZ", 
        robots_scale.z(), 0.3);
    nh_private_.param(ns + "/block_map/swarm_pub_thresh", 
        swarm_pub_thresh_, 0.95);
        
    // nh_private.param(ns + "/Exp/drone_num", 
    //     drone_num, 0);
    // nh_private.param(ns + "/Exp/UAV_id", 
    //     self_id, 1);
    nh_private.param(ns + "/block_map/swarm_tol", 
        swarm_tol_, 0.2);
    nh_private.param(ns + "/block_map/swarm_send_delay", 
        swarm_send_delay_, 0.2);
    nh_private.param(ns + "/block_map/show_block", 
        show_block_, true);

    // if(SDM_->is_ground_){
    //     debug_l_.open("/home/charliedog/rosprojects/MURDER/debug.txt", std::ios::out);
    // }
    start_t_ = ros::WallTime::now().toSec();
    req_flag_ = false;
    finish_flag_ = false;
    if(SDM_->drone_num_ > 1){  //swarm exploration
        use_swarm_ = true;
        drone_num = SDM_->drone_num_;
        swarm_filter_dict_ = new tr1::unordered_map<int, int>;
        swarm_pose_.resize(drone_num);
        cout<<"robots_scale_:"<<robots_scale.transpose()<<endl;
        for(int i = 0; i < drone_num; i++) robots_scale_.emplace_back(robots_scale);
        for(auto &sp : swarm_pose_) sp.first = ros::WallTime::now().toSec() - 1000.0;
        self_id_ = SDM_->self_id_;
    }
    else {
        drone_num = 1;
        use_swarm_ = false;
        self_id = 1;
    }
    stat_n_ = 0;

    cout<<pro_hit_<<" "<<pro_miss_<<endl;

    cam2body_.block(0, 0, 3, 3) = cam2bodyrot.matrix();
    // cam2body_.block(0, 3, 3, 1) = cam2bodyrot.matrix();

    cam2body_(3, 3) = 1.0;

    map_upbd_.x() = ceil((map_upbd_.x() - origin_.x())/resolution_) * resolution_;
    map_upbd_.y() = ceil((map_upbd_.y() - origin_.y())/resolution_) * resolution_;
    map_upbd_.z() = ceil((map_upbd_.z() - origin_.z())/resolution_) * resolution_;

    double dx = origin_.x() - (floor((origin_.x())/resolution_)) * resolution_;
    double dy = origin_.y() - (floor((origin_.y())/resolution_)) * resolution_;
    double dz = origin_.z() - (floor((origin_.z())/resolution_)) * resolution_;

    origin_.x() -= dx;
    origin_.y() -= dy;
    origin_.z() -= dz;

    map_upbd_.x() += resolution_;
    map_upbd_.y() += resolution_;
    map_upbd_.z() += resolution_;
    voxel_num_.x() = ceil((map_upbd_.x())/resolution_);
    voxel_num_.y() = ceil((map_upbd_.y())/resolution_);
    voxel_num_.z() = ceil((map_upbd_.z())/resolution_);
    map_upbd_ = origin_ + map_upbd_ - Vector3d(1e-4, 1e-4, 1e-4);
    map_lowbd_ = origin_ + Vector3d(1e-4, 1e-4, 1e-4);

    block_num_.x() = ceil(double(voxel_num_.x()) / block_size_.x());
    block_num_.y() = ceil(double(voxel_num_.y()) / block_size_.y());
    block_num_.z() = ceil(double(voxel_num_.z()) / block_size_.z());
    blockscale_.x() = resolution_*block_size_.x();
    blockscale_.y() = resolution_*block_size_.y();
    blockscale_.z() = resolution_*block_size_.z();

    edgeblock_size_.x() = voxel_num_.x() - floor(voxel_num_.x() / double(block_size_.x()))*block_size_.x();
    edgeblock_size_.y() = voxel_num_.y() - floor(voxel_num_.y() / double(block_size_.y()))*block_size_.y();
    edgeblock_size_.z() = voxel_num_.z() - floor(voxel_num_.z() / double(block_size_.z()))*block_size_.z();

    if(edgeblock_size_.x() == 0) edgeblock_size_.x() = block_size_.x();
    if(edgeblock_size_.y() == 0) edgeblock_size_.y() = block_size_.y();
    if(edgeblock_size_.z() == 0) edgeblock_size_.z() = block_size_.z();
    

    edgeblock_scale_.x() = resolution_*edgeblock_size_.x();
    edgeblock_scale_.y() = resolution_*edgeblock_size_.y();
    edgeblock_scale_.z() = resolution_*edgeblock_size_.z();
    GBS_.resize(block_num_.x()*block_num_.y()*block_num_.z());
    for(int x = 0; x < block_num_.x(); x++){
        for(int y = 0; y < block_num_.y(); y++){
            for(int z = 0; z < block_num_.z(); z++){
                int idx = x + y * block_num_.x() + z * block_num_.x() * block_num_.y();
                GBS_[idx] = make_shared<Grid_Block>();
                GBS_[idx]->origin_.x() = block_size_.x() * x;
                GBS_[idx]->origin_.y() = block_size_.y() * y;
                GBS_[idx]->origin_.z() = block_size_.z() * z;
                GBS_[idx]->show_ = false;
                if(x == block_num_.x() - 1) GBS_[idx]->block_size_.x() = edgeblock_size_.x();
                else GBS_[idx]->block_size_.x() = block_size_.x();
                if(y == block_num_.y() - 1) GBS_[idx]->block_size_.y() = edgeblock_size_.y();
                else GBS_[idx]->block_size_.y() = block_size_.y();
                if(z == block_num_.z() - 1) GBS_[idx]->block_size_.z() = edgeblock_size_.z();
                else GBS_[idx]->block_size_.z() = block_size_.z();
            }
        }
    }

    cout<<"edgeblock_size_:"<<edgeblock_size_.transpose()<<endl;
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"block_num_:"<<block_num_.transpose()<<endl;


    std_msgs::ColorRGBA color;
    colorhsize_ = (map_upbd_(2) - origin_(2)) / (CG.size() - 1);
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        color_list_.push_back(color);
    }

    // have_odom_ = false;
    have_cam_param_ = false;
    last_odom_ = ros::WallTime::now().toSec() + 100000.0;
    last_update_ = ros::WallTime::now().toSec() - 10.0;
    cout<<"last_update_:"<<last_update_<<endl;
    update_interval_ = 1 / update_interval_;
    cout<<"update_interval_:"<<update_interval_<<endl;

    thr_max_ = log(thr_max_ / (1 - thr_max_));
    thr_min_ = log(thr_min_ / (1 - thr_min_));
    pro_hit_ = log(pro_hit_ / (1 - pro_miss_));
    pro_miss_ = log((1 - pro_miss_) / pro_miss_);
    cout<<"thr_max_:"<<thr_max_<<endl;
    cout<<"thr_min_:"<<thr_min_<<endl;
    cout<<"pro_hit_:"<<pro_hit_<<endl;
    cout<<"pro_miss_:"<<pro_miss_<<endl;

    vox_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ns + "/block_map/voxvis", 10);
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>(ns + "/block_map/debug", 10);

    if(stat_) {
        statistic_pub_ = nh.advertise<std_msgs::Float32>(ns + "/block_map/stat_v", 1);
        statistic_timer_ = nh.createTimer(ros::Duration(0.5), &BlockMap::StatisticV, this);
    }
    if(depth_){
        camparam_sub_ = nh_.subscribe("/block_map/caminfo", 10, &BlockMap::CamParamCallback, this);
    }
    if(show_block_)
        show_timer_ = nh_.createTimer(ros::Duration(1.0 / show_freq_), &BlockMap::ShowMapCallback, this);
    if(SDM_->is_ground_) swarm_timer_ = nh_.createTimer(ros::Duration(0.2), &BlockMap::SwarmMapCallback, this);
    else swarm_timer_ = nh_.createTimer(ros::Duration(1.0), &BlockMap::SwarmMapCallback, this);
}

void BlockMap::InitSwarmBlock(vector<uint16_t> &id_l, vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &bound){
    SBS_.resize(id_l.size());
    for(int i = 0; i < id_l.size(); i++){
        SBS_[i].id_ = id_l[i];
        SBS_[i].down_ = bound[i].second + Eigen::Vector3d::Ones() * 1e-3;
        SBS_[i].up_ = bound[i].first - Eigen::Vector3d::Ones() * 1e-3;
        SBS_[i].sub_num_ = 8;
        SBS_[i].exploration_rate_.resize(8, 0);
        SBS_[i].last_pub_rate_.resize(8, 0);
        SBS_[i].to_pub_.resize(8, false);
    }
}

void BlockMap::OdomCallback(const nav_msgs::OdometryConstPtr &odom){

    // geometry_msgs::Point pt;
    // visualization_msgs::Marker debug1, debug2, debug3;
    // debug1.action = visualization_msgs::Marker::ADD;
    // debug1.pose.orientation.w = 1.0;
    // debug1.type = visualization_msgs::Marker::CUBE_LIST;
    // debug1.scale.x = resolution_;
    // debug1.scale.y = resolution_;
    // debug1.scale.z = resolution_;
    // debug1.color.a = 0.3;
    // debug1.color.r = 1.0;
    // debug1.header.frame_id = "world";
    // debug1.header.stamp = ros::Time::now();
    // debug1.id = 0;

    Eigen::Matrix4d body2world = Matrix4d::Identity();
    Eigen::Quaterniond rot;
    rot.x() = odom->pose.pose.orientation.x;
    rot.y() = odom->pose.pose.orientation.y;
    rot.z() = odom->pose.pose.orientation.z;
    rot.w() = odom->pose.pose.orientation.w;

    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;

    body2world.block(0, 0, 3, 3) = rot.matrix();

    cam2world_ = body2world * cam2body_;
    AwakeBlocks(cam2world_.block(0, 3, 3, 1), max_range_);
    bline_ = false;
    for(double x = 0; x < 0.51; x += resolution_){
        for(double y = -0.3; y < 0.31; y += resolution_){
            for(double z = -0.3; z < 0.31; z += resolution_){
                Eigen::Vector3d pos = body2world.block(0, 0, 3, 3) * Vector3d(x, y, z) + body2world.block(0, 3, 3, 1);
                // pt.x = pos.x();
                // pt.y = pos.y();
                // pt.z = pos.z();
                // debug1.points.push_back(pt);
                if(GetVoxState(pos) == occupied) {
                    bline_ = true;
                    // ROS_ERROR("hl");
                }
            }
        }
    }

    // debug_pub_.publish(debug1);
    // have_odom_ = true;
    last_odom_ = ros::Time::now().toSec();
}

void BlockMap::InsertPCLCallback(const sensor_msgs::PointCloud2ConstPtr &pcl){
    if(!bline_ && have_cam_param_ && ros::Time::now().toSec() - last_odom_ < 0.02 && ros::WallTime::now().toSec() - last_update_ > update_interval_){
        last_update_ = ros::WallTime::now().toSec();

        InsertPcl(pcl);
        // have_odom_ = false;
    }
}

void BlockMap::InsertDepthCallback(const sensor_msgs::ImageConstPtr &img){
    if(!bline_ && have_cam_param_ && ros::Time::now().toSec() - last_odom_ < 0.02 && ros::WallTime::now().toSec() - last_update_ > update_interval_){
        last_update_ = ros::WallTime::now().toSec();

        InsertImg(img);
        // cout<<"cost:"<<ros::WallTime::now().toSec() - last_update_<<"s"<<endl;
        // have_odom_ = false;
    }
}

void BlockMap::CamParamCallback(const sensor_msgs::CameraInfoConstPtr &param){
    ROS_WARN("get param!");

    fx_ = param->K[0];
    cx_ = param->K[2];
    fy_ = param->K[4];
    cy_ = param->K[5];

    have_cam_param_ = true;
    u_max_ = param->width;
    v_max_ = param->height;

    downsample_size_ = (int)floor(resolution_ * fx_ / max_range_);
    downsample_size_ = min((int)floor(resolution_ * fy_ / max_range_), downsample_size_);
    

    u_down_max_ = (int)floor((u_max_) / downsample_size_);
    v_down_max_ = (int)floor((v_max_) / downsample_size_);
    u_max_ = u_down_max_ * downsample_size_;
    v_max_ = v_down_max_ * downsample_size_;
    // depth_step_ = min(depth_step_, downsample_size_);
    cout<<"fx_:"<<fx_<<endl;
    cout<<"fy_:"<<fy_<<endl;
    cout<<"cx_:"<<cx_<<endl;
    cout<<"cy_:"<<cy_<<endl;
    cout<<"depth_step_:"<<depth_step_<<endl;
    cout<<"u_down_max_:"<<u_down_max_<<endl;
    cout<<"v_down_max_:"<<v_down_max_<<endl;
    cout<<"downsample_size_:"<<downsample_size_<<endl;
    downsampled_img_.resize(u_down_max_ * v_down_max_);
    for(int u = 0; u < u_down_max_; u++){
        for(int v = 0; v < v_down_max_; v++){
            Eigen::Vector3d pt(((u + 0.5) * downsample_size_ - cx_) * max_range_ / fx_,((v + 0.5) * downsample_size_ - cy_) * max_range_ / fy_, max_range_);
            pt = pt.normalized() * max_range_;
            downsampled_img_[u + v*u_down_max_].max_depth_ = pt.z();
            downsampled_img_[u + v*u_down_max_].close_depth_ = pt.z();

        }
    }
    camparam_sub_.shutdown();
}


void BlockMap::ShowMapCallback(const ros::TimerEvent &e){
    visualization_msgs::MarkerArray mks;
    visualization_msgs::Marker mk_stand;
    mk_stand.action = visualization_msgs::Marker::ADD;
    mk_stand.pose.orientation.w = 1.0;
    mk_stand.type = visualization_msgs::Marker::POINTS;
    mk_stand.scale.x = resolution_;
    mk_stand.scale.y = resolution_;
    mk_stand.scale.z = resolution_;
    mk_stand.color.a = 1.0;
    mk_stand.header.frame_id = "world";
    mk_stand.header.stamp = ros::Time::now();
    // mk_stand.id = *block_it;


    int i = 0;
    geometry_msgs::Point pt;
    // ROS_ERROR("show size:%d", (int)changed_blocks_.size());
    for(vector<int>::iterator block_it = changed_blocks_.begin(); block_it != changed_blocks_.end(); block_it++, i++){
        mks.markers.push_back(mk_stand);
        mks.markers.back().id = *block_it;

        GBS_[*block_it]->show_ = 0;
        Eigen::Vector3d block_end = GBS_[*block_it]->origin_.cast<double>() * resolution_ + resolution_ * GBS_[*block_it]->block_size_.cast<double>()
           + origin_;
        
        double x, y, z;
        int idx = 0;
        int debug_bk, debug_id;
        if(GBS_[*block_it]->state_ == MIXED){//debug
        for( z = resolution_ * (GBS_[*block_it]->origin_(2) + 0.5) + origin_(2); z < block_end(2); z += resolution_){
            for( y = resolution_ * (GBS_[*block_it]->origin_(1)  + 0.5) + origin_(1); y < block_end(1); y += resolution_){
                for( x = resolution_ * (GBS_[*block_it]->origin_(0) + 0.5) + origin_(0); x < block_end(0); x += resolution_){
                    if(GBS_[*block_it]->state_ == OCCUPIED || GBS_[*block_it]->odds_log_[idx] > 0){
                        pt.x = x;
                        pt.y = y; 
                        pt.z = z;
                        mks.markers.back().points.push_back(pt);
                        mks.markers.back().colors.push_back(Getcolor(z));
                    }
                    GetVox(debug_bk, debug_id, Eigen::Vector3d(x, y, z));
                    if(debug_bk != *block_it || debug_id != idx) {
                        cout<<debug_bk<<";"<<debug_id<<"  "<<Eigen::Vector3d(x, y, z).transpose()<<"origin:"<<GBS_[*block_it]->origin_.transpose()
                            <<"  "<<block_end.transpose()<<" .."<<GBS_[*block_it]->block_size_.transpose()<<endl;
                        cout<<*block_it<<" "<<idx<<endl;
                    }
                    idx++;
                }
            }
        }
        }
        if(mks.markers.back().points.size() == 0){
            mks.markers.back().action = visualization_msgs::Marker::DELETE;
        } 
    }
    if(mks.markers.size() > 0) vox_pub_.publish(mks);
    changed_blocks_.clear();
}

void BlockMap::SwarmMapCallback(const ros::TimerEvent &e){
    if(SDM_->is_ground_){
        while (!SDM_->swarm_sub_map_.empty()){
            InsertSwarmPts(SDM_->swarm_sub_map_.front());
            SDM_->swarm_sub_map_.pop_front();
        }
        
        if(/*SDM_->req_flag_  || */double(SDM_->finish_num_) / SDM_->drone_num_ > SDM_->finish_thresh_ && !finish_flag_ && ros::WallTime::now(). toSec() - start_t_ > min_finish_t_){
            finish_flag_ = true;
            // SDM_->req_flag_ = false;

            exp_comm_msgs::MapReqC mq;
            Eigen::Vector3d up, down;
            list<Eigen::Vector3d> pts;
            for(int f_id = 0; f_id < SBS_.size(); f_id++){
                for(int i = 0; i < 8; i++){
                    GetSBSBound(f_id, i, up, down);//debug

                    if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_){
                        mq.f_id.emplace_back(f_id);
                        mq.block_id.emplace_back(i);
                        pts.push_back((up + down) / 2);
                    }
                }
            }
            for(int i = 0; i < 20; i++){
                cout<<"ground finish!!!===="<<endl;
            }
            Debug(pts);
            mq.flag = 1;
            // mq.flag = 0;
            SDM_->SetMapReq(mq);
        }

        if(SDM_->req_flag_ && !finish_flag_){
            SDM_->req_flag_ = false;
            exp_comm_msgs::MapReqC mq;
            Eigen::Vector3d up, down;
            list<Eigen::Vector3d> pts;
            for(int f_id = 0; f_id < SBS_.size(); f_id++){
                for(int i = 0; i < 8; i++){
                    GetSBSBound(f_id, i, up, down);//debug
                    if(SBS_[f_id].exploration_rate_[i] < swarm_pub_thresh_ && SBS_[f_id].exploration_rate_[i] > 5e-3){
                        mq.f_id.emplace_back(f_id);
                        mq.block_id.emplace_back(i);
                        pts.push_back((up + down) / 2);
                    }
                }
            }

            Debug(pts);
            mq.flag = 0; 
            SDM_->SetMapReq(mq);
        }
        if(SDM_->statistic_ && stat_){
            SDM_->CS_.SetVolume(stat_n_ * pow(resolution_, 3), 0);
        }
    }
    else{
        double cur_t = ros::WallTime::now().toSec();
        if(SDM_->finish_list_[SDM_->self_id_ - 1] && !finish_flag_ && SDM_->req_flag_){
            finish_flag_ = true;
            list<Eigen::Vector3d> pts;
            Eigen::Vector3d up, down;
            for(int i = 0; i < SDM_->mreq_.block_id.size(); i++){
                GetSBSBound(SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i], up, down);//debug
                pts.push_back((up + down) / 2);

                if(SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]]) continue;
                SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]] = true;
                swarm_pub_block_.push_back({{SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i]}, ros::WallTime::now().toSec() - (swarm_send_delay_ + 0.5)});
            }
            for(int i = 0; i < 10; i++) ROS_ERROR("finishhhhhh");
            Debug(pts, 5);
            SDM_->mreq_.block_id.clear();
            SDM_->mreq_.f_id.clear();
        }

        if(SDM_->req_flag_ && !SDM_->finish_list_[SDM_->self_id_ - 1] && !finish_flag_){
            SDM_->req_flag_ = false;
            for(int i = 0; i < SDM_->mreq_.block_id.size(); i++){

                if(SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]]) continue;
                SBS_[SDM_->mreq_.f_id[i]].to_pub_[SDM_->mreq_.block_id[i]] = true;
                swarm_pub_block_.push_back({{SDM_->mreq_.f_id[i], SDM_->mreq_.block_id[i]}, ros::WallTime::now().toSec()});
            }
        }

            // list<swarm_exp_msgs::DtgHFEdge>::iterator erase_it = hfe_it;
            // hfe_it--;
            // SDM_->swarm_sub_hfe_.erase(erase_it);
        for(auto pub_it = swarm_pub_block_.begin(); pub_it != swarm_pub_block_.end(); pub_it++){
            bool t_o_pub = false;
            exp_comm_msgs::MapC msg;
            if(pub_it->second + swarm_send_delay_ < cur_t) t_o_pub = true; // time out
            Vox2Msg(msg, pub_it->first.first, pub_it->first.second);
            if((!finish_flag_ && SBS_[msg.f_id].last_pub_rate_[msg.block_id] + 0.15 > SBS_[msg.f_id].exploration_rate_[msg.block_id]
                || finish_flag_ && SBS_[msg.f_id].last_pub_rate_[msg.block_id] + 0.01 > SBS_[msg.f_id].exploration_rate_[msg.block_id])
                 && t_o_pub){
                SBS_[msg.f_id].to_pub_[msg.block_id] = false;
                auto erase_it = pub_it;
                pub_it--;
                swarm_pub_block_.erase(erase_it);
                continue;
            }

            if(msg.block_state != 0 && SBS_[msg.f_id].exploration_rate_[msg.block_id] > swarm_pub_thresh_ || t_o_pub){
                SDM_->SetMap(msg);
                SBS_[msg.f_id].last_pub_rate_[msg.block_id] = SBS_[msg.f_id].exploration_rate_[msg.block_id];
                SBS_[msg.f_id].to_pub_[msg.block_id] = false;
                auto erase_it = pub_it;
                pub_it--;
                swarm_pub_block_.erase(erase_it);
            }

        }

        // while(!swarm_pub_block_.empty()){
        //     if(swarm_pub_block_.front().second + swarm_send_delay_ < cur_t){
        //     exp_comm_msgs::MapC msg;
        //     SBS_[swarm_pub_block_.front().first.first].to_pub_[swarm_pub_block_.front().first.second] = false;
        //     // for(uint8_t i = 0; i < 8; i++){
        //     Vox2Msg(msg, swarm_pub_block_.front().first.first, swarm_pub_block_.front().first.second);
        //     if(msg.block_state != 0){
        //         SDM_->SetMap(msg);
        //     }
        //     // }
        //     swarm_pub_block_.pop_front();
        //     }
        //     else break;
        // }
    }
}

void BlockMap::InsertPcl(const sensor_msgs::PointCloud2ConstPtr &pcl){
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;

    Eigen::Vector3i cam3i, end3i;
    Eigen::Vector3d end_point, dir, cam, ray_iter;
    Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;
    RayCaster rc;
    int block_id, vox_id;

    newly_register_idx_.clear();
    cam3i = PostoId3(cam2world_.block(0,3,3,1));
    if(InsideMap(cam3i)){
        LoadSwarmFilter();
        cam = cam2world_.block(0,3,3,1);
        
        std::vector<int> indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pcl, *points);
        pcl::removeNaNFromPointCloud(*points, *points, indices);
        cur_pcl_.clear();

        for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = points->begin(); pcl_it != points->end(); pcl_it++){
            bool occ;
            end_point(0) = pcl_it->x;
            end_point(1) = pcl_it->y;
            end_point(2) = pcl_it->z;
            end_point = cam2world_.block(0, 0, 3, 3) * end_point + cam2world_.block(0, 3, 3, 1);

            dir = end_point - cam;
            end3i = PostoId3(end_point);            
            occ = dir.norm() <= max_range_;
            if(!occ)
                end_point = cam + (end_point - cam).normalized() * max_range_;

            GetRayEndInsideMap(cam, end_point, occ);
            if(!GetVox(block_id, vox_id, end_point)) continue;
            if(occ){
                if(use_swarm_ && swarm_filter_dict_->find(PostoId(end_point)) != swarm_filter_dict_->end()) continue;
                cur_pcl_.emplace_back(end_point);
                if(!(GBS_[block_id]->flags_[vox_id] & 2)){

                    GBS_[block_id]->flags_[vox_id] |= 2;

                    if(GBS_[block_id]->flags_[vox_id] & 1){
                    }
                    else{
                        GBS_[block_id]->flags_[vox_id] |= 1;
                        block_ids.push_back(block_id);
                        vox_ids.push_back(vox_id);
                    }
                }
                else{
                    continue;
                }
            }
            else{
                if((GBS_[block_id]->flags_[vox_id] & 1)){
                    continue;
                }
            }
            rc.setInput((end_point - origin_) / resolution_, (cam - origin_) / resolution_);
            
            while (rc.step(ray_iter))
            {
                ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
                if(use_swarm_ && swarm_filter_dict_->find(PostoId(ray_iter)) != swarm_filter_dict_->end()) continue;
                if(GetVox(block_id, vox_id, ray_iter)){

                    if((GBS_[block_id]->flags_[vox_id] & 1)){
                        continue;
                    }
                    else{
                        GBS_[block_id]->flags_[vox_id] |= 1;
                        block_ids.push_back(block_id);
                        vox_ids.push_back(vox_id);
                    }
                }
            }
        }
        Eigen::Vector3d p_it;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++){
            float odds_origin = GBS_[*block_it]->odds_log_[*vox_it];
            if(GBS_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                p_it = Id2LocalPos(GBS_[*block_it], *vox_it);
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
            }
            if(GBS_[*block_it]->flags_[*vox_it] & 2){
                GBS_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);
            }
            else{
                // end_point = Id2LocalPos(GBS_[*block_it], *vox_it);


                GBS_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
            }
            GBS_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(*block_it);
                GBS_[*block_it]->show_ = true;
            }
        }
    }
}

void BlockMap::InsertImg(const sensor_msgs::ImageConstPtr &depth){
    if(!have_cam_param_) return;
    vector<int> block_ids, vox_ids;
    vector<int>::iterator block_it, vox_it;

    Eigen::Vector3i cam3i;
    RayCaster rc;

    cam3i = PostoId3(cam2world_.block(0,3,3,1));
    newly_register_idx_.clear();

    if(InsideMap(cam3i)){
        LoadSwarmFilter();
        double pix_depth;
        int downsamp_u, downsamp_v, downsamp_nex_u, downsamp_nex_v, downsamp_idx;
        int block_id, vox_id;
        Eigen::Vector3i  end3i;
        Eigen::Vector3d end_point, dir, cam, ray_iter;
        Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
        Eigen::Vector3d half_res = Eigen::Vector3d(0.5, 0.5, 0.5) * resolution_;

        cur_pcl_.clear();
        cam = cam2world_.block(0, 3, 3, 1);

        uint16_t* row_ptr;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        if(depth->encoding != sensor_msgs::image_encodings::TYPE_16UC1){
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 1000.0);
        }
        tr1::unordered_map<int, Eigen::Vector3d> cast_pts;
        int k_l = ceil(max_range_ * 10.0 / resolution_); 
        for(int v = 0; v < v_max_; v += depth_step_){
            row_ptr = cv_ptr->image.ptr<uint16_t>(v);
            for(int u = 0; u < u_max_; u += depth_step_, row_ptr += depth_step_){

                pix_depth = (*row_ptr) / 1000.0;
                if(pix_depth == 0 && bline_) continue;

                if(pix_depth == 0)
                    pix_depth = max_range_ + 0.1;

                downsamp_u = floor(u / downsample_size_);
                downsamp_v = floor(v / downsample_size_);
                downsamp_idx = downsamp_u + downsamp_v * u_down_max_;

                end_point.x() = ((u + 0.5)  - cx_) * pix_depth / fx_;
                end_point.y() =  ((v + 0.5)  - cy_) * pix_depth / fy_;
                end_point.z() = pix_depth;
                end_point = cam2world_.block(0, 0, 3, 3) * end_point + cam;
                int key = floor((end_point(2) - cam(2) - max_range_ * 2)/resolution_)*k_l*k_l + 
                    floor((end_point(1)- cam(1) - max_range_ * 2)/resolution_)*k_l + 
                    floor((end_point(0)- cam(0) - max_range_ * 2)/resolution_);

                if(cast_pts.find(key) == cast_pts.end())
                    cast_pts.insert({key, end_point});

            }
        }

        // for(int u = 0; u < u_down_max_; u++){
        //     for(int v = 0; v < v_down_max_; v++){
        //         pix_depth = downsampled_img_[u + v * u_down_max_].close_depth_;
        for(auto &k_p : cast_pts){
                bool occ;
                end_point = k_p.second;
                occ = (k_p.second - cam).norm() < max_range_;
                if(!occ)
                    end_point = cam + (end_point - cam).normalized() * max_range_;
                else{
                    if(!use_swarm_ || (swarm_filter_dict_->find(PostoId(end_point)) == swarm_filter_dict_->end()))
                        cur_pcl_.emplace_back(end_point);
                }

                end_point = (PostoId3(end_point).cast<double>() + half) * resolution_ + origin_;

                GetRayEndInsideMap(cam, end_point, occ);

                if(!GetVox(block_id, vox_id, end_point)) continue;

                if(occ){
                    if(use_swarm_ && swarm_filter_dict_->find(PostoId(end_point)) != swarm_filter_dict_->end()) continue;
                    if(!(GBS_[block_id]->flags_[vox_id] & 2)){

                        GBS_[block_id]->flags_[vox_id] |= 2;

                        if(GBS_[block_id]->flags_[vox_id] & 1){
                        }
                        else{
                            GBS_[block_id]->flags_[vox_id] |= 1;
                            block_ids.push_back(block_id);
                            vox_ids.push_back(vox_id);
                        }
                    }
                    else{
                        continue;
                    }
                }


                rc.setInput((end_point - origin_) / resolution_, (cam - origin_) / resolution_);
                while (rc.step(ray_iter))
                {
                    ray_iter = (ray_iter) * resolution_ + origin_ + half_res;
                    if(use_swarm_ && swarm_filter_dict_->find(PostoId(ray_iter)) != swarm_filter_dict_->end()) continue;
                    if(GetVox(block_id, vox_id, ray_iter)){

                        if((GBS_[block_id]->flags_[vox_id] & 1)){
                            continue;
                        }
                        else{
                            GBS_[block_id]->flags_[vox_id] |= 1;
                            block_ids.push_back(block_id);
                            vox_ids.push_back(vox_id);
                        }
                    }
                }
        }

        Eigen::Vector3d p_it;
        for(block_it = block_ids.begin(), vox_it = vox_ids.begin(); block_it != block_ids.end(); block_it++, vox_it++){
            float odds_origin = GBS_[*block_it]->odds_log_[*vox_it];
            if(GBS_[*block_it]->odds_log_[*vox_it] < thr_min_ - 1.0){
                p_it = Id2LocalPos(GBS_[*block_it], *vox_it);
                // if(abs(p_it(2) - 6.5) < 0.01){
                //     int x = *vox_it % GBS_[*block_it]->block_size_(0);
                //     int y = ((*vox_it - x)/GBS_[*block_it]->block_size_(0)) % GBS_[*block_it]->block_size_(1);
                //     int z = ((*vox_it - x) - y*GBS_[*block_it]->block_size_(0))/GBS_[*block_it]->block_size_(1)/GBS_[*block_it]->block_size_(0);
                //     cout<<"x:"<<x<<" y:"<<y<<" z:"<<z<<endl;
                //     cout<<"p_it:"<<p_it.transpose()<<"origin:"<<origin_.transpose()<<endl;
                //     ROS_ERROR("error------");
                //     ros::shutdown();
                //     return;
                // }
                if(stat_){
                    if(p_it(0) > stat_lowbd_(0) && p_it(1) > stat_lowbd_(1) && p_it(2) > stat_lowbd_(2) &&
                    p_it(0) < stat_upbd_(0) && p_it(1) < stat_upbd_(1) && p_it(2) < stat_upbd_(2))
                    stat_n_++;
                }
                newly_register_idx_.push_back(p_it);
                odds_origin = 0.0;
            }
            if(GBS_[*block_it]->flags_[*vox_it] & 2){
                GBS_[*block_it]->odds_log_[*vox_it] = min(odds_origin + pro_hit_, thr_max_);
            }
            else{
                // end_point = Id2LocalPos(GBS_[*block_it], *vox_it);


                GBS_[*block_it]->odds_log_[*vox_it] = max(odds_origin + pro_miss_, thr_min_);
            }
            GBS_[*block_it]->flags_[*vox_it] = 0;
            if(!GBS_[*block_it]->show_ && show_block_){
                changed_blocks_.push_back(*block_it);
                GBS_[*block_it]->show_ = true;
            }
        }
    }
    // Debug();
}

void BlockMap::ProjectToImg(const sensor_msgs::PointCloud2ConstPtr &pcl, vector<double> &depth_img){
    Eigen::Vector2i uv;
    std::vector<int> indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*pcl, *points);
    pcl::removeNaNFromPointCloud(*points, *points, indices);

    depth_img = void_img_;
    
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator pcl_it = points->begin(); pcl_it != points->end(); pcl_it++){
        SpointToUV(pcl_it->x / pcl_it->z, pcl_it->y / pcl_it->z, uv);
        if(uv(0) >= 0 && uv(0) < u_max_ && uv(1) >= 0 && uv(1) < v_max_){
            int idx = uv(0) + uv(1) * u_max_;
            depth_img[idx] = min(depth_img[idx], double(pcl_it->z));
        }
    }
}

void BlockMap::AwakeBlocks(const Eigen::Vector3d &center, const double &range){
    Eigen::Vector3d upbd, lowbd;
    Eigen::Vector3i upbd_id3, lowbd_id3; 
    int idx;
    upbd.x() = min(range + center(0) + resolution_, map_upbd_.x());
    upbd.y() = min(range + center(1) + resolution_, map_upbd_.y());
    upbd.z() = min(range + center(2) + resolution_, map_upbd_.z());

    lowbd.x() = max(center(0) - range - resolution_, map_lowbd_.x());
    lowbd.y() = max(center(1) - range - resolution_, map_lowbd_.y());
    lowbd.z() = max(center(2) - range - resolution_, map_lowbd_.z());

    upbd_id3.x() = floor((upbd.x() - origin_(0)) / blockscale_.x());
    upbd_id3.y() = floor((upbd.y() - origin_(1)) / blockscale_.y());
    upbd_id3.z() = floor((upbd.z() - origin_(2)) / blockscale_.z());
    lowbd_id3.x() = floor((lowbd.x() - origin_(0)) / blockscale_.x());
    lowbd_id3.y() = floor((lowbd.y() - origin_(1)) / blockscale_.y());
    lowbd_id3.z() = floor((lowbd.z() - origin_(2)) / blockscale_.z());

    for(int x = lowbd_id3.x(); x <= upbd_id3.x(); x++){
        for(int y = lowbd_id3.y(); y <= upbd_id3.y(); y++){
            for(int z = lowbd_id3.z(); z <= upbd_id3.z(); z++){
                idx = x + y * block_num_(0) + z * block_num_(0) * block_num_(1);
                GBS_[idx]->Awake((float)thr_max_, (float)thr_min_);
            }
        }
    }
}

void BlockMap::InsertSwarmPts(exp_comm_msgs::MapC &msg){
    // debug_l_<<"f:"<<int(msg.f_id)<<"  b:"<<int(msg.block_id)<<"  "<<int(msg.block_state)<<endl;
    if(msg.f_id < 0 || msg.f_id >= SBS_.size()) return;
    if(msg.block_id < 0 || msg.block_id >= 8) return;
    if(msg.block_state == 0) return;
    Eigen::Vector3d up, down;
    if(!GetSBSBound(msg.f_id, msg.block_id, up, down)){
        ROS_ERROR("error InsertSwarmPts GetSBSBound");
        return;
    }

    Eigen::Vector3d center = (up + down) / 2;
    double range = 0;
    for(int dim = 0; dim < 3; dim++) range = max(range, up(dim) - down(dim) + 0.5);
    AwakeBlocks(center, range);

    vector<Eigen::Vector3d> pts;
    vector<uint8_t> states;

    Flags2Vox(msg.flags, up, down, msg.block_state, pts, states);
    // if(pts.size() > 0)cout<<"success!"<<endl;
    InseartVox(pts, states);
    UpdateSBS(msg.f_id, msg.block_id);
    // debug_l_<<"f:"<<int(msg.f_id)<<"  b:"<<int(msg.block_id)<<"  rate:"<<SBS_[msg.f_id].exploration_rate_[msg.block_id]<<"  "<<finish_flag_<<endl;
}

void BlockMap::SendSwarmBlockMap(const int &f_id, const bool &send_now){
    if(f_id < 0 || f_id >= SBS_.size()) return;
    if(send_now){
        exp_comm_msgs::MapC msg;
        for(uint8_t i = 0; i < 8; i++){
            SBS_[f_id].to_pub_[i] = false;
            Vox2Msg(msg, f_id, i);
            if(msg.block_state != 0){
                SDM_->SetMap(msg);
            }
        }
    }
    else{
        for(uint8_t i = 0; i < 8; i++){
            if(!SBS_[f_id].to_pub_[i]){
                swarm_pub_block_.push_back({{f_id, i}, ros::WallTime::now().toSec()});
                SBS_[f_id].to_pub_[i] = true;
            }
        }
    }
}

bool BlockMap::PosBBXOccupied(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
    Eigen::Vector3d lowbd, upbd, v_it;
    VoxelState state;
    lowbd = pos - bbx / 2;
    upbd = pos + bbx / 2 + Eigen::Vector3d::Ones() * (resolution_ - 1e-3);
    for(v_it(0) = lowbd(0); v_it(0) < upbd(0); v_it(0) += resolution_){
        for(v_it(1) = lowbd(1); v_it(1) < upbd(1); v_it(1) += resolution_){
            for(v_it(2) = lowbd(2); v_it(2) < upbd(2); v_it(2) += resolution_){
                state = GetVoxState(v_it);
                if(state == VoxelState::occupied) return true;
            }
        }
    }
    return false;
}

bool BlockMap::PosBBXFree(const Eigen::Vector3d &pos, const Eigen::Vector3d &bbx){
    Eigen::Vector3d lowbd, upbd, v_it;
    VoxelState state;
    lowbd = pos - bbx / 2;
    upbd = pos + bbx / 2 + Eigen::Vector3d::Ones() * (resolution_ - 1e-3);
    for(v_it(0) = lowbd(0); v_it(0) < upbd(0); v_it(0) += resolution_){
        for(v_it(1) = lowbd(1); v_it(1) < upbd(1); v_it(1) += resolution_){
            for(v_it(2) = lowbd(2); v_it(2) < upbd(2); v_it(2) += resolution_){
                state = GetVoxState(v_it);
                if(state != VoxelState::free) return false;
            }
        }
    }
    return true;
}

void BlockMap::StatisticV(const ros::TimerEvent &e){
    std_msgs::Float32 msg;
    msg.data = stat_n_ * pow(resolution_, 3);
    statistic_pub_.publish(msg);
}

void BlockMap::Debug(list<Eigen::Vector3d> &pts, int ddd){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 2 + ddd;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.pose.position.x = SDM_->self_id_ * 0.1 - 0.5;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    if(SDM_->is_ground_)
        mk.color.r = 1.0;
    else{
        if(ddd == 1){
            mk.color.g = 1.0;
        }
        else if(ddd == 2){
            mk.color.g = 1.0;
            mk.color.r = 0.5;
        }
        else{
            mk.color.b = 1.0;
            mk.pose.position.z = 0.1;
        }
    }
    mk.color.a = 0.5;
    geometry_msgs::Point pt;
    if(!finish_flag_) mk.lifetime = ros::Duration(1.0);
    ROS_WARN("id:%d Debug", SDM_->self_id_);
    for(auto &p : pts){
        // Eigen::Vector3d p = IdtoPos(p_id.first);
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);
        mk.points.emplace_back(pt);
    }
    debug_pub_.publish(mk);
}