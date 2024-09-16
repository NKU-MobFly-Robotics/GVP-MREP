#include <frontier_grid/frontier_grid.h>
void FrontierGrid::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    nh_ = nh;
    nh_private_ = nh_private;
    std::string ns = ros::this_node::getName();
    string sensor;
    bool show_frontier;
    nh_private_.param(ns + "/Exp/minX", 
        origin_.x(), -10.0);
    nh_private_.param(ns + "/Exp/minY", 
        origin_.y(), -10.0);
    nh_private_.param(ns + "/Exp/minZ", 
        origin_.z(), 0.0);
    nh_private_.param(ns + "/Exp/maxX", 
        up_bd_.x(), 10.0);
    nh_private_.param(ns + "/Exp/maxY", 
        up_bd_.y(), 10.0);
    nh_private_.param(ns + "/Exp/maxZ", 
        up_bd_.z(), 5.0);
    nh_private_.param(ns + "/block_map/resolution", 
        resolution_, 0.1);
    nh_private_.param(ns + "/block_map/sensor_max_range", 
        sensor_range_, 5.0);
    nh_private_.param(ns + "/Frontier/sample_max_range", 
        sample_max_range_, 4.5);
    nh_private_.param(ns + "/Frontier/grid_scale", 
        node_scale_, 5.0);
    nh_private_.param(ns + "/Frontier/viewpoint_thresh", 
        vp_thresh_, 2.0);
    nh_private_.param(ns + "/Frontier/observe_thresh", 
        obs_thresh_, 0.85);
    nh_private_.param(ns + "/Frontier/resample_duration", 
        resample_duration_, 1.0);
    nh_private_.param(ns + "/Frontier/sample_hor_dir_num", 
        samp_h_dir_num_, 10);
    nh_private_.param(ns + "/Frontier/sample_ver_dir_num", 
        samp_v_dir_num_, 3);
    nh_private_.param(ns + "/Frontier/sample_dist_num", 
        samp_dist_num_, 3);
    nh_private_.param(ns + "/Frontier/sensor_type", 
        sensor, string("Depth_Camera"));
    nh_private_.param(ns + "/Frontier/FOV_hor_num", 
        FOV_h_num_, 15);
    nh_private_.param(ns + "/Frontier/FOV_ver_num", 
        FOV_v_num_, 10); 
    nh_private_.param(ns + "/Frontier/cam_hor", 
        cam_hor_, 0.5 * M_PI);
    nh_private_.param(ns + "/Frontier/cam_ver", 
        cam_ver_, 0.5 * M_PI);    
    nh_private_.param(ns + "/Frontier/livox_ver_low", 
        livox_ver_low_, -10/180 * M_PI);
    nh_private_.param(ns + "/Frontier/livox_ver_up", 
        livox_ver_up_, 75/180 * M_PI);    
    nh_private_.param(ns + "/Frontier/ray_samp_dist1", 
        ray_samp_dist1_, 0.2);
    // nh_private_.param(ns + "/Frontier/samp_free_thresh", 
    //     samp_free_thresh_, 25);
    nh_private_.param(ns + "/Frontier/ray_samp_dist2", 
        ray_samp_dist2_, 0.1);    
    nh_private_.param(ns + "/Frontier/show_frontier", 
        show_frontier, true);
    nh_private_.param(ns + "/Frontier/min_vp_num", 
        min_vp_num_, 6);    
    nh_private_.param(ns + "/Exp/robot_sizeX", 
        Robot_size_(0), 0.5);    
    nh_private_.param(ns + "/Exp/robot_sizeY", 
        Robot_size_(1), 0.5);    
    nh_private_.param(ns + "/Exp/robot_sizeZ", 
        Robot_size_(2), 0.4);    

        
    node_scale_ = ceil(node_scale_ / resolution_) * resolution_;
    for(int dim = 0; dim < 3; dim++){
        node_num_(dim) = ceil((up_bd_(dim) - origin_(dim)) / node_scale_);
    }
    scan_count_ = 0;
    cout<<"node_scale_:"<<node_scale_<<endl;
    cout<<"up_bd_:"<<up_bd_.transpose()<<endl;
    cout<<"origin_:"<<origin_.transpose()<<endl;
    cout<<"node_num_:"<<node_num_.transpose()<<endl;
    
    use_swarm_ = (SDM_->drone_num_ > 1 && !SDM_->is_ground_);
    cout<<"use_swarm_:"<<use_swarm_<<endl;
    cout<<"SDM_->drone_num_:"<<int(SDM_->drone_num_)<<endl;
    cout<<"SDM_->is_ground_:"<<SDM_->is_ground_<<endl;

    sample_flag_ = false;

    Eigen::Vector3i v_it;
    samp_dir_num_ = samp_v_dir_num_ * samp_dist_num_;
    samp_num_ = samp_h_dir_num_ * samp_v_dir_num_ * samp_dist_num_;
    samp_h_dir_ = M_PI * 2 / samp_h_dir_num_;
    for(v_it(2) = 0; v_it(2) < node_num_(2); v_it(2)++){
        for(v_it(1) = 0; v_it(1) < node_num_(1); v_it(1)++){
            for(v_it(0) = 0; v_it(0) < node_num_(0); v_it(0)++){
                CoarseFrontier CF;
                Eigen::Vector3i vox_num;
                for(int dim = 0; dim < 3; dim++){
                    vox_num(dim) = floor(node_scale_ / resolution_);
                    double remain_d = up_bd_(dim) - origin_(dim) - v_it(dim) * node_scale_;
                    CF.down_(dim) = v_it(dim) * node_scale_ + origin_(dim);
                    CF.up_(dim) = v_it(dim) * node_scale_ + node_scale_ + origin_(dim);
                    CF.center_(dim) = CF.down_(dim) / 2 + CF.up_(dim) / 2;
                    if(remain_d < node_scale_){
                        CF.center_(dim) = remain_d * 0.5 + v_it(dim) * node_scale_ + origin_(dim);
                        CF.up_(dim) = remain_d + v_it(dim) * node_scale_ + origin_(dim);
                        vox_num(dim) = floor((CF.up_(dim) - CF.down_(dim)) / resolution_);
                    }
                    CF.last_sample_ = ros::WallTime::now().toSec();
                    CF.last_strong_check_ = ros::WallTime::now().toSec();
                }
                CF.unknown_num_ = vox_num(0) * vox_num(1) * vox_num(2);
                CF.thresh_num_ = floor((1.0 - obs_thresh_) * CF.unknown_num_);
                CF.f_state_ = 0;
                
                // CF.dirs_state_.resize(samp_h_dir_num_, 0);
                // CF.dirs_free_num_.resize(samp_h_dir_num_, 0);
                CF.local_vps_.resize(samp_num_, 0);
                // CF.public_vps_.resize(samp_num_, 0);
                CF.flags_.reset();
                CF.owner_ = 0;
                f_grid_.emplace_back(CF);                
            }
        }
    }
    SDM_->SetFrontierVpNum(samp_num_);

    if(!SDM_->is_ground_){
        sample_timer_ = nh.createTimer(ros::Duration(0.33), &FrontierGrid::SampleVpsCallback, this);
        lazy_samp_timer_ = nh.createTimer(ros::Duration(0.2), &FrontierGrid::LazySampleCallback, this);
    }
    if(show_frontier)
        show_timer_ = nh.createTimer(ros::Duration(0.5), &FrontierGrid::ShowVpsCallback, this);
    show_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns+"/Frontier/grid", 1);
    debug_pub_ = nh.advertise<visualization_msgs::Marker>("/Frontier/debug", 1);
    // //FOV down sample
    if(sensor == "Depth_Camera"){
        sensor_type_ = CAMERA;
        ROS_WARN("use camera");
    }
    else if(sensor == "Livox"){
        sensor_type_ = LIVOX;
        ROS_WARN("use Livox");
    }
    else{
        ROS_ERROR("error sensor type!");
        ros::shutdown();
        return;
    }

    for(int i = 0; i < samp_dist_num_; i++){
        // sample_dists_.push_back((sample_max_range_ - sqrt(3) * node_scale_) / max(1, samp_dist_num_) * i + sqrt(3)/2 * node_scale_ - 1e-3);
        sample_dists_.push_back((sample_max_range_ - sqrt(3)/2 * node_scale_) / max(1, samp_dist_num_-1) * i + sqrt(3)/2 * node_scale_ - 1e-3);
        cout<<i<<"samp d:"<<sample_dists_.back()<<endl;
    }
    for(int i = 0; i < samp_v_dir_num_; i++){
        if(sensor_type_ == CAMERA){
            sample_vdir_sins_.push_back(sin(M_PI * 0.25 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
            sample_vdir_coses_.push_back(cos(M_PI * 0.25 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
            sample_v_dirs_.push_back(M_PI * 0.25 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2);
        }
        else if(sensor_type_ == LIVOX){
            sample_vdir_sins_.push_back(sin(M_PI * 0.12 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
            sample_vdir_coses_.push_back(cos(M_PI * 0.12 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.22 / 2));
            sample_v_dirs_.push_back(M_PI * 0.12 / max(samp_v_dir_num_ - 1, 1) * i - M_PI * 0.25 / 2);
        }
    }
    for(int i = 0; i < samp_h_dir_num_; i++){
        sample_hdir_sins_.push_back(sin(M_PI * 2.0 / max(samp_h_dir_num_, 1) * i - M_PI));
        sample_hdir_coses_.push_back(cos(M_PI * 2.0 / max(samp_h_dir_num_, 1) * i - M_PI));
        sample_h_dirs_.push_back(M_PI * 2.0 / max(samp_h_dir_num_, 1) * i - M_PI);
    }
    gain_rays_.resize(samp_num_);
    gain_dirs_.resize(samp_num_);
    Robot_pos_.setZero();
    InitGainRays();
    cout<<"samp_h_dir_num_:"<<samp_h_dir_num_<<endl;
    cout<<"samp_v_dir_num_:"<<samp_v_dir_num_<<endl;
    cout<<"samp_dist_num_:"<<samp_dist_num_<<endl;
    cout<<"samp_num_:"<<samp_num_<<endl;
    cout<<"sensor_type_:"<<sensor_type_<<endl;

}

bool FrontierGrid::SampleVps(list<Eigen::Vector3i> &posis){
    list<int> idxs;
    for(auto &p : posis){
        int idx = Posi2Idx(p);
        if(idx != -1) idxs.push_back(idx);
    }
    return SampleVps(idxs);
}

bool FrontierGrid::SampleVps(list<Eigen::Vector3d> &poses){
    list<int> idxs;
    for(auto &p : poses){
        int idx = Pos2Idx(p);
        if(idx != -1) idxs.push_back(idx);
    }
    return SampleVps(idxs);
}

bool FrontierGrid::SampleVps(list<int> &idxs){
    double gain;
    Eigen::Vector4d vp_pose;
    Eigen::Vector3d vp_pos;
    int vp_id;
    bool flag = false;
    double cur_t = ros::WallTime::now().toSec();
    for(auto &idx : idxs){
        if(idx < 0 || idx  >= f_grid_.size() || f_grid_[idx].f_state_ == 2) continue;
        f_grid_[idx].flags_.reset(1);
        if(cur_t - f_grid_[idx].last_sample_ < resample_duration_) continue;
        f_grid_[idx].last_sample_ = cur_t;

        list<uint8_t> dieing_vps; // for swarm
        
        for(int h_id = 0; h_id < samp_h_dir_num_; h_id++){
            for(int dir_id = 0; dir_id < samp_dir_num_; dir_id++){
                for(int l_id = 0; l_id < samp_dist_num_; l_id++){
                    vp_id = (h_id * samp_dir_num_ + dir_id) * samp_dist_num_ + l_id;
                    if(!GetVp(idx, vp_id, vp_pose) ||  f_grid_[idx].local_vps_[vp_id] == 2) continue;
                    vp_pos = vp_pose.block(0,0,3,1);
                    if(!LRM_->IsFeasible(vp_pos) || !LRM_->InsideMap(vp_pos) || LRM_->StrangePoint(vp_pos)){
                        if(BM_->PosBBXOccupied(vp_pos, Robot_size_) || LRM_->StrangePoint(vp_pos) ){
                            dieing_vps.push_back(vp_id);
                            f_grid_[idx].local_vps_[vp_id] = 2;
                        }
                        continue;
                    }
                    if(GetGain(idx, vp_id) < vp_thresh_) {
                        dieing_vps.push_back(vp_id);
                        f_grid_[idx].local_vps_[vp_id] = 2;
                        if(!f_grid_[idx].flags_[2]){
                            f_grid_[idx].flags_.set(2);
                            exploring_frontiers_show_.emplace_back(idx);
                        }
                    }
                    else{
                        if(LRM_->IsLocalFeasible(vp_pos)) flag = true;
                        f_grid_[idx].local_vps_[vp_id] = 1;
                        if(!f_grid_[idx].flags_[2]){
                            f_grid_[idx].flags_.set(2);
                            exploring_frontiers_show_.emplace_back(idx);
                        }
                    }
                }
            }
        }
        int alive_num = 0;
        for(int h_id = 0; h_id < samp_num_; h_id++){
            if(f_grid_[idx].local_vps_[h_id] != 2){
                alive_num++;
            }
        }
        if(alive_num < min_vp_num_){
            f_grid_[idx].f_state_ = 2;
            if(!f_grid_[idx].flags_[2]){
                explored_frontiers_show_.push_back(idx);
                f_grid_[idx].flags_.set(2);
            }
            if(use_swarm_&& !SDM_->is_ground_) {
                SDM_->SetDTGFn(idx, f_grid_[idx].local_vps_, 1, false); 
                BM_->SendSwarmBlockMap(idx, false);
            }
        }
        else{
            if(use_swarm_ && !dieing_vps.empty()&& !SDM_->is_ground_) SDM_->SetDTGFn(idx, f_grid_[idx].local_vps_, 0, true); 
        }
    }
    return flag;
}

void FrontierGrid::UpdateFrontier(const vector<Eigen::Vector3d> &pts){
    int idx;
    list<int> idx_list;

    for(auto &p : pts){
        idx = Pos2Idx(p);
        if(idx == -1) continue;
        f_grid_[idx].unknown_num_--;

        
        if(!f_grid_[idx].flags_[0]){
            f_grid_[idx].flags_.set(0);
            idx_list.push_back(idx);
        }
    }

    for(auto &it_idx : idx_list){
        f_grid_[it_idx].flags_.reset(0);
        if(f_grid_[it_idx].f_state_ == 0) {
            // ROS_WARN("self exploring %d", it_idx);
            f_grid_[it_idx].f_state_ = 1;
            f_grid_[it_idx].flags_.set(3);
            if(use_swarm_ && !SDM_->is_ground_){
                list<uint8_t> vp_temp;
                SDM_->SetDTGFn(it_idx, f_grid_[it_idx].local_vps_, 0, true);
                // SDM_->SetDTGFnDeadvps(it_idx, vp_temp);
            }
        }

        if(f_grid_[it_idx].f_state_ == 1){
            if(f_grid_[it_idx].unknown_num_ < f_grid_[it_idx].thresh_num_){
                ExpandFrontier(it_idx, true);
                f_grid_[it_idx].f_state_ = 2;
                if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(it_idx, false);
                if(use_swarm_ && !SDM_->is_ground_) SDM_->SetDTGFn(it_idx, f_grid_[it_idx].local_vps_, 0, false);
                // if(use_swarm_) SDM_->SetDTGFnDead(it_idx);
            }

            //show
            if(!f_grid_[it_idx].flags_[2] && f_grid_[it_idx].f_state_ == 1){
                exploring_frontiers_show_.push_back(it_idx);
                f_grid_[it_idx].flags_.set(2);
            }
            else if(!f_grid_[it_idx].flags_[2] && f_grid_[it_idx].f_state_ == 2){
                explored_frontiers_show_.push_back(it_idx);
                f_grid_[it_idx].flags_.set(2);
            }

            if(!f_grid_[it_idx].flags_[1] && f_grid_[it_idx].f_state_ == 1){
                f_grid_[it_idx].flags_.set(1);
                exploring_frontiers_.push_back(it_idx);
            }
        }
    }


}

double FrontierGrid::GetGain(const int &f_id, const int &vp_id){
    bool vis_free;
    double gain = 0;
    auto &rays = gain_rays_[vp_id];
    VoxelState state;
    Eigen::Vector3d chk_pt;
    Eigen::Vector4d v_pose;
    // cout<<"gain vp id:"<<vp_id<<endl;
    if(!GetVp(f_id, vp_id, v_pose)){
        ROS_ERROR("GetGain: error vp id%d", int(vp_id));
        ros::shutdown();
        return -1;
    }
    for(auto &ray : rays){
        vis_free = true;
        for(auto &vox : ray.first){
            chk_pt = vox + f_grid_[f_id].center_;
            state = BM_->GetVoxState(chk_pt);
            if(state == VoxelState::occupied || state == VoxelState::out){
                vis_free = false;
                break;
            }
        }
        if(!vis_free) continue;

        for(auto &v_g : ray.second){
            chk_pt = v_g.first + f_grid_[f_id].center_;
            state = BM_->GetVoxState(chk_pt);
            if(state == VoxelState::free){
                continue;
            }
            else if(state == VoxelState::occupied || state == VoxelState::out){
                break;
            }
            else{
                gain += v_g.second;
                break;
            }
        }
    }
    return gain;
}

void FrontierGrid::InitGainRays(){
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    Eigen::Vector4d vp;
    double dtheta = cam_hor_ / FOV_h_num_;
    double dphi = cam_ver_ / FOV_v_num_;
    double cos_phi = cos(dphi);
    double sin_2_dphi = sin(dphi / 2);

    for(int v_id = 0; v_id < samp_num_; v_id++){
        if(!GetVp(f, v_id, vp)){
            ROS_ERROR("InitGainRays0");
            ros::shutdown();
        }
        if(sensor_type_ == SensorType::CAMERA){
            dtheta = cam_hor_ / FOV_h_num_;
            dphi = cam_ver_ / FOV_v_num_;
            cos_phi = cos(dphi);
            sin_2_dphi = sin(dphi / 2);
            
            for(int h = 0; h < FOV_h_num_; h++){
                double h_dir = vp(3) + double(h) / FOV_h_num_ * cam_hor_ - cam_hor_ / 2;
                for(int v = 0; v < FOV_v_num_; v++){
                    bool valid_ray = false;
                    double v_dir = double(v) / double(FOV_v_num_) * cam_ver_ - cam_ver_ / 2;
                    tr1::unordered_map<int, double> l1;
                    list<Eigen::Vector3d> ray1;
                    list<pair<Eigen::Vector3d, double>> ray2;
                    Eigen::Vector3d dir;
                    // cos_phi = cos(v_dir);
                    dir(0) = cos(h_dir) * cos(v_dir);
                    dir(1) = sin(h_dir) * cos(v_dir);
                    dir(2) = sin(v_dir);
                    double l_gain = 0;
                    for(double l = 0; l < sensor_range_; l += ray_samp_dist1_){
                        Eigen::Vector3d p = dir * l;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        int id = BM_->PostoId(p);
                        if(abs(pm.x()) < node_scale_ / 2 && abs(pm.y()) < node_scale_ / 2
                             && abs(pm.z()) < node_scale_ / 2){
                            valid_ray = true;
                            l_gain = l;
                            break;
                        }
                        if(l1.find(id) == l1.end()){
                            l1.insert(pair<int, double>{id, l});
                            ray1.push_back(pm);
                        }
                    }
                    tr1::unordered_map<int, double> l2;
                    for(; l_gain < sensor_range_; l_gain += ray_samp_dist2_){
                        Eigen::Vector3d p = dir * l_gain;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        if(abs(pm.x()) > node_scale_ / 2 || abs(pm.y()) > node_scale_ / 2
                             || abs(pm.z()) > node_scale_ / 2){
                            break;
                        }
                        int id = BM_->PostoId(pm);
                        if(l2.find(id) == l2.end()){
                            l2.insert(pair<int, double>{id, l_gain});
                            double gain = 2*dtheta*pow(l_gain, 2)*sin_2_dphi*cos_phi;
                            ray2.push_back({pm, gain});
                        }
                    }
                    if(valid_ray && ray2.size() > 0){
                        gain_rays_[v_id].push_back({ray1, ray2});
                        gain_dirs_[v_id].push_back({dir * l_gain + vp.block(0, 0, 3, 1), 2*dtheta*sin_2_dphi*cos_phi*pow(l_gain, 2)});
                    }
                }
            }
        }
        else if(sensor_type_ == SensorType::LIVOX){
            dtheta = M_PI * 2 / FOV_h_num_;
            dphi = (livox_ver_up_ - livox_ver_low_) / FOV_v_num_;
            cos_phi = cos(dphi);
            sin_2_dphi = sin(dphi / 2);
            for(int h = 0; h < FOV_h_num_; h++){
                double h_dir = vp(3) + double(h) / FOV_h_num_ * M_PI * 2 - M_PI;
                for(int v = 0; v < FOV_v_num_; v++){
                    bool valid_ray = false;
                    double v_dir = double(v) / FOV_v_num_ * (livox_ver_up_ - livox_ver_low_) + livox_ver_low_;
                    tr1::unordered_map<int, double> l1;
                    list<Eigen::Vector3d> ray1;
                    list<pair<Eigen::Vector3d, double>> ray2;
                    Eigen::Vector3d dir;
                    // cos_phi = cos(v_dir);
                    dir(0) = cos(h_dir) * cos(v_dir);
                    dir(1) = sin(h_dir) * cos(v_dir);
                    dir(2) = sin(v_dir);
                    double l_gain = 0;
                    for(double l = 0; l < sensor_range_; l += ray_samp_dist1_){
                        Eigen::Vector3d p = dir * l;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        int id = BM_->PostoId(p);
                        if(abs(pm.x()) < node_scale_ / 2 && abs(pm.y()) < node_scale_ / 2
                             && abs(pm.z()) < node_scale_ / 2){
                            valid_ray = true;
                            l_gain = l;
                            break;
                        }
                        if(l1.find(id) == l1.end()){
                            l1.insert(pair<int, double>{id, l});
                            ray1.push_back(pm);
                        }
                    }
                    tr1::unordered_map<int, double> l2;
                    for(; l_gain < sensor_range_; l_gain += ray_samp_dist2_){
                        Eigen::Vector3d p = dir * l_gain;
                        Eigen::Vector3d pm = p + vp.block(0, 0, 3, 1);
                        if(abs(pm.x()) > node_scale_ / 2 || abs(pm.y()) > node_scale_ / 2
                             || abs(pm.z()) > node_scale_ / 2){
                            break;
                        }
                        int id = BM_->PostoId(pm);
                        if(l2.find(id) == l2.end()){
                            l2.insert(pair<int, double>{id, l_gain});
                            double gain = 2*dtheta*pow(l_gain, 2)*sin_2_dphi*cos_phi;
                            ray2.push_back({pm, gain});
                        }
                    }
                    if(valid_ray && ray2.size() > 0){
                        gain_rays_[v_id].push_back({ray1, ray2});
                        gain_dirs_[v_id].push_back({dir * l_gain + vp.block(0, 0, 3, 1), 2*dtheta*sin_2_dphi*cos_phi * pow(l_gain, 2)});
                    }
                }
            }
        }
        else{
            ROS_ERROR("InitGainRays1");
            ros::shutdown();
            return;
        }
    }
}

bool FrontierGrid::SampleVps(){
    bool flag = SampleVps(exploring_frontiers_);
    exploring_frontiers_.clear();
    sample_flag_ = true;
    return flag;
}

void FrontierGrid::SampleVpsCallback(const ros::TimerEvent &e){
    // cout<<"sample num:"<<exploring_frontiers_.size()<<endl;
    SampleVps(exploring_frontiers_);
    exploring_frontiers_.clear();
    sample_flag_ = true;
}

void FrontierGrid::ExpandFrontier(const int &idx, const bool &local_exp){
    Eigen::Vector3i n(1, node_num_(0), node_num_(0) * node_num_(1));
    Eigen::Vector3i id3;
    double cur_t = ros::WallTime::now().toSec();
    if(!Idx2Posi(idx, id3)) return;
    for(int dim = 0; dim < 3; dim++){
        if(id3(dim) != 0){
            int n1_idx = idx - n(dim);
            if(n1_idx < f_grid_.size() && n1_idx >= 0 && f_grid_[n1_idx].f_state_ == 0){

                f_grid_[n1_idx].f_state_ = 1;
                if(local_exp) f_grid_[n1_idx].flags_.set(3);
                if(use_swarm_ && !SDM_->is_ground_){ //send awake
                    list<uint8_t> vp_temp;
                    SDM_->SetDTGFn(n1_idx, f_grid_[n1_idx].local_vps_, 0, true);
                    // SDM_->SetDTGFnDeadvps(n1_idx, vp_temp);
                }
                // ROS_WARN("self exploring expand %d", n1_idx);
                f_grid_[n1_idx].last_sample_ = cur_t - 100.0;
                if(f_grid_[n1_idx].unknown_num_ < f_grid_[n1_idx].thresh_num_) {
                    if(f_grid_[n1_idx].f_state_ != 2 && use_swarm_ && !SDM_->is_ground_) SDM_->SetDTGFn(n1_idx, f_grid_[n1_idx].local_vps_, 0, false); //SDM_->SetDTGFnDead(n1_idx);//send dead
                    f_grid_[n1_idx].f_state_ = 2;
                    if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(n1_idx, false);
                }

                if(!f_grid_[n1_idx].flags_[2] && f_grid_[n1_idx].f_state_ == 1){
                    exploring_frontiers_show_.push_back(n1_idx);
                    f_grid_[n1_idx].flags_.set(2);
                }
                else if(!f_grid_[n1_idx].flags_[2] && f_grid_[n1_idx].f_state_ == 2){
                    explored_frontiers_show_.push_back(n1_idx);
                    f_grid_[n1_idx].flags_.set(2);
                }

                if(!f_grid_[n1_idx].flags_[1] && f_grid_[n1_idx].f_state_ == 1){
                    f_grid_[n1_idx].flags_.set(1);
                    exploring_frontiers_.push_back(n1_idx);
                }
            }
        }
        if(id3(dim) != node_num_(dim) - 1){
            int n2_idx = idx + n(dim);
            if(n2_idx < f_grid_.size() && n2_idx >= 0 && f_grid_[n2_idx].f_state_ == 0){
                f_grid_[n2_idx].f_state_ = 1;
                if(local_exp) f_grid_[n2_idx].flags_.set(3);
                if(use_swarm_ && !SDM_->is_ground_){ //send awake
                    list<uint8_t> vp_temp;
                    SDM_->SetDTGFn(n2_idx, f_grid_[n2_idx].local_vps_, 0, true);
                    // SDM_->SetDTGFnDeadvps(n2_idx, vp_temp);
                }
                // ROS_WARN("self exploring expand2 %d", n2_idx);
                f_grid_[n2_idx].last_sample_ = cur_t - 100.0;
                if(f_grid_[n2_idx].unknown_num_ < f_grid_[n2_idx].thresh_num_) {
                    if(f_grid_[n2_idx].f_state_ != 2 && use_swarm_ && !SDM_->is_ground_) SDM_->SetDTGFn(n2_idx, f_grid_[n2_idx].local_vps_, 0, false);//SDM_->SetDTGFnDead(n2_idx);//send dead
                    f_grid_[n2_idx].f_state_ = 2;
                    if(use_swarm_ && !SDM_->is_ground_) BM_->SendSwarmBlockMap(n2_idx, false);
                }

                if(!f_grid_[n2_idx].flags_[2] && f_grid_[n2_idx].f_state_ == 1){
                    exploring_frontiers_show_.push_back(n2_idx);
                    f_grid_[n2_idx].flags_.set(2);
                }
                else if(!f_grid_[n2_idx].flags_[2] && f_grid_[n2_idx].f_state_ == 2){
                    explored_frontiers_show_.push_back(n2_idx);
                    f_grid_[n2_idx].flags_.set(2);
                }

                if(!f_grid_[n2_idx].flags_[1] && f_grid_[n2_idx].f_state_ == 1){
                    f_grid_[n2_idx].flags_.set(1);
                    exploring_frontiers_.push_back(n2_idx);
                }
            }
        }
    }
}

void FrontierGrid::GetWildGridsBBX(const Eigen::Vector3d &center, const Eigen::Vector3d &box_scale, list<pair<int, list<pair<int, Eigen::Vector3d>>>> &f_list){
    Eigen::Vector3d upbd, lowbd;
    Eigen::Vector3i upid, lowid, it;
    int f_id;
    // list<int> debug_list;
    upbd = center + box_scale / 2;
    lowbd = center - box_scale / 2;

    for(int dim = 0; dim < 3; dim++){
        upbd(dim) = min(upbd(dim), up_bd_(dim) - 1e-3);
        upbd(dim) = max(upbd(dim), origin_(dim) + 1e-3);
        lowbd(dim) = min(lowbd(dim), up_bd_(dim) - 1e-3);
        lowbd(dim) = max(lowbd(dim), origin_(dim) + 1e-3);
        upid(dim) = (upbd(dim) - origin_(dim)) / node_scale_;
        lowid(dim) = (lowbd(dim) - origin_(dim)) / node_scale_;
    }
    for(it(0) = lowid(0); it(0) <= upid(0); it(0)++){
        for(it(1) = lowid(1); it(1) <= upid(1); it(1)++){
            for(it(2) = lowid(2); it(2) <= upid(2); it(2)++){
                f_id = Posi2Idx(it);
                if(f_id == -1) continue;
                list<pair<int, Eigen::Vector3d>> vps;
                Eigen::Vector4d vp_pose;
                Eigen::Vector3d vp_pos;
                for(int v_id = 0; v_id < samp_num_; v_id++){
                    if(f_grid_[f_id].local_vps_[v_id] == 1 && GetVp(f_id, v_id, vp_pose)){
                        vp_pos = vp_pose.block(0, 0, 3, 1);
                        vps.push_back({v_id, vp_pos});
                    }
                }
                if(vps.size() > 0){
                    f_list.push_back({f_id, vps});
                    // debug_list.emplace_back(f_id);
                }
            }   
        }   
    }
    // Debug(debug_list);
}

void FrontierGrid::LazySampleCallback(const ros::TimerEvent &e){
    Eigen::Vector3d up, low;
    Eigen::Vector3i up_id3, low_id3, it;
    int f_id;
    double cur_t = ros::WallTime::now().toSec();
    up = Robot_pos_ + Eigen::Vector3d::Ones() * sensor_range_;
    low = Robot_pos_ - Eigen::Vector3d::Ones() * sensor_range_;
    for(int dim = 0; dim < 3; dim++){
        up_id3(dim) = min(node_num_(dim) - 1, int(floor((up(dim) - origin_(dim))/node_scale_)));
        low_id3(dim) = max(0, int(floor((low(dim) - origin_(dim))/node_scale_)));
    }
    for(it(0) = low_id3(0); it(0) <= up_id3(0); it(0)++){
        for(it(1) = low_id3(1); it(1) <= up_id3(1); it(1)++){
            for(it(2) = low_id3(2); it(2) <= up_id3(2); it(2)++){
                f_id = Posi2Idx(it);
                if(f_id == -1) continue;
                if(f_grid_[f_id].f_state_ == 1 && cur_t - f_grid_[f_id].last_sample_ > 2.0 && !f_grid_[f_id].flags_[1]){
                    f_grid_[f_id].flags_.set(1);
                    exploring_frontiers_.push_back(f_id);
                }
            }   
        }   
    }
}

bool FrontierGrid::StrongCheckViewpoint(const int &f_id, const int &v_id, const bool &allow_unknown){
    Eigen::Vector4d vp_pose;
    if(!GetVp(f_id, v_id, vp_pose)) return false;
    auto &frontier = f_grid_[f_id];

    // block check
    Eigen::Vector3d pos = vp_pose.block(0, 0, 3, 1);
    if(allow_unknown && BM_->PosBBXOccupied(pos, Robot_size_)) {
        // cout<<"colli1"<<endl;
        return false;
    }
    else if(!allow_unknown && !BM_->PosBBXFree(pos, Robot_size_)) {
        // cout<<"colli2"<<endl;
        return false;
    }

    //gain check
    Eigen::Vector3d f_scale = (frontier.up_ - frontier.down_) / 2;

    list<Eigen::Vector3d> ray;
    double gain = 0;
    bool inside_f;
    VoxelState state;
    // ROS_WARN("StrongCheckViewpoint0");
    for(auto &d_l : gain_dirs_[v_id]){
        //debug
        // if((pos - (d_l.first + f_grid_[f_id].center_)).norm() > sensor_range_){
        //     ROS_ERROR("error ray");
        //     cout<<pos.transpose()<<endl;
        //     cout<<(d_l.first + f_grid_[f_id].center_).transpose()<<endl;
        //     cout<<(d_l.first).transpose()<<endl;
        //     cout<<(f_grid_[f_id].center_).transpose()<<endl;
        // }
        BM_->GetCastLine(pos, d_l.first + frontier.center_, ray);
        for(auto &p : ray){
            inside_f = true;
            for(int dim = 0; dim < 3; dim++){
                if(abs(frontier.center_(dim) - p(dim)) > f_scale(dim)){
                    inside_f = false;
                    break;
                }
            }
            state = BM_->GetVoxState(p);
            if(!inside_f){
                if(state == VoxelState::occupied || state == VoxelState::out){
                    break;
                }
            }
            else{
                if(state == VoxelState::free){
                    continue;
                }
                else if(state == VoxelState::occupied || state == VoxelState::out){
                    break;
                }
                else{
                    gain += d_l.second * pow((p - pos).norm(), 2);
                    break;
                }
            }
        }
    }
    // ROS_WARN("StrongCheckViewpoint1");
    if(gain < vp_thresh_) {
        return false;
    }
    return true;
}

void FrontierGrid::ShowVpsCallback(const ros::TimerEvent &e){
    // cout<<"SHOW!!!!!!!!!!!!!!!!!!!!!!!"<<endl;//debug
    Eigen::Vector4d vp_pose;
    visualization_msgs::MarkerArray mka;
    visualization_msgs::Marker mk1, mk2;
    mk1.header.frame_id = "world";
    mk1.header.stamp = ros::Time::now();
    mk1.id = 1;
    mk1.action = visualization_msgs::Marker::ADD;
    mk1.type = visualization_msgs::Marker::CUBE;
    mk1.scale.x = node_scale_;
    mk1.scale.y = node_scale_;
    mk1.scale.z = node_scale_;
    mk1.color.a = 0.2;
    mk1.color.b = 0.7;
    mk1.color.g = 0.6;
    mk1.color.r = 0.6;
    mk1.pose.position.x = 0;
    mk1.pose.position.y = 0;
    mk1.pose.position.z = 0;
    mk1.pose.orientation.x = 0;
    mk1.pose.orientation.y = 0;
    mk1.pose.orientation.z = 0;
    mk1.pose.orientation.w = 1;
    mk2 = mk1;
    mk2.type = visualization_msgs::Marker::LINE_LIST;
    mk2.scale.x = 0.02;
    mk2.scale.y = 0.02;
    mk2.scale.z = 0.02;
    std_msgs::ColorRGBA cl;
    // ROS_WARN("ShowVpsCallback0");
    // for(int f = 0; f < f_grid_.size(); f++){
    for(auto &f : exploring_frontiers_show_){
        // ROS_WARN("ShowVpsCallback0.0");
        auto &frontier = f_grid_[f];
        if(frontier.f_state_ == 0)
            continue;
        // ROS_WARN("ShowVpsCallback0.1");
        if(frontier.f_state_ == 2){
            explored_frontiers_show_.push_back(f);
            continue;
        }
        // ROS_WARN("ShowVpsCallback0.2");
        frontier.flags_.reset(2);
        mk1.action = visualization_msgs::Marker::ADD;
        mk2.action = visualization_msgs::Marker::ADD;
        // mk1.points.clear();
        mk2.points.clear();
        mk1.id = f * 2;
        mk2.id = f * 2 + 1;
        // if(f_grid_[f].owner_ != 0 && f_grid_[f].owner_ != 1 && f_grid_[f].owner_ != 2){
        //     for(int j = 0; j < 10; j++)
        //         ROS_ERROR("id:%d owner:%d", SDM_->self_id_, f_grid_[f].owner_);
        // }
        cl = CM_->Id2Color(f_grid_[f].owner_, 0.15);
        mk1.color = cl;
        mk1.pose.position.x = frontier.center_(0);
        mk1.pose.position.y = frontier.center_(1);
        mk1.pose.position.z = frontier.center_(2);
        mk1.scale.x = frontier.up_(0) - frontier.down_(0);
        mk1.scale.y = frontier.up_(1) - frontier.down_(1);
        mk1.scale.z = frontier.up_(2) - frontier.down_(2);
        mk2.color = cl;
        mk2.color.a = 0.3;
        for(int vp_id = 0; vp_id < samp_num_; vp_id++){
            if(frontier.local_vps_[vp_id] == 1 && GetVp(f, vp_id, vp_pose)){
                // LoadVpLines(mk2, vp_pose);
            }
        }
        mka.markers.emplace_back(mk1);
        mka.markers.emplace_back(mk2);
    }
    mk1.points.clear();
    mk2.points.clear();
    mk1.action = visualization_msgs::Marker::ADD;
    mk2.action = visualization_msgs::Marker::DELETE;
    mk1.color.a = 0.01;
    mk1.color.b = 0.7;
    mk1.color.g = 0.7;
    mk1.color.r = 0.7;
    for(auto &f : explored_frontiers_show_){
        auto &frontier = f_grid_[f];
        frontier.flags_.reset(2);
        mk1.id = f * 2;
        mk2.id = f * 2 + 1;
        mk1.pose.position.x = frontier.center_(0);
        mk1.pose.position.y = frontier.center_(1);
        mk1.pose.position.z = frontier.center_(2);
        mk1.scale.x = frontier.up_(0) - frontier.down_(0);
        mk1.scale.y = frontier.up_(1) - frontier.down_(1);
        mk1.scale.z = frontier.up_(2) - frontier.down_(2);
        
        mka.markers.emplace_back(mk1);
        mka.markers.emplace_back(mk2);
    }
    // ROS_WARN("ShowVpsCallback2");
    show_pub_.publish(mka);
    explored_frontiers_show_.clear();
    exploring_frontiers_show_.clear();
}

void FrontierGrid::ShowGainDebug(){
    visualization_msgs::Marker mk, mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 1;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE;
    mk.scale.x = node_scale_;
    mk.scale.y = node_scale_;
    mk.scale.z = node_scale_;
    mk.color.a = 0.2;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    Eigen::Vector3d z_pos = Eigen::Vector3d::Zero();
    Eigen::Vector4d v_pose;
    mk.color.b = 0;
    mkr1 = mk;
    mkr2 = mk;
    mkr1.color.r = 0.7;
    mkr2.color.g = 0.7;
    mkr1.color.a = 0.7;
    mkr2.color.a = 0.7;
    mkr1.scale.x = resolution_;
    mkr1.scale.y = resolution_;
    mkr1.scale.z = resolution_;
    mkr2.scale.x = resolution_;
    mkr2.scale.y = resolution_;
    mkr2.scale.z = resolution_;
    mkr1.type = visualization_msgs::Marker::CUBE_LIST;
    mkr2.type = visualization_msgs::Marker::CUBE_LIST;
    geometry_msgs::Point pt;
    for(int i = 0; i < gain_rays_.size(); i++){

        mkr1.id = i*2 + 2;
        mkr2.id = i*2 + 3;
        mkr1.points.clear();
        mkr2.points.clear();

        mkr1.action = visualization_msgs::Marker::ADD;
        mkr2.action = visualization_msgs::Marker::ADD;
        if(!ros::ok()) return;
        if(!GetVp(z_pos, i, v_pose)){
            ROS_ERROR("error vp id%d", int(i));
            return;
        }
        pt.x = v_pose(0);
        pt.y = v_pose(1);
        pt.z = v_pose(2);
        mkr1.points.push_back(pt);
        for(auto &rays : gain_rays_[i]){
            for(auto & p1 : rays.first){
                pt.x = p1.x();
                pt.y = p1.y();
                pt.z = p1.z();
                mkr1.points.push_back(pt);
            }
            for(auto & p2 : rays.second){
                pt.x = p2.first.x();
                pt.y = p2.first.y();
                pt.z = p2.first.z();
                mkr2.points.push_back(pt);
            }
        }
        ros::Duration(0.05).sleep();
        debug_pub_.publish(mkr2);
        debug_pub_.publish(mkr1);
        ros::Duration(0.3).sleep();
        
        mkr1.action = visualization_msgs::Marker::DELETE;
        mkr2.action = visualization_msgs::Marker::DELETE;
        debug_pub_.publish(mkr2);
        debug_pub_.publish(mkr1);
    }

}

void FrontierGrid::LoadVpLines(visualization_msgs::Marker &mk, Eigen::Vector4d &vp){
    geometry_msgs::Point pt0, pt1, pt2, pt3, pt4;
    pt0.x = vp(0);    
    pt0.y = vp(1);    
    pt0.z = vp(2);
    pt1.x = vp(0) + 0.5 * cos(vp(3) + cam_hor_/2) / cos(-cam_ver_/2);    
    pt1.y = vp(1) + 0.5 * sin(vp(3) + cam_hor_/2) / cos(-cam_ver_/2);
    pt1.z = vp(2) + tan(-cam_ver_/2) * 0.5;
    pt2.x = vp(0) + 0.5 * cos(vp(3) - cam_hor_/2) / cos(-cam_ver_/2);      
    pt2.y = vp(1) + 0.5 * sin(vp(3) - cam_hor_/2) / cos(-cam_ver_/2);    
    pt2.z = vp(2) + tan(-cam_ver_/2) * 0.5;
    pt3.x = vp(0) + 0.5 * cos(vp(3) + cam_hor_/2) / cos(cam_ver_/2);    
    pt3.y = vp(1) + 0.5 * sin(vp(3) + cam_hor_/2) / cos(cam_ver_/2);
    pt3.z = vp(2) + tan(cam_ver_/2) * 0.5;
    pt4.x = vp(0) + 0.5 * cos(vp(3) - cam_hor_/2) / cos(cam_ver_/2);    
    pt4.y = vp(1) + 0.5 * sin(vp(3) - cam_hor_/2) / cos(cam_ver_/2); 
    pt4.z = vp(2) + tan(cam_ver_/2) * 0.5;

    mk.points.push_back(pt0);
    mk.points.push_back(pt1);
    mk.points.push_back(pt0);
    mk.points.push_back(pt2);
    mk.points.push_back(pt0);
    mk.points.push_back(pt3);
    mk.points.push_back(pt0);
    mk.points.push_back(pt4);

    mk.points.push_back(pt1);
    mk.points.push_back(pt2);

    mk.points.push_back(pt2);
    mk.points.push_back(pt4);

    mk.points.push_back(pt3);
    mk.points.push_back(pt4);
    
    mk.points.push_back(pt3);
    mk.points.push_back(pt1);
}

void FrontierGrid::Debug(list<int> &v_ids){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -1;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.scale.x = node_scale_;
    mk.scale.y = node_scale_;
    mk.scale.z = node_scale_;
    mk.color.a = 1.0;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    geometry_msgs::Point pt;
    for(auto &v_id : v_ids){
        if(v_id < 0 || v_id >= f_grid_.size()) continue;
        pt.x = f_grid_[v_id].center_(0);
        pt.y = f_grid_[v_id].center_(1);
        pt.z = f_grid_[v_id].center_(2);
        mk.points.emplace_back(pt);
    }
    debug_pub_.publish(mk);
}

void FrontierGrid::Debug(list<Eigen::Vector3d> &pts){
    visualization_msgs::Marker mk, mkr1, mkr2;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = scan_count_;
    scan_count_++;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;
    mk.color.a = 1.0;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;

    geometry_msgs::Point pt;
    for(auto &p : pts){
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        mk.points.push_back(pt);
    }
    debug_pub_.publish(mk);
}

void FrontierGrid::DebugShowAll(){
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -1;
    // scan_count_++;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.scale.x = 0.25;
    mk.scale.y = 0.25;
    mk.scale.z = 0.25;
    mk.color.a = 0.3;
    mk.color.b = 1.0;
    mk.pose.position.x = 0;
    mk.pose.position.y = 0;
    mk.pose.position.z = 0;
    mk.pose.orientation.x = 0;
    mk.pose.orientation.y = 0;
    mk.pose.orientation.z = 0;
    mk.pose.orientation.w = 1;
    for(auto &f : f_grid_){
        if(f.f_state_ == 1){
            for(int i = 0; i < samp_num_; i++){
                if(f.local_vps_[i] == 1){
                    Eigen::Vector4d vpt;
                    GetVp(f.center_, i, vpt);
                    geometry_msgs::Point pt;
                    pt.x = vpt(0);
                    pt.y = vpt(1);
                    pt.z = vpt(2);
                    mk.points.emplace_back(pt);
                }
            }
        }
    }
    debug_pub_.publish(mk);
}

void FrontierGrid::DebugViewpoint(){
    for(int i = 0; i < f_grid_.size(); i++){
        if(f_grid_[i].f_state_ != 2){
            ROS_WARN("id: %d unexplored f:%d", SDM_->self_id_, i);
            cout<<"id:"<<int(SDM_->self_id_)<<" c:"<<f_grid_[i].center_.transpose()<<" flag:"<<int(f_grid_[i].f_state_)<<endl;
            for(int j = 0; j < f_grid_[i].local_vps_.size(); j++){
                cout<<"id:"<<int(SDM_->self_id_)<<" v:"<<j<<" vs:"<<int(f_grid_[i].local_vps_[j])<<endl;
            }
        }
    }
}