#include <multisim_gazebo/multi_rotors.h>
#include <gazebo_msgs/SetModelState.h>
// #include <gazebo_msgs/SetModelStateRequest.h>

void MultiRotors::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private){
    nh_ = nh;
    nh_private_ = nh_private;
    string ns = ros::this_node::getName();
    // nh_private_.param("/multirotors/is_center", center_gazebo_, false);
    nh_private_.param(ns + "/multirotors/robot_num", robot_num_, 1);
    nh_private_.param(ns + "/multirotors/local_num", local_num_, 1);
    nh_private_.param(ns + "/multirotors/local_ids", local_ids_, {1});
    nh_private_.param(ns + "/multirotors/basic_names", local_blks_, {string("robot")});
    nh_private_.param(ns + "/multirotors/model_names", all_mns_, {string("robot")});
    nh_private_.param(ns + "/multirotors/set_freq", set_freq_, 20.0);
    odoms_.resize(robot_num_);
    odoms_sub_.resize(local_num_);
    have_odom_.resize(robot_num_, 0);
    odoms_show_pub_.resize(robot_num_);
    states_sub_ = nh.subscribe("/communication/states", 10, &MultiRotors::StatesCallback, this);
    states_pub_ = nh.advertise<multisim_gazebo::states>("/multisim_local/states", 50);

    for(int i = 0; i < local_num_; i++){
        odoms_sub_[i] = nh.subscribe("/multisim_local/odom" + to_string(local_ids_[i]), 1, &MultiRotors::OdomCallback, this);
    }
    for(int i = 0; i < robot_num_; i++){
        bool adv_flag = true;
        for(auto &id : local_ids_) if(id == i + 1) {
            adv_flag = false;
            break;
        }   
        if(adv_flag) odoms_show_pub_[i] = nh.advertise<nav_msgs::Odometry>("/multisim_swarm/odom" + to_string(i+1), 1);
    }
    states_timer_ = nh_private_.createTimer(ros::Duration(1/set_freq_), &MultiRotors::StatesTimerCallback, this);
}

void MultiRotors::StatesCallback(const multisim_gazebo::statesConstPtr& msg){
    double dt = abs((msg->stamp - ros::WallTime::now().toSec()));
    // if(dt > 0.1){
    //     ROS_ERROR("from %d delay%f", msg->from_computer, dt);
    // }
    // else{
        for(auto &odom : msg->odoms){
            int o_idx = odom.id - 1;
            if(o_idx + 1 <= robot_num_){
                odoms_[o_idx] = odom.odoms;
                have_odom_[o_idx] = 1;
            }
            else{
                ROS_ERROR("unknown odom id: %d", o_idx + 1);
            }
        }
    // }
}

void MultiRotors::StatesTimerCallback(const ros::TimerEvent& e){
    ros::WallTime cs = ros::WallTime::now();
    multisim_gazebo::states msg;
    msg.odoms.resize(local_num_);
    for(int i = 0; i < local_num_; i++){
        msg.odoms[i].odoms = odoms_[local_ids_[i] - 1];
        msg.odoms[i].id = local_ids_[i];
    }
    msg.stamp = ros::WallTime::now().toSec();
    states_pub_.publish(msg);
    for(int i = 0; i < robot_num_; i++){
        bool local_flag = false;
        for(auto &l_id : local_ids_){
            if(l_id - 1 == i) {
                local_flag = true;
                break;
            } 
        }
        if(local_flag || !have_odom_[i]) continue;
        gazebo_msgs::SetModelState state_srv;
        // int idx = local_ids_[i] - 1;
        state_srv.request.model_state.reference_frame = "world";
        state_srv.request.model_state.model_name = all_mns_[i];
        state_srv.request.model_state.pose = odoms_[i].pose.pose;
        state_srv.request.model_state.twist = odoms_[i].twist.twist;
        ros::service::call("/gazebo/set_model_state", state_srv);

        odoms_show_pub_[i].publish(odoms_[i]);
    }
    if((ros::WallTime::now() - cs).toSec() > 1/set_freq_)
        cout<<"call spent:"<<(ros::WallTime::now() - cs).toSec()<<endl;
}

void MultiRotors::OdomCallback(const nav_msgs::OdometryConstPtr& msg){
    string child_frame = msg->child_frame_id;
    int l_idx = 0;
    for(auto &blk : local_blks_){
        if(blk == child_frame) break;
        l_idx++;
    }
    if(l_idx == local_ids_.size()) {
        ROS_ERROR("unknown frame:%s", child_frame.c_str());
        return;
    }
    int o_idx = local_ids_[l_idx] - 1;
    have_odom_[o_idx] = 1;
    odoms_[o_idx] = *msg;
}



