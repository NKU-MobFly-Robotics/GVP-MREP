#include <murder_swarm/murderFSM.h>

void MurderFSM::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){
    ros::NodeHandle nh_ = nh;
    std::string ns = ros::this_node::getName();
    nh_private.param(ns + "/EXP/finish_hold_duration", finish_hold_duration_, 10.0);
    trigger_sub_ = nh_.subscribe("/start_trigger", 1, &MurderFSM::TriggerCallback, this);
    // master-trigger clock anchor: MurderFSM::start_t_ follows the statistics t=0
    takeoff_clock_sub_ = nh_.subscribe("/takeoff", 1, &MurderFSM::TakeoffClockCallback, this);

    // flown-distance logging (ch4), mirrors FAME/RACER
    len_odom_sub_ = nh_.subscribe("/odom", 10, &MurderFSM::LenOdomCallback, this);
    traj_length_timer_ = nh.createTimer(ros::Duration(0.1), &MurderFSM::TrajLengthTimerCallback, this);

    fsm_timer_ = nh.createTimer(ros::Duration(0.01), &MurderFSM::FSMCallback, this);
    M_planner_.init(nh, nh_private);


    finish_t_ = ros::WallTime::now().toSec();
    exploring_ = false;
    state_ = M_State::SLEEP;
}

void MurderFSM::ChangeState(const M_State &state){
    // ROS_WARN("FSM from %d to %d", state_, state);
    std::cout << "from  \033[0;46m"<<"id: "<<int(M_planner_.SDM_.self_id_)<<"  "
        <<state_names[state_]<<"\033[0m to \033[0;46m" <<state_names[state]<<"\033[0m" << std::endl;
    state_ = state; 
}

void MurderFSM::TriggerCallback(const std_msgs::EmptyConstPtr &msg){
    start_trigger_ = true;
    exploring_ = true;
    // start_t_ is anchored by TakeoffClockCallback on /takeoff (t=0);
    // do not overwrite it with the post-takeoff time here.
}

void MurderFSM::TakeoffClockCallback(const std_msgs::EmptyConstPtr &msg){
    start_t_ = ros::WallTime::now().toSec();
}

void MurderFSM::FSMCallback(const ros::TimerEvent &e){
    bool exc_plan = true;
    int ap = M_planner_.AllowPlan(ros::WallTime::now().toSec());
    
    if(state_ != FINISH){
        finish_t_ = ros::WallTime::now().toSec();
    }

    if(ap == 0) exc_plan = true; // allow plan                                                                           
    else if(ap == 1) exc_plan = false; // not satisfy plan interval         
    else if(ap == 2) exc_plan = true;  // current position not in free space
    // else if(ap == 3 && (state_ == M_State::EXCUTE /*|| state_ == M_State::LOCALPLAN*/)) exc_plan = false; // sensor not update
    else if(ap == 3) exc_plan = false; // sensor not update
    else if(ap == 4) exc_plan = true;  // viewpoints not sampled

    if(!exc_plan) {
        return;
    }

    switch (state_)
    {
        case M_State::EXCUTE :{
            /* trajectory check */
            if(!M_planner_.TrajCheck()){ // traj infeasible, find new target
                int rp = M_planner_.SwitchMode();
                if(1){ // try local plan, before sampling and DTG updating
                    cout<<"traj infeasible"<<endl;
                    ChangeState(M_State::LOCALPLAN);
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
            }

            /* viewpoints check */
            int vp_st = M_planner_.ViewPointsCheck(0.005);
            if(vp_st != 0){
                int rp = M_planner_.SwitchMode();
                if(1){ // try local plan, before sampling and DTG updating
                    cout<<"vp infeasible"<<endl;
                    ChangeState(M_State::LOCALPLAN);
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
            }

            /* Plan again, if time out */
            if(ap == 0){
                int replan = M_planner_.Replan(false, false);
                if(replan != 0){
                    int rp = M_planner_.SwitchMode();
                    if(1){ // try local plan, before sampling and DTG updating
                        ChangeState(M_State::LOCALPLAN);
                        M_planner_.SetPlanInterval(0.009);
                        break;
                    }
                }
            }
            
            /* still EXCUTE */
            M_planner_.SetPlanInterval(0.05);
            break;
        }
        case M_State::FINISH :{
            // one-shot finish marker, same as FAME/RACER ch6
            if(!finish_recorded_ && M_planner_.SDM_.statistic_){
                finish_recorded_ = true;
                M_planner_.SDM_.CS_.SetVolume(0.0, 5);
            }
            if(ros::WallTime::now().toSec() - start_t_ > 20.0) M_planner_.BroadCastFinish();
            if(ap == 2){                                       //traj is still free, wait till reach free place
                M_planner_.SetPlanInterval(0.009);
                break;
            }
            int rp = M_planner_.SwitchMode();
            if(rp != 0){ // live again
                ChangeState(M_State::LOCALPLAN);
                M_planner_.SetPlanInterval(0.009);
                break;
            }
            else{
                if(ros::WallTime::now().toSec() - finish_t_ > finish_hold_duration_){
                    // if(M_planner_.GoHome()){
                        M_planner_.SetPlanInterval(2.0);
                    //     break;
                    // }
                }
            }
            break;
        }
        case M_State::GLOBALPLAN :{
            if(ap == 2){ // wait
                M_planner_.SetPlanInterval(0.009);
                break;
            }

            if(M_planner_.GlobalPlan()){ // plan success
                M_planner_.SetPlanInterval(0.009);
                ChangeState(M_State::EXCUTE);
                break;
            }
            else{ // wait
                int rp = M_planner_.SwitchMode();
                if(rp == 1){ // local plan again
                    ChangeState(M_State::LOCALPLAN);
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
                else if(rp == 2){ // global plan
                    M_planner_.SetPlanInterval(0.3);
                    break;
                }
                else{
                    ChangeState(M_State::FINISH);
                    // for(int i = 0; i < 10; i++) ROS_ERROR("finish1");
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
            }
        }
        case M_State::LOCALPLAN :{
            if(ap == 2){                                       //traj is still free, wait till reach free place
                M_planner_.SetPlanInterval(0.009); 
                break;
            }

            if(M_planner_.LocalPlan()){                 //plan success
                M_planner_.SetPlanInterval(0.009);
                ChangeState(M_State::EXCUTE);
                break;
            }
            else{
                int rp = M_planner_.SwitchMode();
                if(rp == 1){ // local plan again
                    M_planner_.SetPlanInterval(0.3);
                    M_planner_.GVP_.LocalExplorableDebug();
                    break;
                }
                else if(rp == 2){ // global plan
                    ChangeState(M_State::GLOBALPLAN);
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
                else{
                    ChangeState(M_State::FINISH);
                    // for(int i = 0; i < 10; i++) ROS_ERROR("finish2");
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
            }
            break;
        }
        case M_State::SLEEP :{
            if(!exploring_){
                if(start_trigger_)
                M_planner_.SetPlanInterval(0.3);
            }
            else{
                if(1){ // try local plan, before sampling and DTG updating
                    ChangeState(M_State::LOCALPLAN);
                    double sleep_t = 1.1 * M_planner_.SDM_.self_id_ / M_planner_.SDM_.drone_num_;
                    M_planner_.SetPlanInterval(sleep_t);
                    break;
                }
            }
            break;
        }
    }
}


void MurderFSM::LenOdomCallback(const nav_msgs::OdometryConstPtr &msg){
    len_pos_(0) = msg->pose.pose.position.x;
    len_pos_(1) = msg->pose.pose.position.y;
    len_pos_(2) = msg->pose.pose.position.z;
}

void MurderFSM::TrajLengthTimerCallback(const ros::TimerEvent &e){
    if(!have_len_pos_){
        have_len_pos_ = true;
        last_len_log_time_ = ros::Time::now();
        return;
    }
    // path length is integrated from consecutive odometry samples; jumps
    // (sim resets) are ignored
    Eigen::Vector3d last = len_pos_;
    static Eigen::Vector3d prev = last;
    const double step = (last - prev).norm();
    if(step < 1.0) traj_length_ += step;
    prev = last;

    if(M_planner_.SDM_.statistic_ && exploring_ &&
        (ros::Time::now() - last_len_log_time_).toSec() >= 2.0){
        M_planner_.SDM_.CS_.SetVolume(traj_length_, 4);
        last_len_log_time_ = ros::Time::now();
    }
}
