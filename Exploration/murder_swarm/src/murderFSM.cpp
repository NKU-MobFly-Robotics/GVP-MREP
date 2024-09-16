#include <murder_swarm/murderFSM.h>

void MurderFSM::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){
    ros::NodeHandle nh_ = nh;
    std::string ns = ros::this_node::getName();
    nh_private.param(ns + "/EXP/finish_hold_duration", finish_hold_duration_, 10.0);
    trigger_sub_ = nh_.subscribe("/start_trigger", 1, &MurderFSM::TriggerCallback, this);

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
    start_t_ = ros::WallTime::now().toSec();
}

void MurderFSM::FSMCallback(const ros::TimerEvent &e){
    bool exc_plan;
    int ap = M_planner_.AllowPlan(ros::WallTime::now().toSec());
    
    if(state_ != FINISH){
        finish_t_ = ros::WallTime::now().toSec();
    }

    if(ap == 0) exc_plan = true;                                                                            
    else if(ap == 1) exc_plan = false;                    
    else if(ap == 2) exc_plan = true;              
    else if(ap == 3 && (state_ == M_State::EXCUTE /*|| state_ == M_State::LOCALPLAN*/)) exc_plan = false; 
    else if(ap == 3) exc_plan = false;
    else if(ap == 4) exc_plan = true;

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
                    // M_planner_.Stay(M_planner_.init_pose_);
                M_planner_.SetPlanInterval(0.3);
            }
            else{
                int rp = M_planner_.SwitchMode();
                if(1){ // try local plan, before sampling and DTG updating
                    ChangeState(M_State::LOCALPLAN);
                    M_planner_.SetPlanInterval(0.009);
                    break;
                }
            }
            break;
        }
    }
}

