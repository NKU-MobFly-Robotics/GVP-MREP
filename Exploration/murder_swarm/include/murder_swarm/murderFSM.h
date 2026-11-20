#ifndef MURDER_SWARM_FSM_H_
#define MURDER_SWARM_FSM_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <murder_swarm/murder.h>

class MurderFSM{
public:
    /**
     * EXCUTE: excute traj, strongcheck several viewpoints, check the feasibility of the traj
     * SLEEP: before exploratin
     * FINISH: no explorable viewpoint, return start place
     * LOCALPLAN: to the closest local viewpoint
     * GLOBALPLAN: to the closest global viewpoint
     */
    enum M_State{EXCUTE, SLEEP, FINISH, LOCALPLAN, GLOBALPLAN };
    vector<string> state_names = {"EXCUTE", "SLEEP", "FINISH", "LOCALPLAN", "GLOBALPLAN"};
    MurderFSM(){};
    ~MurderFSM(){};
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
private:
    void FSMCallback(const ros::TimerEvent &e);
    void TriggerCallback(const std_msgs::EmptyConstPtr &msg);
    // anchors start_t_ to the master trigger /takeoff (statistics t=0)
    void TakeoffClockCallback(const std_msgs::EmptyConstPtr &msg);
    void ChangeState(const M_State &state);
    // flown-distance logging: accumulate at 10 Hz, write to CS_ ch4 every 2 s
    void LenOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void TrajLengthTimerCallback(const ros::TimerEvent &e);
    double fsm_min_duration_;
    double finish_t_, finish_hold_duration_;
    bool exploring_, start_trigger_;
    ros::Subscriber trigger_sub_;
    ros::Subscriber takeoff_clock_sub_;
    Murder M_planner_;
    ros::Timer fsm_timer_;
    M_State state_;
    double start_t_;

    ros::Subscriber len_odom_sub_;
    ros::Timer traj_length_timer_;
    double traj_length_ = 0.0;
    Eigen::Vector3d len_pos_ = Eigen::Vector3d::Zero();
    bool have_len_pos_ = false;
    ros::Time last_len_log_time_;
    bool finish_recorded_ = false;
};
#endif