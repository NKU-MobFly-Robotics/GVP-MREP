#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/LocalTraj.h>
#include <std_msgs/Empty.h>
#include "quadrotor_msgs/PositionCommand.h"


#include <gcopter/traj_opt.h>
#include <yaw_planner/yaw_planner.h>

#include <glog/logging.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher control_pub_, control_l_pub_, show_pub_, trigger_pub_;
ros::Subscriber traj_sub_;
ros::Subscriber takeoff_sub_;
int traj_state_;
Trajectory<5> traj_;  
Eigen::Vector3d recover_pt_;
list<swarm_exp_msgs::LocalTraj> trajs_;
double start_t_;  
YawPlanner yaw_traj_;

bool ready_;
Eigen::Vector3d robot_pos_, takeoff_pos_;
double robot_yaw_;
double max_v_, max_a_, max_yawd_, max_yawdd_;

double init_t_;

ros::Timer run_timer_;
void Showcmd(Eigen::Vector3d pos, double yaw);
bool TryUpdateTraj();

// FAME-aligned takeoff: climb to takeoff_z over 30 x 33 ms, then fly forward
// 1.25 m at 0.5 m/s (yaw fixed), no settle, then publish the start trigger.
void Takeoff(const std_msgs::EmptyPtr &msg){
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    quadrotor_msgs::PositionCommand cmd_lite;

    cmd_lite.header.stamp = ros::Time::now();
    cmd_lite.trajectory_id = 0;
    cmd_lite.position.x = takeoff_pos_(0);
    cmd_lite.position.y = takeoff_pos_(1);
    cmd_lite.position.z = takeoff_pos_(2);
    cmd_lite.velocity.x = 0.0;
    cmd_lite.velocity.y = 0.0;
    cmd_lite.velocity.z = 0.0;
    cmd_lite.acceleration.x = 0.0;
    cmd_lite.acceleration.y = 0.0;
    cmd_lite.acceleration.z = 0.0;
    cmd_lite.yaw = 0.0;
    cmd_lite.yaw_dot = 0.0;

    trajectory_point.position_W.x() = takeoff_pos_(0);
    trajectory_point.position_W.y() = takeoff_pos_(1);
    trajectory_point.orientation_W_B.w() = 1.0;

    for(int i = 0; i < 30; i++){
        trajectory_msgs::MultiDOFJointTrajectory samples_array;
        ros::Duration(0.033).sleep();
        const double ratio = static_cast<double>(i + 1) / 30.0;
        trajectory_point.position_W.z() = takeoff_pos_(2) * ratio;
        cmd_lite.position.z = takeoff_pos_(2) * ratio;
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        control_pub_.publish(samples_array);
        control_l_pub_.publish(cmd_lite);
    }
    trajectory_point.position_W.z() = takeoff_pos_(2);
    cmd_lite.position.z = takeoff_pos_(2);
    cmd_lite.velocity.x = cmd_lite.velocity.y = cmd_lite.velocity.z = 0.0;
    {
        trajectory_msgs::MultiDOFJointTrajectory samples_array;
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        control_pub_.publish(samples_array);
        control_l_pub_.publish(cmd_lite);
    }

    // forward flight, same as FAME: 1.25 m at 0.5 m/s along +x, yaw 0
    const double forward_dist = 1.45;
    const double forward_vel = 0.5;
    const double forward_duration = forward_dist / forward_vel;
    const ros::WallTime forward_start = ros::WallTime::now();
    while(ros::ok() && (ros::WallTime::now() - forward_start).toSec() < forward_duration){
        const double ratio = min(1.0,
            (ros::WallTime::now() - forward_start).toSec() / forward_duration);
        trajectory_msgs::MultiDOFJointTrajectory samples_array;
        const double fx = takeoff_pos_(0) + ratio * forward_dist;
        trajectory_point.position_W.x() = fx;
        cmd_lite.position.x = fx;
        cmd_lite.velocity.x = forward_vel;
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        control_pub_.publish(samples_array);
        control_l_pub_.publish(cmd_lite);
        ros::spinOnce();
        ros::WallDuration(0.033).sleep();
    }
    trajectory_point.position_W.x() = takeoff_pos_(0) + forward_dist;
    cmd_lite.position.x = takeoff_pos_(0) + forward_dist;
    cmd_lite.velocity.x = 0.0;
    {
        trajectory_msgs::MultiDOFJointTrajectory samples_array;
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        control_pub_.publish(samples_array);
        control_l_pub_.publish(cmd_lite);
    }

    trigger_pub_.publish(*msg);
    ready_ = true;
    takeoff_sub_.shutdown();
}

void RunCallback(const ros::TimerEvent &e){
    if(!ready_) return;
    trajectory_msgs::MultiDOFJointTrajectory samples_array;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    quadrotor_msgs::PositionCommand cmd_lite;

    Eigen::Vector3d p, v, a;
    double yaw, yawd, yawdd;
    // ROS_WARN("run0");
    while (TryUpdateTraj()){}
    if(traj_state_ == 1 && false){
        p = recover_pt_ - robot_pos_;
        p = robot_pos_ + min(p.norm(), 0.3) * p.normalized();
        v.setZero();
        a.setZero();
        yaw = robot_yaw_;
        yawd = 0;
        yawdd = 0;
    }
    else if(traj_state_ == 2){
        // cout<<traj_.getPieceNum()<<endl;
        double cur_t = ros::WallTime::now().toSec();
        // if(cur_t - start_t_> traj_.getTotalDuration()){
        //     cur_t = start_t_ + traj_.getTotalDuration() - 1e-4;
        //     v.setZero();
        //     a.setZero();
        // }
        // else{
        //     v = traj_.getVel(cur_t - start_t_);
        //     a = traj_.getAcc(cur_t - start_t_);
        // }
        // p = traj_.getPos(cur_t - start_t_);

        double t = cur_t - start_t_;
        if (t < 0.0) t = 0.0;                                   // 补下界
        if (t > traj_.getTotalDuration()) {
            t = traj_.getTotalDuration() - 1e-4;
            v.setZero(); a.setZero();
        } else {
            v = traj_.getVel(t); a = traj_.getAcc(t);
        }
        p = traj_.getPos(t);
        yaw_traj_.GetCmd(t, yaw, yawd, yawdd);


        // yaw_traj_.GetCmd(min((cur_t - start_t_)*1.3, cur_t - start_t_ + 0.3), yaw, yawd, yawdd);
        // yaw_traj_.GetCmd(cur_t - start_t_, yaw, yawd, yawdd);
        // double temp_yaw;
        // yaw_traj_.GetCmd(cur_t - start_t_, temp_yaw, yawd, yawdd);
        if(v.norm() > max_v_)
            v = v.normalized() * max_v_;
        if(a.norm() > max_a_)
            a = a.normalized() * max_a_;
        if(abs(yawd) > max_yawd_) yawd = yawd / abs(yawd) * max_yawd_;
        if(abs(yawdd) > max_yawdd_) yawdd = yawdd / abs(yawdd) * max_yawdd_;
    }
    else return;

    // 执行端兜底：位置超过 ±100m 或非有限值时强制跳过本周期命令，
    // poscmd_2_odom 会保持上一条合法命令，阻断发散值进入 odometry 反馈环
    if(!p.allFinite() || p.cwiseAbs().maxCoeff() > 100.0){
        ROS_ERROR_THROTTLE(1.0, "traj_exc: skip insane position cmd (%.3e, %.3e, %.3e)",
                           p.x(), p.y(), p.z());
        return;
    }

    trajectory_point.position_W.x() = p.x();
    trajectory_point.position_W.y() = p.y();
    trajectory_point.position_W.z() = p.z();
    trajectory_point.velocity_W.x() = v.x();
    trajectory_point.velocity_W.y() = v.y();
    trajectory_point.velocity_W.z() = v.z();
    trajectory_point.acceleration_W.x() = a.x();
    trajectory_point.acceleration_W.y() = a.y();
    trajectory_point.acceleration_W.z() = a.z();

    trajectory_point.setFromYaw(yaw);
    trajectory_point.setFromYawRate(yawd);
    trajectory_point.setFromYawAcc(yawdd);

    cmd_lite.position.x = p.x();
    cmd_lite.position.y = p.y();
    cmd_lite.position.z = p.z();
    cmd_lite.velocity.x = v.x();
    cmd_lite.velocity.y = v.y();
    cmd_lite.velocity.z = v.z();
    cmd_lite.acceleration.x = a.x();
    cmd_lite.acceleration.y = a.y();
    cmd_lite.acceleration.z = a.z();
    cmd_lite.yaw = yaw;
    cmd_lite.yaw_dot = yawd;

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    control_pub_.publish(samples_array);
    control_l_pub_.publish(cmd_lite);

    Showcmd(p, yaw);
}

void Showcmd(Eigen::Vector3d pos, double yaw){
    static double last_update = ros::WallTime::now().toSec();
    if(ros::WallTime::now().toSec() - last_update < 0.08) return;
    last_update = ros::WallTime::now().toSec();
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 3;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::LINE_STRIP;
    mka.markers[0].scale.x = 0.15;
    mka.markers[0].scale.y = 0.15;
    mka.markers[0].scale.z = 0.15;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.9;
    mka.markers[0].color.g = 0.1;
    mka.markers[0].color.b = 0.7;
    mka.markers[0].points.resize(2);
    mka.markers[0].points[0].x = pos(0);
    mka.markers[0].points[0].y = pos(1);
    mka.markers[0].points[0].z = pos(2);
    mka.markers[0].points[1].x = pos(0) + cos(yaw) * 0.5;
    mka.markers[0].points[1].y = pos(1) + sin(yaw) * 0.5;
    mka.markers[0].points[1].z = pos(2);
    mka.markers[0].pose.orientation.w = 1.0;
    show_pub_.publish(mka);
}

void ShowTraj(){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(2);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 1;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.03;
    mka.markers[0].scale.y = 0.03;
    mka.markers[0].scale.z = 0.03;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.2;
    mka.markers[0].color.g = 0.9;
    mka.markers[0].color.b = 0.1;

    mka.markers[1] = mka.markers[0];
    mka.markers[1].type = visualization_msgs::Marker::LINE_LIST;
    mka.markers[1].id = 2;
    mka.markers[1].color.r = 0.8;
    mka.markers[1].color.g = 0.3;
    mka.markers[1].scale.x = 0.03;
    mka.markers[1].scale.y = 0.03;
    mka.markers[1].scale.z = 0.03;
    if(traj_state_ == 1){
        Eigen::Vector3d p;
        for(double d = 0; d <= 1.0; d += 0.0499){
            p = (1-d)*recover_pt_ + d*robot_pos_;
            geometry_msgs::Point pt;
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            mka.markers[0].points.emplace_back(pt);
        }
    }
    else{
        for(double delta = 0; delta < traj_.getTotalDuration(); delta += 0.025){
            Eigen::Vector3d p;
            geometry_msgs::Point pt;
            p = traj_.getPos(delta);
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            mka.markers[0].points.emplace_back(pt);
        }
        for(double delta = 0; delta < traj_.getTotalDuration(); delta += 0.10){
            Eigen::Vector3d p;
            geometry_msgs::Point pt;
            double yaw_p, yaw_v, yaw_a;
            p = traj_.getPos(delta);
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            mka.markers[1].points.emplace_back(pt);
            yaw_traj_.GetCmd(delta, yaw_p, yaw_v, yaw_a);
            pt.x += cos(yaw_p) * 0.25;
            pt.y += sin(yaw_p) * 0.25;
            mka.markers[1].points.emplace_back(pt);
        }
    }
    show_pub_.publish(mka);
}

void TrajCallback(const swarm_exp_msgs::LocalTrajConstPtr &traj){
    trajs_.push_back(*traj);

}

bool TryUpdateTraj(){
    if(trajs_.empty()) return false;
    double cur_t = ros::WallTime::now().toSec();
    if(trajs_.front().start_t > cur_t) return false;
    if(trajs_.front().state == 1){
        traj_state_ = 1;
        recover_pt_(0) = trajs_.front().recover_pt.x;
        recover_pt_(1) = trajs_.front().recover_pt.y;
        recover_pt_(2) = trajs_.front().recover_pt.z;
    }
    else if(trajs_.front().state == 2){
        traj_state_ = 2;
        start_t_ = trajs_.front().start_t;
        int col = 0;
        int t_idx = 0;
        Eigen::MatrixXd cM(3, 6);
        traj_.clear();

        for(int i = 0; i < trajs_.front().coef_p.size(); i++, col++){
            cM(0, col) = trajs_.front().coef_p[i].x;
            cM(1, col) = trajs_.front().coef_p[i].y;
            cM(2, col) = trajs_.front().coef_p[i].z;
            if(col == 5){
                traj_.emplace_back(double(trajs_.front().t_p[t_idx]), cM);
                col = -1;
                t_idx++;
            }
        }

        t_idx = 0;
        yaw_traj_.A_.resize(trajs_.front().coef_yaw.size());
        yaw_traj_.T_.resize(trajs_.front().t_yaw.size());
        for(int i = 0; i < trajs_.front().coef_yaw.size(); i++){
            yaw_traj_.A_(i) = double(trajs_.front().coef_yaw[i]);
            if((i + 1) % 6 == 0){
                yaw_traj_.T_(t_idx) = double(trajs_.front().t_yaw[t_idx]);
                t_idx++;
            }
        }
        double p,v ,a;
        yaw_traj_.GetCmd(yaw_traj_.T_.sum(), p, v, a);
    }
    // cout<<"traj_state_:"<<traj_state_<<"   "<<cur_t - init_t_<<endl;
    ShowTraj();
    trajs_.pop_front();
    return true;
}

void OdomCallback(const nav_msgs::OdometryConstPtr& odom){
    robot_pos_.x() = odom->pose.pose.position.x;
    robot_pos_.y() = odom->pose.pose.position.y;
    robot_pos_.z() = odom->pose.pose.position.z;
    Eigen::Quaterniond qua;
    qua.x() = odom->pose.pose.orientation.x;
    qua.y() = odom->pose.pose.orientation.y;
    qua.z() = odom->pose.pose.orientation.z;
    qua.w() = odom->pose.pose.orientation.w;
    robot_yaw_ = atan2(qua.matrix()(1, 0), qua.matrix()(0, 0));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "frontier_test");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

    init_t_ = ros::WallTime::now().toSec();
    bool auto_takeoff;
    traj_state_ = -1;
    ready_ = false;
    control_pub_ = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
        > ("/command/trajectory", 5);

    control_l_pub_ = nh.advertise<quadrotor_msgs::PositionCommand> ("/command/traj_lite", 10);
    string ns = ros::this_node::getName();
    show_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/traj_show", 5);
    traj_sub_ = nh.subscribe("/trajectory_cmd", 1, &TrajCallback);
    run_timer_ = nh.createTimer(ros::Duration(0.033), &RunCallback);
    trigger_pub_ = nh.advertise<std_msgs::Empty>("/start_trigger", 5);
    nh_private.param(ns + "/Exp/AutoTakeoff", auto_takeoff, false);
    nh_private.param(ns + "/Exp/takeoff_x", takeoff_pos_(0), 0.0);
    nh_private.param(ns + "/Exp/takeoff_y", takeoff_pos_(1), 0.0);
    nh_private.param(ns + "/Exp/takeoff_z", takeoff_pos_(2), 1.0);
    nh_private.param(ns + "/opt/MaxVel", max_v_, 1.5);
    nh_private.param(ns + "/opt/MaxAcc", max_a_, 1.5);
    nh_private.param(ns + "/opt/YawVel", max_yawd_, 1.5);
    nh_private.param(ns + "/opt/YawAcc", max_yawdd_, 1.5);

    if(auto_takeoff){
        std_msgs::EmptyPtr e;
        ros::Duration(5.0).sleep();
        Takeoff(e);
    }
    else{
        takeoff_sub_ = nh.subscribe("/takeoff", 1, &Takeoff);
    }
    ros::spin();
    return 0;
}