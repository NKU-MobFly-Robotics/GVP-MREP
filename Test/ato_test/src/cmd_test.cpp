#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/LocalTraj.h>
#include <swarm_exp_msgs/SwarmTraj.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/PositionTarget.h>

#include <gcopter/traj_opt.h>
#include <yaw_planner/yaw_planner.h>

#include <glog/logging.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

ros::Publisher control_pub_, show_pub_, trigger_pub_, command_real_pub_;
ros::Subscriber traj_sub_;
ros::Subscriber takeoff_sub_;
int traj_state_;
Trajectory<5> traj_;  
Eigen::Vector3d recover_pt_;
list<swarm_exp_msgs::SwarmTraj> trajs_;
double start_t_;  
// YawPlanner yaw_traj_;

bool ready_;
Eigen::Vector3d robot_pos_, takeoff_pos_;
double robot_yaw_;
double max_v_, max_a_, max_yawd_, max_yawdd_;

double init_t_;
int id_, drone_num_;

ros::Timer run_timer_;
void Showcmd(Eigen::Vector3d pos, double yaw);
bool TryUpdateTraj();

Eigen::Vector4d getColor(const double& h, double alpha) {
  double h1 = h;
  if (h1 < 0.0 || h1 > 1.0) {
    std::cout << "h out of range" << std::endl;
    h1 = 0.0;
  }

  double lambda;
  Eigen::Vector4d color1, color2;
  if (h1 >= -1e-4 && h1 < 1.0 / 6) {
    lambda = (h1 - 0.0) * 6;
    color1 = Eigen::Vector4d(1, 0, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 1, 1);
  } else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6) {
    lambda = (h1 - 1.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 0, 1, 1);
  } else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6) {
    lambda = (h1 - 2.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 0, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 1, 1);
  } else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6) {
    lambda = (h1 - 3.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 1, 1);
    color2 = Eigen::Vector4d(0, 1, 0, 1);
  } else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6) {
    lambda = (h1 - 4.0 / 6) * 6;
    color1 = Eigen::Vector4d(0, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 1, 0, 1);
  } else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4) {
    lambda = (h1 - 5.0 / 6) * 6;
    color1 = Eigen::Vector4d(1, 1, 0, 1);
    color2 = Eigen::Vector4d(1, 0, 0, 1);
  }

  Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
  fcolor(3) = alpha;

  return fcolor;
}

void RunCallback(const ros::TimerEvent &e){
    if(!ready_){
        mavros_msgs::PositionTarget real_pt;
        real_pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        real_pt.type_mask = 0;
        real_pt.position.x = takeoff_pos_(0) + 1.0;
        real_pt.position.y = takeoff_pos_(1);
        real_pt.position.z = takeoff_pos_(2);
        real_pt.yaw = 0;
        real_pt.yaw_rate = 0;
        command_real_pub_.publish(real_pt);
        return;
    }    trajectory_msgs::MultiDOFJointTrajectory samples_array;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    Eigen::Vector3d p, v, a;
    double yaw, yawd, yawdd;
    // ROS_WARN("run0");
    while (TryUpdateTraj()){}
    if(traj_state_ == 1){
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
        if(cur_t - start_t_> traj_.getTotalDuration()){
            cur_t = start_t_ + traj_.getTotalDuration() - 1e-4;
        }
        p = traj_.getPos(cur_t - start_t_);
        v = traj_.getVel(cur_t - start_t_);
        a = traj_.getAcc(cur_t - start_t_);
        // yaw_traj_.GetCmd(min((cur_t - start_t_)*1.3, cur_t - start_t_ + 0.3), yaw, yawd, yawdd);
        // yaw_traj_.GetCmd(cur_t - start_t_, yaw, yawd, yawdd);
        // double temp_yaw;
        // yaw_traj_.GetCmd(cur_t - start_t_, temp_yaw, yawd, yawdd);
        yaw = 0;
        yawd = 0;
        yawdd = 0;
        if(v.norm() > max_v_)
            v = v.normalized() * max_v_;
        if(a.norm() > max_a_)
            a = a.normalized() * max_a_;
        if(abs(yawd) > max_yawd_) yawd = yawd / abs(yawd) * max_yawd_;
        if(abs(yawdd) > max_yawdd_) yawdd = yawdd / abs(yawdd) * max_yawdd_;
    }
    else return;
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

    mavros_msgs::PositionTarget real_pt;
    real_pt.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    real_pt.type_mask = 0;
    real_pt.position.x = p(0);
    real_pt.position.y = p(1);
    real_pt.position.z = p(2);
    real_pt.velocity.x = v(0);
    real_pt.velocity.y = v(1);
    real_pt.velocity.z = v(2);
    real_pt.acceleration_or_force.x = a(0);
    real_pt.acceleration_or_force.y = a(1);
    real_pt.acceleration_or_force.z = a(2);
    real_pt.yaw = yaw;
    real_pt.yaw_rate = yawd;

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    control_pub_.publish(samples_array);
    command_real_pub_.publish(real_pt);
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
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.3;
    mka.markers[0].scale.y = 0.3;
    mka.markers[0].scale.z = 0.3;
    mka.markers[0].color.a = 1.0;
    mka.markers[0].color.r = 0.9;
    mka.markers[0].color.g = 0.1;
    mka.markers[0].color.b = 0.7;
    mka.markers[0].points.resize(1);
    mka.markers[0].points[0].x = pos(0);
    mka.markers[0].points[0].y = pos(1);
    mka.markers[0].points[0].z = pos(2);

    mka.markers[0].pose.orientation.w = 1.0;
    show_pub_.publish(mka);
}

void ShowTraj(){
    visualization_msgs::MarkerArray mka;
    mka.markers.resize(1);
    mka.markers[0].header.frame_id = "world";
    mka.markers[0].header.stamp = ros::Time::now();
    mka.markers[0].id = 1;
    mka.markers[0].action = visualization_msgs::Marker::ADD;
    mka.markers[0].type = visualization_msgs::Marker::SPHERE_LIST;
    mka.markers[0].scale.x = 0.1;
    mka.markers[0].scale.y = 0.1;
    mka.markers[0].scale.z = 0.1;
    mka.markers[0].color.a = 1.0;
    auto color = getColor((id_ - 1) / double(drone_num_ - 1), 1);
    mka.markers[0].color.r = color[0];
    mka.markers[0].color.g = color[1];
    mka.markers[0].color.b = color[2];
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
        // for(double delta = 0; delta < traj_.getTotalDuration(); delta += 0.10){
        //     Eigen::Vector3d p;
        //     geometry_msgs::Point pt;
        //     double yaw_p, yaw_v, yaw_a;
        //     p = traj_.getPos(delta);
        //     pt.x = p(0);
        //     pt.y = p(1);
        //     pt.z = p(2);
        //     mka.markers[1].points.emplace_back(pt);
        //     yaw_traj_.GetCmd(delta, yaw_p, yaw_v, yaw_a);
        //     pt.x += cos(yaw_p) * 0.25;
        //     pt.y += sin(yaw_p) * 0.25;
        //     mka.markers[1].points.emplace_back(pt);
        // }
    }
    show_pub_.publish(mka);
}

void TrajCallback(const swarm_exp_msgs::SwarmTrajConstPtr &traj){
    trajs_.push_back(*traj);
    ready_ = true;
}

bool TryUpdateTraj(){
    if(trajs_.empty()) return false;
    double cur_t = ros::WallTime::now().toSec();
    if(trajs_.front().start_t > cur_t) return false;

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
    traj_state_ = -1;
    ready_ = false;
    command_real_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 5);
    control_pub_ = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
        > ("/command/trajectory", 5);
    string ns = ros::this_node::getName();
    show_pub_ = nh.advertise<visualization_msgs::MarkerArray>(ns + "/traj_show", 5);
    traj_sub_ = nh.subscribe("/trajectory_cmd", 1, &TrajCallback);
    run_timer_ = nh.createTimer(ros::Duration(0.033), &RunCallback);
    trigger_pub_ = nh.advertise<std_msgs::Empty>("/start_trigger", 5);
    nh_private.param(ns + "/Exp/takeoff_x", takeoff_pos_(0), 0.0);
    nh_private.param(ns + "/Exp/takeoff_y", takeoff_pos_(1), 0.0);
    nh_private.param(ns + "/Exp/takeoff_z", takeoff_pos_(2), 1.0);
    nh_private.param(ns + "/opt/MaxVel", max_v_, 1.5);
    nh_private.param(ns + "/opt/MaxAcc", max_a_, 1.5);
    nh_private.param(ns + "/opt/YawVel", max_yawd_, 1.5);
    nh_private.param(ns + "/opt/YawAcc", max_yawdd_, 1.5);
    nh_private.param("drone_id", id_, 1);
    nh_private.param("drone_num", drone_num_, 1);

    ros::spin();
    return 0;
}