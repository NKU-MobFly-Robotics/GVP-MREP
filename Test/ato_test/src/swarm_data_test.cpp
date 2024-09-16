#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <gazebo_msgs/SetModelState.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <glog/logging.h>


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

ros::Publisher control_pub_, show_pub_;
ros::Subscriber target_sub_, plan_sub_;

shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
SynchronizerImageOdom sync_image_odom_;
shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
vector<string> all_mns_;

SwarmDataManager *SDM_ptr_;
BlockMap *BM_ptr_;
lowres::LowResMap *LRM_ptr_;
Eigen::Vector3d robot_pos_;

geometry_msgs::Pose swarm_pose_;

ros::Timer fresh_timer_;
bool set_pos = false;
void SetCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    swarm_pose_ = msg->pose.pose;
    set_pos = true;
    swarm_pose_.position.z = 1.25;


}

void FreshTimerCallback(const ros::TimerEvent &e){
    static int iii = 0;
    if(!set_pos) return;
    int ii = iii % 70;
    iii++;
    swarm_pose_.position.z = 1.2 + 0.7 * sin(M_PI * ii / 33);
    swarm_pose_.orientation.w = cos(M_PI * ii / 70);
    swarm_pose_.orientation.x = sin(M_PI * ii / 70);
    swarm_pose_.orientation.y = 0;
    swarm_pose_.orientation.z = 0;
    for(int i = 0; i < SDM_ptr_->Poses_.size(); i++){
        if(i + 1 == SDM_ptr_->self_id_) continue;
        SDM_ptr_->Poses_[i] = swarm_pose_;
        SDM_ptr_->Pose_t_[i] = ros::WallTime::now().toSec();
        gazebo_msgs::SetModelState state_srv;
        state_srv.request.model_state.reference_frame = "world";
        state_srv.request.model_state.model_name = all_mns_[i];
        state_srv.request.model_state.pose = swarm_pose_;
        cout<<"set:"<<swarm_pose_.orientation.x<<" "<<swarm_pose_.orientation.y<<" "<<swarm_pose_.orientation.z<<" "<<swarm_pose_.orientation.w<<" "<<endl;
        state_srv.request.model_state.twist.linear.x = 0;
        state_srv.request.model_state.twist.linear.y = 0;
        state_srv.request.model_state.twist.linear.z = 0;
        state_srv.request.model_state.twist.angular.x = 0;
        state_srv.request.model_state.twist.angular.y = 0;
        state_srv.request.model_state.twist.angular.z = 0;
        ros::service::call("/gazebo/set_model_state", state_srv);
    }
        visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 1;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.scale.x = 0.3;
    mk.scale.y = 0.3;
    mk.scale.z = 0.1;
    mk.color.a = 0.4;
    mk.color.g = 1.0;
    mk.pose = swarm_pose_;
    show_pub_.publish(mk);
}


void TargetCallback(const geometry_msgs::PoseStampedPtr &msg){

    trajectory_msgs::MultiDOFJointTrajectory samples_array;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W.x() = msg->pose.position.x;
    trajectory_point.position_W.y() = msg->pose.position.y;
    trajectory_point.position_W.z() = 1.5;

    if(!LRM_ptr_->IsFeasible(trajectory_point.position_W)) {
        ROS_ERROR("infeasible!!!!!");
        return;
    }
    trajectory_point.orientation_W_B.w() = msg->pose.orientation.w;
    trajectory_point.orientation_W_B.x() = msg->pose.orientation.x;
    trajectory_point.orientation_W_B.y() = msg->pose.orientation.y;
    trajectory_point.orientation_W_B.z() = msg->pose.orientation.z;
    ROS_WARN("t0");
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
    samples_array.points.push_back(trajectory_point_msg);
    control_pub_.publish(samples_array);
}

void ImgOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom)
{
    static double last = ros::WallTime::now().toSec();
    if(ros::WallTime::now().toSec() - last < 0.1) return;
    Quaterniond qua;
    Eigen::Matrix4d robot_pose;
    robot_pose.setOnes();
    qua.x() = odom->pose.pose.orientation.x;
    qua.y() = odom->pose.pose.orientation.y;
    qua.z() = odom->pose.pose.orientation.z;
    qua.w() = odom->pose.pose.orientation.w;

    robot_pos_(0) = odom->pose.pose.position.x;
    robot_pos_(1) = odom->pose.pose.position.y;
    robot_pos_(2) = odom->pose.pose.position.z;

    robot_pose.block(0, 0, 3, 3) = qua.toRotationMatrix();
    robot_pose.block(0, 3, 3, 1) = robot_pos_;
    ros::WallTime t0 = ros::WallTime::now();
    BM_ptr_->OdomCallback(odom);
    BM_ptr_->InsertImg(img);
    LRM_ptr_->SetEternalNode(robot_pos_);
    LRM_ptr_->UpdateLocalBBX(robot_pose, BM_ptr_->cur_pcl_);
    cout<<"cost:"<<(ros::WallTime::now() - t0).toSec()<<endl;
    last = t0.toSec();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "swarm_data_test");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    string ns = ros::this_node::getName();
    nh_private.param(ns + "/multisim/model_names", all_mns_, {string("robot")});

    SwarmDataManager SDM;
    SDM_ptr_ = &SDM;
    SDM.init(nh, nh_private);

    BlockMap BM;
    BM_ptr_ = &BM;
    BM.init(nh, nh_private);
    BM.SetSwarmDataManager(SDM_ptr_);
    lowres::LowResMap LRM;

    ColorManager CM;
    CM.init(nh, nh_private);
    LRM.SetColorManager(&CM);


    LRM.SetMap(&BM);
    LRM.init(nh, nh_private);
    LRM_ptr_ = &LRM;
    control_pub_ = nh.advertise < trajectory_msgs::MultiDOFJointTrajectory
        > ("/command/trajectory", 5);

    show_pub_ = nh.advertise<visualization_msgs::Marker>("/other_robot", 5);

    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh, "/depth", 10));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 10));
    sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(boost::bind(ImgOdomCallback,  _1, _2));
    target_sub_ = nh.subscribe("/move_base_simple/goal", 1, &TargetCallback);
    plan_sub_ = nh.subscribe("/initialpose", 1, &SetCallback);
    fresh_timer_ = nh.createTimer(ros::Duration(0.05), &FreshTimerCallback);
    ros::Duration(1.5).sleep();
    {
        trajectory_msgs::MultiDOFJointTrajectory samples_array;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W.x() = 0.0;
        trajectory_point.position_W.y() = 0.0;
        trajectory_point.position_W.z() = 1.5;
        trajectory_point.orientation_W_B.w() = 1.0;
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        control_pub_.publish(samples_array);
        ros::Duration(1.5).sleep();
        trajectory_point.position_W.x() = 1.5;
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        control_pub_.publish(samples_array);
    }
    ros::spin();
    return 0;
}