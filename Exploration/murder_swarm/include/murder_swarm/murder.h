#ifndef MURDER_SWARM_H_
#define MURDER_SWARM_H_

#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <tr1/unordered_map>

#include <multiDTG/dtg_structures.h>
#include <multiDTG/multiDTG.h>
#include <block_map/color_manager.h>
#include <block_map/block_map.h>
#include <lowres_map/lowres_map.h>
#include <swarm_data/swarm_data.h>
#include <gcopter/traj_opt.h>
#include <frontier_grid/frontier_grid.h>
#include <yaw_planner/yaw_planner.h>
#include <swarm_data/swarm_data.h>
#include <graph_partition/graph_partition.h>

#include <visualization_msgs/MarkerArray.h>
#include <swarm_exp_msgs/LocalTraj.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

class Murder{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyPCLOdom;
typedef shared_ptr<message_filters::Synchronizer<SyncPolicyPCLOdom>> SynchronizerPCLOdom;
public: 
    Murder(){};
    ~Murder(){};
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void BroadCastFinish();
    
    /**
     * @brief 
     * 
     * @param T current time 
     * @return int allow: 0; not allow: 1(time fail), 2(pos infeasible), 3(sensor not update); may allow: 4(viewpoint not sampled)
     */
    int AllowPlan(const double &T);

    /**
     * @brief set allow plan flags, and set the plan interval
     * 
     * @param intv interval
     */
    void SetPlanInterval(const double &intv);

    bool GoHome();

    bool LocalPlan();

    bool GlobalPlan();

    bool TrajCheck();

    /**
     * @brief try to fly to a collision free place
     * 
     * @return true     find a place
     * @return false    GG 
     */
    bool Recover();

    /**
     * @brief slowly fly to pose
     * 
     * @param pose target pose
     */
    void Stay(const Eigen::Vector4d &pose);

    /**
     * @brief 
     * 
     * @param new_target true: do not replan. decide local or global plan. false: if current target not die, replan
     * @param ignore_t   false: replan, if reach the replan time. Otherwise return 0.
     * @return int 0: current or new trajectory is feasible. 1: please try another plan.
     */
    int Replan(const bool &new_target, const bool &ignore_t = false);

    /**
     * @brief 
     * 
     * @return int 0: finish, 1: local plan, 2: global plan 
     */
    int SwitchMode();
    /**
     * @brief 
     * 
     * @param t int 0: current or new trajectory is feasible. 1: please try another plan.
     * @return int 
     */
    int ViewPointsCheck(const double &t);
    void ImgOdomCallback(const sensor_msgs::ImageConstPtr& img,
                               const nav_msgs::OdometryConstPtr& odom);
    void PCLOdomCallback(const sensor_msgs::PointCloud2ConstPtr& pcl,
                               const nav_msgs::OdometryConstPtr& odom);
    void BodyOdomCallback(const nav_msgs::OdometryConstPtr& odom);
    void ShowTraj(vector<Eigen::Vector3d> &path, vector<Eigen::MatrixX4d> &h);
    void ShowPath(list<Eigen::Vector3d> &path, int id);
    Eigen::Vector4d init_pose_;
    SwarmDataManager SDM_;
    DTG::GraphVoronoiPartition GVP_;

private:
	inline void LoadShowPose(const geometry_msgs::Pose &pose);
    Eigen::Vector3d GetEndV(uint16_t f_id, uint8_t v_id, Eigen::Vector3d ps, Eigen::Vector3d vs, bool dangerous);

//     bool TrajPlanA(vector<Eigen::Vector3d> &safe_path, const Eigen::Vector3d &ps, const Eigen::Vector3d &vs, const Eigen::Vector3d &as,
//             const Eigen::Vector3d &pe, const Eigen::Vector3d &ve, const Eigen::Vector3d &ae, const double &yps, 
//             const double &yds, const double &ydds, const double &ype, const double &yde, const double &ydde, const Eigen::Vector3d &gazept);
    bool TrajPlanB(const Eigen::Vector3d &ps, const Eigen::Vector3d &vs, const Eigen::Vector3d &as,
            const Eigen::Vector3d &pe, const Eigen::Vector3d &ve, const Eigen::Vector3d &ae, const double &yps, 
            const double &yds, const double &ydds, const double &ype, const double &yde, const double &ydde, const Eigen::Vector3d &gazept, const double &handt);
    void PlanYaw(const double &yps, const double &yds, const double &ydds, const double &ype, 
            const double &yde, const double &ydde, const Eigen::Vector3d &gazept,bool gaze = false);
    void GetFollowPath(list<Eigen::Vector3d> &path, list<Eigen::Vector3d> &path_follow);
    void PublishTraj(bool recover);
    bool SwarmFeasiCheck();
    void Debug(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2);
    void Debug(list<Eigen::Vector3d> &pts);
    void CreateVisModel();
    BlockMap BM_;
    DTG::MultiDTG MDTG_;
    YawPlanner YawP_;
    lowres::LowResMap LRM_;
    AtoTraj TrajOpt_;
    FrontierGrid FG_;
    ColorManager CM_;

    ros::Publisher show_pub_, traj_pub_, posevis_pub_;
    ros::Subscriber odom_sub_;
    ros::NodeHandle nh_, nh_private_;

    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> vi_odom_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pcl_sub_;
    SynchronizerImageOdom sync_image_odom_;
    SynchronizerPCLOdom sync_pointcloud_odom_;
    visualization_msgs::MarkerArray vis_model_;

    list<pair<uint16_t, uint32_t>> GVP_hn_;  // self global partition
    list<pair<uint16_t, uint32_t>> local_hn_;  // self local partition
    list<uint16_t> local_fn_;   // self exploring fn

    Eigen::Vector3d p_, v_, home_p_;
    Eigen::Vector3d last_safe_;
    Eigen::Vector4d target_, recover_pose_;
    Eigen::Vector4d target_vp_pose_;
    int target_f_id_, target_v_id_; // -1: no target now, -2: go home
    Eigen::Matrix4d robot_pose_;
    double yaw_, yaw_v_;
    bool sensor_flag_, have_odom_;

    double last_map_update_t_;    //blockmap & DTG
    double traj_start_t_, traj_end_t_;
    double reach_out_t_;
    double traj_length_;
    double plan_t_, replan_t_;
    double exc_duration_, replan_duration_, check_duration_;
    double colli_range_;
    double strong_check_interval_;
    int local_max_search_iter_;
    bool dangerous_path_, reach_end_traj_, swarm_check_;
    double acc_off_;
    int global_max_search_iter_;
};

inline void Murder::LoadShowPose(const geometry_msgs::Pose &pose){
	vis_model_.markers[0].header.stamp = ros::Time::now();
	vis_model_.markers[1].header.stamp = ros::Time::now();
	vis_model_.markers[2].header.stamp = ros::Time::now();
	vis_model_.markers[3].header.stamp = ros::Time::now();
	vis_model_.markers[4].header.stamp = ros::Time::now();

	vis_model_.markers[0].pose.orientation = pose.orientation;
	vis_model_.markers[1].pose.orientation = pose.orientation;
	vis_model_.markers[2].pose.orientation = pose.orientation;
	vis_model_.markers[3].pose.orientation = pose.orientation;
	vis_model_.markers[4].pose = pose;

	Eigen::Matrix3d rot = robot_pose_.block(0, 0, 3, 3);
	Eigen::Vector3d p(0.225, 0.225, -0.02);
	vector<Eigen::Vector3d> pl;
	pl.emplace_back(p);
	p(0) = -p(0);
	pl.emplace_back(p);
	p(1) = -p(1);
	pl.emplace_back(p);
	p(0) = -p(0);
	pl.emplace_back(p);

	for(int i = 0; i < 4; i++){
		p = rot * pl[i] + p_;
		vis_model_.markers[i].pose.position.x = p(0);
		vis_model_.markers[i].pose.position.y = p(1);
		vis_model_.markers[i].pose.position.z = p(2);
	}

}

#endif