#include "bridge_ros_tcp.h"
#include <glog/logging.h>

// -------------------- Helper --------------------

double getCoarseDelayTime(const ros::Time &msg_stamp) {
  return abs((ros::Time::now() - msg_stamp).toSec());
}

bool checkTolerableDelay(const double &msg_stamp) {
  return abs((ros::WallTime::now().toSec() - msg_stamp)) <= max_tolerable_delay_;
}

bool isBridgeMsgValidForDrones(const ros::Time &msg_stamp, const int msg_id) {
  return (msg_id == self_id_) &&
         (abs((ros::Time::now() - msg_stamp).toSec()) <= max_tolerable_delay_);
}

bool isBridgeMsgValidForGroundStation(const ros::Time &msg_stamp,
                                      const int msg_id) {
  int ind = remapGroundStationID(self_id_);
  return (msg_id == ind) &&
         (abs((ros::Time::now() - msg_stamp).toSec()) <= max_tolerable_delay_);
}

//
// -------------------- Send --------------------

template <typename T>
int sendToAllDrones(string topic, T &msg) {
  int err_code = 0;
  for (int i = 1; i < drone_num_+1; ++i) {
    if (i == self_id_in_bridge_) continue;  // skip self

    int err_code_i = bridge->sendMsgToOne(i, topic, msg);
    err_code += err_code_i;
    if (err_code_i < 0)
      ROS_ERROR("[Bridge] Send Error %s to Drone %d!", typeid(T).name(), i);
  }
  return err_code;
}

template <typename T>
int sendToAllGroundStations(string topic, T &msg) {
  int err_code = 0;
  for (int i = 0; i < ground_station_num_; ++i) {
    int ind = remapGroundStationID(i);
    if (ind == self_id_in_bridge_) continue;  // skip self

    int err_code_i = bridge->sendMsgToOne(ind, topic, msg);
    err_code += err_code_i;
    if (err_code_i < 0)
      ROS_ERROR("[Bridge] Send Error %s to Ground Station %d!",
                typeid(T).name(), ind);
  }
  return err_code;
}

//
// -------------------- Register Callback --------------------

void registerCallbackForAllDrones(
    string topic_name, function<void(int, ros::SerializedMessage &)> callback) {
  for (int i = 1; i < drone_num_+1; ++i) {
    if (i == self_id_in_bridge_) continue;

    bridge->registerCallback(i, topic_name, callback);
  }
}

void registerCallbackForAllGroundStations(
    string topic_name, function<void(int, ros::SerializedMessage &)> callback) {
  for (int i = 0; i < ground_station_num_; ++i) {
    int ind = remapGroundStationID(i);
    if (ind == self_id_in_bridge_) continue;

    bridge->registerCallback(ind, topic_name, callback);
  }
}

// ---------------------------------------------------------
// -------------------- Bridge Callback --------------------
// ---------------------------------------------------------

// ******************** Drones from Ground ********************
// void taskBridgeCallback(int ID, ros::SerializedMessage &m) {
//   if (is_groundstation_) return;

//   swarm_bridge_msgs::SwarmBool task_msg;
//   ros::serialization::deserializeMessage(m, task_msg);
//   if (isBridgeMsgValidForDrones(task_msg.header.stamp, task_msg.drone_id))
//     task_pub_.publish(task_msg.data);
// }

// void odomsBridgeCallback(int ID, ros::SerializedMessage &m){
//   if (is_groundstation_) return;
//   multisim_gazebo::states swarm_odoms_msg;
//   ros::serialization::deserializeMessage(m, swarm_odoms_msg);
//   if (checkTolerableDelay(swarm_odoms_msg.stamp))  //
//     swarm_odoms_pub_.publish(swarm_odoms_msg);
// }

void TrajBridgeCallback(int ID, ros::SerializedMessage &m){
	exp_comm_msgs::SwarmTrajC traj;
	ros::serialization::deserializeMessage(m, traj);
	if (checkTolerableDelay(traj.pub_t))
		swarm_traj_pub_.publish(traj);
}

void PoseBridgeCallback(int ID, ros::SerializedMessage &m){
	exp_comm_msgs::IdPoseC pose;
	ros::serialization::deserializeMessage(m, pose);
	if (checkTolerableDelay(pose.pub_t))
		swarm_pose_pub_.publish(pose);
}

void DTGBridgeCallback(int ID, ros::SerializedMessage &m){
	exp_comm_msgs::DtgBagC bag;
	ros::serialization::deserializeMessage(m, bag);
	if (checkTolerableDelay(bag.pub_t)){
  	swarm_DTG_pub_.publish(bag);
  }
}

void DTGAnsBridgeCallback(int ID, ros::SerializedMessage &m){
  exp_comm_msgs::DtgBagAnswer ans;
  // ROS_ERROR("get ans!");
	ros::serialization::deserializeMessage(m, ans);
  swarm_DTGans_pub_.publish(ans);
}

void TriggerBridgeCallback(int ID, ros::SerializedMessage &m){
	std_msgs::Empty msg;
	// ros::serialization::deserializeMessage(m, msg);
  for(int i = 0; i < 100; i++) ROS_WARN("takeoff");
  swarm_trigger_pub_.publish(msg);
}

void odomsBridgeCallback(int ID, ros::SerializedMessage &m){
  if (is_groundstation_) return;
  multisim_gazebo::states swarm_odoms_msg;
  ros::serialization::deserializeMessage(m, swarm_odoms_msg);

  if (checkTolerableDelay(swarm_odoms_msg.stamp))  //
    swarm_odoms_pub_.publish(swarm_odoms_msg);
}
// ------------------------------------------------------
// -------------------- ROS Callback --------------------
// ------------------------------------------------------

// ******************** Ground to Drones ********************

// void taskRosCallback(const swarm_bridge_msgs::SwarmBoolPtr &msg) {
//   if (!is_groundstation_) return;
//   bridge->sendMsgToOne(msg->drone_id, "/task", *msg);
// }

// void odomsRosCallback(const multisim_gazebo::statesPtr &msg){
//   if (is_groundstation_) return;
//   bridge->sendMsgToAll("/gazebo_odoms", *msg);
// }




void TrajRosCallback(const swarm_exp_msgs::SwarmTrajPtr &msg){
	exp_comm_msgs::SwarmTrajC traj;
	traj.coef_p = msg->coef_p;
	traj.order_p = msg->order_p;
	traj.pub_t = ros::WallTime::now().toSec();
	traj.start_t = msg->start_t;
	traj.t_p = msg->t_p;
  traj.id = self_id_;
	// traj.t_yaw = msg->t_yaw;
	// traj.order_yaw = msg->order_yaw;
	// traj.coef_yaw = msg->coef_yaw;
	for(auto &to_id : msg->to_uavs){
		bridge->sendMsgToOne(to_id, "/swarm_traj", traj);
	}
}

void PoseRosCallback(const swarm_exp_msgs::IdPosePtr &msg){
	exp_comm_msgs::IdPoseC pose;
	pose.id = msg->id;
	pose.pose = msg->pose;
	pose.pub_t = msg->pub_t;
	for(auto &to_id : msg->to_uavs){
		bridge->sendMsgToOne(to_id, "/swarm_pose", pose);
	}
}

void DTGRosCallback(const swarm_exp_msgs::DtgBagPtr &msg){
	exp_comm_msgs::DtgBagC bag;
	// bag.FFedges = msg->FFedges;
	bag.Fnodes = msg->Fnodes;
	bag.HFedges = msg->HFedges;
	bag.HHedges = msg->HHedges;
	bag.Hnodes = msg->Hnodes;
	bag.id = msg->id;
	bag.pub_t = ros::WallTime::now().toSec();
  bag.from_uav = self_id_;
	for(auto &to_id : msg->to_uavs){
	  // bridge->sendMsgToAll("/swarm_DTG", bag);
    bridge->sendMsgToOne(to_id, "/swarm_DTG", bag);
	  // bridge->sendMsgToAll("/swarm_DTG", bag);
	}
}

void DTGAnsRosCallback(const exp_comm_msgs::DtgBagAnswerPtr &msg){
  bridge->sendMsgToOne(msg->to_uav, "/swarm_DTGAns", *msg);
}

void TriggerRosCallback(const std_msgs::EmptyPtr &msg){
  swarm_trigger_pub_.publish(*msg);
  for(int i = 0; i < 100; i++) ROS_WARN("takeoff");
	bridge->sendMsgToAll("/swarm_takeoff", *msg);
}

void odomsRosCallback(const multisim_gazebo::statesPtr &msg){
  if (is_groundstation_) return;
  bridge->sendMsgToAll("/gazebo_odoms", *msg);
}
// ----------------------------------------------
// -------------------- Main --------------------
// ----------------------------------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_bridge_tcp");
  ros::NodeHandle nh("~");
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();

  nh.param("max_tolerable_delay", max_tolerable_delay_, 0.3);
  nh.param("self_id", self_id_, -1);
  nh.param("is_ground_station", is_groundstation_, false);
  nh.param("drone_num", drone_num_, 0);
  nh.param("ground_station_num", ground_station_num_, 0);

  // ******************** ID and IP list ********************
  id_list_.resize(drone_num_ + ground_station_num_);
  ip_list_.resize(drone_num_ + ground_station_num_);
  // set IP and ID in ros param
  for (int i = 1; i < 1+drone_num_; ++i) {
    if (i < drone_num_+1)
      nh.param("drone_ip_" + to_string(i), ip_list_[i-1], string("127.0.0.1"));
    else
      nh.param("ground_station_ip_" + to_string(i - drone_num_), ip_list_[i-1],
               string("127.0.0.1"));
    id_list_[i-1] = i;
  }

  // the ground station ID = self ID + drone_num_
  if (is_groundstation_)
    self_id_in_bridge_ = remapGroundStationID(self_id_);
  else
    self_id_in_bridge_ = self_id_;

  if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99) {
    ROS_WARN("[TCP Bridge]: Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  // initialize the bridge
  bridge.reset(
      new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));

  if (is_groundstation_) {
    // ---------- ROS -> Bridge ----------
    // Task
    // task_sub_ = nh.subscribe("task_start", 10, taskRosCallback,
    //                          ros::TransportHints().tcpNoDelay());
  } else {
		local_traj_sub_ = nh.subscribe("traj_send", 10, &TrajRosCallback);
		local_pose_sub_ = nh.subscribe("pose_send", 10, &PoseRosCallback);
		local_DTG_sub_ = nh.subscribe("DTG_send", 10, &DTGRosCallback);
  }
  local_DTGans_sub_ = nh.subscribe("DTGAns_send", 10, &DTGAnsRosCallback);
  swarm_trigger_sub_ = nh.subscribe("takeoff", 10, &TriggerRosCallback);
  local_odoms_sub_ = nh.subscribe("local_odoms", 1, odomsRosCallback);

	swarm_traj_pub_ = nh.advertise<exp_comm_msgs::SwarmTrajC>("traj_rec", 10);
  	registerCallbackForAllDrones("/swarm_traj", TrajBridgeCallback);

	swarm_pose_pub_ = nh.advertise<exp_comm_msgs::IdPoseC>("pose_rec", 10);
  	registerCallbackForAllDrones("/swarm_pose", PoseBridgeCallback);

	swarm_DTG_pub_ = nh.advertise<exp_comm_msgs::DtgBagC>("DTG_rec", 10);
	  registerCallbackForAllDrones("/swarm_DTG", DTGBridgeCallback);

  swarm_trigger_pub_ = nh.advertise<std_msgs::Empty>("takeoffself", 10);
	  registerCallbackForAllDrones("/swarm_takeoff", TriggerBridgeCallback);

  swarm_odoms_pub_ = nh.advertise<multisim_gazebo::states>(
      "swarm_odoms", 10);
  registerCallbackForAllDrones("/gazebo_odoms", odomsBridgeCallback);

  swarm_DTGans_pub_ = nh.advertise<exp_comm_msgs::DtgBagAnswer>(
      "DTGAns_rec", 10);
  registerCallbackForAllDrones("/swarm_DTGAns", DTGAnsBridgeCallback);
  ros::spin();
  bridge->StopThread();

  return 0;
}
