#include "bridge_ros_tcp_sim.h"
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
  for (int i = 1; i < computer_num_+1; ++i) {
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
  for (int i = 1; i < computer_num_+1; ++i) {
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

void MapBridgeCallback(int ID, ros::SerializedMessage &m){
  exp_comm_msgs::MapC map;
	ros::serialization::deserializeMessage(m, map);
  swarm_map_pub_.publish(map);
}

void MapReqBridgeCallback(int ID, ros::SerializedMessage &m){
  exp_comm_msgs::MapReqC mapreq;
	ros::serialization::deserializeMessage(m, mapreq);
  swarm_mapreq_pub_.publish(mapreq);
}

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

void JobBridgeCallback(int ID, ros::SerializedMessage &m){
  exp_comm_msgs::SwarmJobC job;
	ros::serialization::deserializeMessage(m, job);
	if (checkTolerableDelay(job.pub_t)){
  	swarm_job_pub_.publish(job);
  }
}

void StateBridgeCallback(int ID, ros::SerializedMessage &m){
	exp_comm_msgs::SwarmStateC state;
	ros::serialization::deserializeMessage(m, state);
	if (checkTolerableDelay(state.pub_t)){
  	swarm_state_pub_.publish(state);
  }
}

void TriggerBridgeCallback(int ID, ros::SerializedMessage &m){
	std_msgs::Empty msg;
	// ros::serialization::deserializeMessage(m, msg);
  for(int i = 0; i < 100; i++) ROS_WARN("takeoff");
  swarm_trigger_pub_.publish(msg);
}

void odomsBridgeCallback(int ID, ros::SerializedMessage &m){
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


void MapRosCallback(const exp_comm_msgs::MapCPtr &msg){
  if(is_groundstation_){
    swarm_map_pub_.publish(*msg);
  }
  else{
    bridge->sendMsgToOne(ground_id_, "/swarm_map", *msg);
  }
}

void MapReqRosCallback(const exp_comm_msgs::MapReqCPtr &msg){
  if(is_groundstation_){
    bridge->sendMsgToAll("/swarm_mapreq", *msg);
    if(!computer_uav_.empty()) swarm_mapreq_pub_.publish(*msg);
  }
}

void TrajRosCallback(const swarm_exp_msgs::SwarmTrajPtr &msg){
	exp_comm_msgs::SwarmTrajC traj;
	traj.coef_p = msg->coef_p;
	traj.order_p = msg->order_p;
	traj.pub_t = ros::WallTime::now().toSec();
	traj.start_t = msg->start_t;
	traj.t_p = msg->t_p;
  traj.id = msg->from_uav;
	// traj.t_yaw = msg->t_yaw;
	// traj.order_yaw = msg->order_yaw;
	// traj.coef_yaw = msg->coef_yaw;
  list<uint8_t> to_computer;
  bool self_pub = false;
	for(auto &to_id : msg->to_uavs){
    for(int i = 0; i < computer_num_; i++){
      if(!self_pub && computer_uav_[i].first <= to_id && to_id <= computer_uav_[i].second){
        to_computer.emplace_back(i + 1);
        self_pub = true;
        break;
      }
    }
	}

  bool have_ground = false;
  for(auto &to_id : to_computer){
    if(to_id == ground_id_) {
      have_ground = true;
      break;
    }
  }
  if(!have_ground) to_computer.emplace_back(ground_id_);

  for(auto &to_id : to_computer){
    if(to_id == self_id_) swarm_traj_pub_.publish(traj);
    else bridge->sendMsgToOne(to_id, "/swarm_traj", traj);
  }
}

void PoseRosCallback(const swarm_exp_msgs::IdPosePtr &msg){
	exp_comm_msgs::IdPoseC pose;
	pose.id = msg->id;
	pose.pose = msg->pose;
	pose.pub_t = msg->pub_t;

  list<uint8_t> to_computer;
  bool self_pub = false;
	for(auto &to_id : msg->to_uavs){
    for(int i = 0; i < computer_num_; i++){
      if(!self_pub && computer_uav_[i].first <= to_id && to_id <= computer_uav_[i].second){
        to_computer.emplace_back(i + 1);
        self_pub = true;
        break;
      }
    }
	}

  bool have_ground = false;
  for(auto &to_id : to_computer){
    if(to_id == ground_id_) {
      have_ground = true;
      break;
    }
  }
  if(!have_ground) to_computer.emplace_back(ground_id_);

  for(auto &to_id : to_computer){
    if(to_id == self_id_) swarm_pose_pub_.publish(pose);
    else bridge->sendMsgToOne(to_id, "/swarm_pose", pose);
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
  bag.from_uav = msg->from_uav;

  list<uint8_t> to_computer;
  bool self_pub = false;
	for(auto &to_id : msg->to_uavs){
    for(int i = 0; i < computer_num_; i++){
      if(!self_pub && computer_uav_[i].first <= to_id && to_id <= computer_uav_[i].second){
        to_computer.emplace_back(i + 1);
        self_pub = true;
        break;
      }
    }
	}

  bool have_ground = false;
  for(auto &to_id : to_computer){
    if(to_id == ground_id_) {
      have_ground = true;
      break;
    }
  }
  if(!have_ground) to_computer.emplace_back(ground_id_);


  for(auto &to_id : to_computer){
    if(to_id == self_id_) {
      swarm_DTG_pub_.publish(bag);
    }
    else bridge->sendMsgToOne(to_id, "/swarm_DTG", bag);
  }
}

void DTGAnsRosCallback(const exp_comm_msgs::DtgBagAnswerPtr &msg){
  bridge->sendMsgToOne(msg->to_uav, "/swarm_DTGAns", *msg);
  for(int i = 0; i < computer_num_; i++){
    if(computer_uav_[i].first <= msg->to_uav && msg->to_uav <= computer_uav_[i].second){
      if(i + 1 == self_id_) swarm_DTGans_pub_.publish(*msg);
      else bridge->sendMsgToOne(i + 1, "/swarm_DTGAns", *msg);
      break;
    }
  }
}

void JobRosCallback(const exp_comm_msgs::SwarmJobCPtr &msg){
  bridge->sendMsgToAll("/swarm_job", *msg);
  swarm_job_pub_.publish(*msg);
}

void StateRosCallback(const exp_comm_msgs::SwarmStateCPtr &msg){
  bridge->sendMsgToAll("/swarm_state", *msg);
  swarm_state_pub_.publish(*msg);
}

void TriggerRosCallback(const std_msgs::EmptyPtr &msg){
  swarm_trigger_pub_.publish(*msg);
  for(int i = 0; i < 100; i++) ROS_WARN("takeoff");
	bridge->sendMsgToAll("/swarm_takeoff", *msg);
  swarm_traj_pub_.publish(*msg);
}

void odomsRosCallback(const multisim_gazebo::statesPtr &msg){
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
  nh.param("computer_num", computer_num_, 0);
  nh.param("ground_station_num", ground_station_num_, 0);
  nh.param("ground_id", ground_id_, 1);

  // ******************** ID and IP list ********************
  id_list_.resize(computer_num_ + ground_station_num_);
  ip_list_.resize(computer_num_ + ground_station_num_);
  computer_uav_.resize(computer_num_ + ground_station_num_);
  // set IP and ID in ros param
  int s_id = 1;
  for (int i = 1; i < 1+computer_num_; ++i) {
    int sim_num;
    if (i < computer_num_+1)
      nh.param("sim_ip_" + to_string(i), ip_list_[i-1], string("127.0.0.1"));
    // else
    //   nh.param("ground_station_ip_" + to_string(i - drone_num_), ip_list_[i-1],
    //            string("127.0.0.1"));
    nh.param("sim_num_" + to_string(i), sim_num, 0);
    computer_uav_[i-1].first = s_id;
    computer_uav_[i-1].second = max(s_id + sim_num - 1, s_id);
    s_id = max(s_id + sim_num - 1, s_id) + 1;
    id_list_[i-1] = i;
  }

  // the ground station ID = self ID + drone_num_
  // if (is_groundstation_)
  //   self_id_in_bridge_ = remapGroundStationID(self_id_);
  // else
  self_id_in_bridge_ = self_id_;

  if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99) {
    ROS_WARN("[TCP Bridge]: Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  // initialize the bridge
  bridge.reset(
      new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));

  if (is_groundstation_) {
    swarm_map_pub_ = nh.advertise<exp_comm_msgs::MapC>("map_rec", 1000);
  	registerCallbackForAllDrones("/swarm_map", MapBridgeCallback);

    local_mapreq_sub_ = nh.subscribe("mapreq_send", 100, &MapReqRosCallback);
  } 

  local_map_sub_ = nh.subscribe("map_send", 1000, &MapRosCallback);

  swarm_mapreq_pub_ = nh.advertise<exp_comm_msgs::MapReqC>("mapreq_rec", 10);
  registerCallbackForAllDrones("/swarm_mapreq", MapReqBridgeCallback);

  local_traj_sub_ = nh.subscribe("traj_send", 100, &TrajRosCallback);
	swarm_traj_pub_ = nh.advertise<exp_comm_msgs::SwarmTrajC>("traj_rec", 100);
  	registerCallbackForAllDrones("/swarm_traj", TrajBridgeCallback);

  local_pose_sub_ = nh.subscribe("pose_send", 100, &PoseRosCallback);
	swarm_pose_pub_ = nh.advertise<exp_comm_msgs::IdPoseC>("pose_rec", 100);
  	registerCallbackForAllDrones("/swarm_pose", PoseBridgeCallback);

  local_DTG_sub_ = nh.subscribe("DTG_send", 100, &DTGRosCallback);
	swarm_DTG_pub_ = nh.advertise<exp_comm_msgs::DtgBagC>("DTG_rec", 100);
	  registerCallbackForAllDrones("/swarm_DTG", DTGBridgeCallback);

  local_DTGans_sub_ = nh.subscribe("DTGAns_send", 100, &DTGAnsRosCallback);
  swarm_DTGans_pub_ = nh.advertise<exp_comm_msgs::DtgBagAnswer>(
      "DTGAns_rec", 10);
  registerCallbackForAllDrones("/swarm_DTGAns", DTGAnsBridgeCallback);

  local_job_sub_ = nh.subscribe("job_send", 100, &JobRosCallback);
  swarm_job_pub_ = nh.advertise<exp_comm_msgs::SwarmJobC>("job_rec", 100);
  registerCallbackForAllDrones("/swarm_job", JobBridgeCallback);

  local_state_sub_ = nh.subscribe("state_send", 100, &StateRosCallback);
  swarm_state_pub_ = nh.advertise<exp_comm_msgs::SwarmStateC>("state_rec", 100);
  registerCallbackForAllDrones("/swarm_state", StateBridgeCallback);

  local_trigger_sub_ = nh.subscribe("takeoff", 100, &TriggerRosCallback);
  swarm_trigger_pub_ = nh.advertise<std_msgs::Empty>("takeoffself", 100);
	  registerCallbackForAllDrones("/swarm_takeoff", TriggerBridgeCallback);

  local_odoms_sub_ = nh.subscribe("local_odoms", 100, odomsRosCallback);
  swarm_odoms_pub_ = nh.advertise<multisim_gazebo::states>(
      "swarm_odoms", 100);
  registerCallbackForAllDrones("/gazebo_odoms", odomsBridgeCallback);

  ros::spin();
  bridge->StopThread();

  return 0;
}
