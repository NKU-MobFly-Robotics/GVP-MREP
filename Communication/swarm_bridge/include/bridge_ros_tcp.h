#ifndef SWARM_BRIDGE_ROS_TCP_H
#define SWARM_BRIDGE_ROS_TCP_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <multisim_gazebo/multi_rotors.h>

#include <exp_comm_msgs/DtgBagC.h>
#include <exp_comm_msgs/SwarmTrajC.h>
#include <exp_comm_msgs/IdPoseC.h>
#include <exp_comm_msgs/DtgBagAnswer.h>


#include <swarm_exp_msgs/IdPose.h>
#include <swarm_exp_msgs/DtgBag.h>
#include <swarm_exp_msgs/SwarmTraj.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "reliable_bridge.hpp"

// ----------------------------------------------------
// -------------------- Parameters --------------------
// ----------------------------------------------------
std::vector<int> id_list_;     // ID list
std::vector<string> ip_list_;  // IP list

int self_id_;             // Self ID in this type of device
int self_id_in_bridge_;   // Self ID in bridge (considering ground station)
int drone_num_;           // the number of drones
int ground_station_num_;  // the number of ground stations
bool is_groundstation_;   // whether this device is ground station
bool use_pose_;           // 0-use odom; 1-use pose

unique_ptr<ReliableBridge> bridge;  // msg - tcp bridge

// ---------- Ground - Drone ----------
ros::Subscriber task_sub_;  // Ground
ros::Publisher task_pub_;  // Drone

// ---------- Drone - Drone ----------
double max_tolerable_delay_;
ros::Publisher swarm_traj_pub_, swarm_pose_pub_, swarm_DTG_pub_, swarm_DTGans_pub_, swarm_trigger_pub_;
ros::Subscriber local_traj_sub_, local_pose_sub_, local_DTG_sub_, local_DTGans_sub_, swarm_trigger_sub_;
// ********** Simulaion **********
ros::Publisher swarm_odoms_pub_;
ros::Subscriber local_odoms_sub_;

// ---------------------------------------------------
// -------------------- Functions --------------------
// ---------------------------------------------------

//
// -------------------- Helper --------------------

/**
 * @brief Get the ID of ground station in bridge.
 *
 * @param id the index of the device in ground station list.
 * @return int: the id in all device (bridge).
 */
inline int remapGroundStationID(int id) {  //
  return id + drone_num_;
}

/**
 * @brief Get the delay time of message.
 * TODO: Due to the different devices, the time stamp is not synchronous.
 *
 * @param msg_stamp the stamp of the message.
 * @return double: the delay second.
 */
double getCoarseDelayTime(const ros::Time &msg_stamp);

/**
 * @brief Whether the delay is tolerable.
 * TODO: Due to the different devices, the time stamp is not synchronous.
 *
 * @param msg_stamp the stamp of the message.
 * @return true: the delay is tolerable.
 * @return false: the delay is not tolerable.
 */
bool checkTolerableDelay(const double &msg_stam);

/**
 * @brief Whether the msg is sent for self drone and delay is tolerable.
 * TODO: Due to the different devices, the time stamp is not synchronous.
 *
 * @param msg_stamp the stamp of the message.
 * @param msg_id the ID of the received message from bridge.
 * @return true: the msg is tolerable and used for self.
 * @return false: the msg is not used for self, or the msg is not tolerable.
 */
bool isBridgeMsgValidForDrones(const ros::Time &msg_stamp, const int msg_id);

/**
 * @brief Whether the msg is sent for self ground station and delay is
 * tolerable.
 * TODO: Due to the different devices, the time stamp is not synchronous.
 *
 * @param msg_stamp the stamp of the message.
 * @param msg_id the ID of the received message from bridge.
 * @return true: the msg is tolerable and used for self.
 * @return false: the msg is not used for self, or the msg is not tolerable.
 */
bool isBridgeMsgValidForGroundStation(const ros::Time &msg_stamp,
                                      const int msg_id);

//
// -------------------- Send --------------------

/**
 * @brief Send the message to all drones except self (no ground stations).
 *
 * @tparam T <message type>
 * @param topic: the name of topic in bridge.
 * @param msg: the ros message need to be sent.
 * @return int: 0 for success, -1 for queen full, -2 for no id.
 */
template <typename T>
int sendToAllDrones(string topic, T &msg);

/**
 * @brief Send the message to all ground station except self (no drones).
 *
 * @tparam T <message type>
 * @param topic: the name of topic
 * @param msg: the ros message need to be sent.
 * @return int: 0 for success, -1 for queen full, -2 for no id.
 */
template <typename T>
int sendToAllGroundStations(string topic, T &msg);

//
// -------------------- Register Callback --------------------

/**
 * @brief Register callback function for message from drones.
 *
 * @param topic_name: the name of topic
 * @param callback: callback function, like `void callbackSample(int ID,
 * ros::SerializedMessage& m) {std_msgs::Empty msg;
 * ros::serialization::deserializeMessage(m, msg);}`
 */
void registerCallbackForAllDrones(
    string topic_name, function<void(int, ros::SerializedMessage &)> callback);

/**
 * @brief Register callback function for message from ground stations.
 *
 * @param topic_name: the name of topic
 * @param callback: callback function, like `void callbackSample(int ID,
 * ros::SerializedMessage& m) {std_msgs::Empty msg;
 * ros::serialization::deserializeMessage(m, msg);}`
 */
void registerCallbackForAllGroundStations(
    string topic_name, function<void(int, ros::SerializedMessage &)> callback);

// ---------------------------------------------------------
// -------------------- Bridge Callback --------------------
// ---------------------------------------------------------

//
// ******************** Drones from Ground ********************

// /**
//  * @brief Bridge callback of Task Start for Drones subscribed from ground
//  * station. The subscribed message will check the coarse delay, and
//  * it will publish topic by task_pub_ (std_msgs::Bool).
//  *
//  * @param ID the device ID
//  * @param m the msg from bridge
//  */
// void taskBridgeCallback(int ID, ros::SerializedMessage &m);

// ------------------------------------------------------
// -------------------- ROS Callback --------------------
// ------------------------------------------------------

//
// ******************** Ground to Drones ********************

// /**
//  * @brief ROS callback of Task for ground station.
//  * It sends the task start to the Designated Drone by bridge in "/task"
//  * (swarm_bridge_msgs::SwarmPoseStamped).
//  *
//  * @param msg the msg from ROS
//  */
// void taskRosCallback(const swarm_bridge_msgs::SwarmBoolPtr &msg);

//
// ********** Drones to Drones **********

// void odomsBridgeCallback(int ID, ros::SerializedMessage &m);
// void odomsRosCallback(const multisim_gazebo::statesPtr &msg);
void TrajBridgeCallback(int ID, ros::SerializedMessage &m);
void PoseBridgeCallback(int ID, ros::SerializedMessage &m);
void DTGBridgeCallback(int ID, ros::SerializedMessage &m);
void DTGAnsBridgeCallback(int ID, ros::SerializedMessage &m);
void TriggerBridgeCallback(int ID, ros::SerializedMessage &m);


void TrajRosCallback(const swarm_exp_msgs::SwarmTrajPtr &msg);
void PoseRosCallback(const swarm_exp_msgs::IdPosePtr &msg);
void DTGRosCallback(const swarm_exp_msgs::DtgBagPtr &msg);
void DTGAnsRosCallback(const exp_comm_msgs::DtgBagAnswerPtr &msg);
void TriggerRosCallback(const std_msgs::EmptyPtr &msg);

/* MultiRotors Simulation */
void odomsBridgeCallback(int ID, ros::SerializedMessage &m);
void odomsRosCallback(const multisim_gazebo::statesPtr &msg);
#endif