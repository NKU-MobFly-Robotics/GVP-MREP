#ifndef SWARM_BRIDGE_ROS_UDP_H
#define SWARM_BRIDGE_ROS_UDP_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include "communication_msgs/Bspline.h"

#include "swarm_bridge_msgs/SwarmBool.h"
#include "swarm_bridge_msgs/SwarmCompressedImage.h"
#include "swarm_bridge_msgs/SwarmJoy.h"
#include "swarm_bridge_msgs/SwarmOdometry.h"
#include "swarm_bridge_msgs/SwarmPath.h"
#include "swarm_bridge_msgs/SwarmPointCloud2.h"
#include "swarm_bridge_msgs/SwarmPoseStamped.h"
#include "swarm_bridge_msgs/SwarmState.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <iostream>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#define UDP_PORT 8081
#define BUF_LEN 1048576     // 1MB
#define BUF_LEN_SHORT 1024  // 1KB

using namespace std;

// ----------------------------------------------------
// -------------------- Parameters --------------------
// ----------------------------------------------------

int udp_server_fd_, udp_send_fd_;  // recv and send file descriptor

string udp_ip_;
struct sockaddr_in addr_udp_send_;
char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];

enum MESSAGE_TYPE {
  ODOM = 100,
  POSE,
  STATE,
  TRAJECTORY,
  GOAL,
  MAP,
  IMAGE,
  JOY
} massage_type_;

bool broadcast_img_, broadcast_joy_;
double max_tolerable_delay_;
bool is_groundstation_;  // whether this device is ground station
int drone_id_;           // Self ID in this type of device (Drones)
int groundstation_id_;   // Self ID in this type of device (Ground Station)
bool use_pose_;          // 0-use odom; 1-use pose

// ---------- Drone -> Ground ----------
// Drone
ros::Subscriber req_img_sub_;
// Ground
ros::Subscriber req_joy_sub_;

// Drone
ros::Subscriber odom_sub_, pose_sub_, state_sub_, traj_sub_, goal_sub_,
    map_sub_, img_sub_;

// Ground
ros::Publisher odom_pub_, pose_pub_, state_pub_, traj_pub_, goal_pub_, map_pub_,
    img_pub_;

ros::Time odom_stamp_, pose_stamp_, state_stamp_, traj_stamp_, goal_stamp_,
    map_stamp_, img_stamp_;
double odom_fre_, pose_fre_, state_fre_, traj_fre_, img_fre_, goal_fre_,
    map_fre_;

// ---------- Ground -> Drone ----------
// Ground
ros::Subscriber joy_sub_;

// Drone
ros::Publisher joy_pub_;

ros::Time joy_stamp_;
double joy_fre_;

// ---------------------------------------------------
// -------------------- Functions --------------------
// ---------------------------------------------------

/**
 * @brief Initialize a UDP broadcast.
 *
 * @param ip the IP
 * @param port the Port
 * @return int: the file descriptor of the sender
 */
int initBroadcast(const char *ip, const int port);

/**
 * @brief Bind the UDP to port.
 *
 * @param ip the IP
 * @param port the Port
 * @param server_fd the file descriptor of the receiver
 * @return int: the file descriptor of the receiver
 */
int udpBindToPort(const char *ip, const int port, int &server_fd);

/**
 * @brief A thread of UDP receiver. It receives the message by UDP and publish
 * the ros message.
 * NOTE: the received message must be in MESSAGE_TYPE, otherwise it is ignored.
 *
 */
void udpReceiveThread();

/**
 * @brief Serialize the message and add it to udp_send_buf_
 *
 * @param msg_type the type of message
 * @param msg the message needed to be sent.
 * @return int: the bytes of message
 */
template <typename T>
int serializeMessage(const MESSAGE_TYPE msg_type, const T &msg);

/**
 * @brief Deserialize the message and add it to udp_send_buf_
 *
 * @param msg_type the type of message
 * @param msg the message needed to be sent.
 * @return int: the bytes of message
 */
template <typename T>
int deserializeMessage(T &msg);

/**
 * @brief The ROS callback for broadcasting Odom.
 * It marks the drone ID, check the frequency and broadcasts the message whose
 * type is swarm_bridge_msgs::SwarmOdometry.
 *
 * @param msg the message needed to be sent.
 */
void odomRosCallback(const nav_msgs::OdometryPtr &msg);

/**
 * @brief The ROS callback for broadcasting Pose.
 * It marks the drone ID, check the frequency and broadcasts the message whose
 * type is swarm_bridge_msgs::SwarmPoseStamped.
 *
 * @param msg the message needed to be sent.
 */
void poseRosCallback(const geometry_msgs::PoseStampedPtr &msg);

/**
 * @brief The ROS callback for broadcasting Drone State.
 * It marks the drone ID, check the frequency and broadcasts the message whose
 * type is swarm_bridge_msgs::SwarmState.
 *
 * @param msg the message needed to be sent.
 */
void stateRosCallback(const swarm_bridge_msgs::SwarmStatePtr &msg);

/**
 * @brief The ROS callback for broadcasting Trajectory.
 * It marks the drone ID, check the frequency and broadcasts the message whose
 * type is swarm_bridge_msgs::SwarmPath.
 *
 * @param msg the message needed to be sent.
 */
void trajRosCallback(const communication_msgs::BsplinePtr &msg);

/**
 * @brief The ROS callback for broadcasting Goal.
 * It marks the drone ID, check the frequency and broadcasts the message whose
 * type is swarm_bridge_msgs::SwarmPoseStamped.
 *
 * @param msg the message needed to be sent.
 */
void goalRosCallback(const geometry_msgs::PoseStampedPtr &msg);

/**
 * @brief The ROS callback for broadcasting Map.
 * It marks the drone ID, check the frequency and broadcasts the message whose
 * type is swarm_bridge_msgs::SwarmPointCloud2.
 *
 * @param msg the message needed to be sent.
 */
void mapRosCallback(const sensor_msgs::PointCloud2Ptr &msg);

/**
 * @brief The ROS callback for broadcasting Image.
 * When the image is required, it marks the drone ID, check the frequency and
 * broadcasts the message whose type is swarm_bridge_msgs::SwarmCompressedImage.
 *
 * @param msg the message needed to be sent.
 */
void imgRosCallback(const sensor_msgs::CompressedImagePtr &msg);

/**
 * @brief The ROS callback for broadcasting Joy.
 * When the joy is enable, it check the frequency and broadcasts the message
 * whose type is swarm_bridge_msgs::SwarmJoy.
 * NOTE: the subscribed message must preset the drone_id.
 *
 * @param msg the message needed to be sent.
 */
void joyRosCallback(const swarm_bridge_msgs::SwarmJoyPtr &msg);

/**
 * @brief The ROS callback for Joy Requirement.
 *
 * @param msg the message that determines whether publish joy.
 */
void reqjoyRosCallback(const std_msgs::BoolPtr &msg);

/**
 * @brief The ROS callback for Img Requirement.
 *
 * @param msg the message that determines whether publish img.
 */
void reqimgRosCallback(const std_msgs::BoolPtr &msg);

/**
 * @brief Whether the delay is tolerable.
 * TODO: Due to the different devices, the time stamp is not synchronous.
 *
 * @param msg_stamp the stamp of the message.
 * @return true: the delay is tolerable.
 * @return false: the delay is not tolerable.
 */
bool checkTolerableDelay(const ros::Time &msg_stamp);
#endif