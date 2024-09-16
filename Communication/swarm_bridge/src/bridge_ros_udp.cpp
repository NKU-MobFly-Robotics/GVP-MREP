#include "bridge_ros_udp.h"

bool checkTolerableDelay(const ros::Time &msg_stamp) {
  return abs((ros::Time::now() - msg_stamp).toSec()) <= max_tolerable_delay_;
}

int initBroadcast(const char *ip, const int port) {
  int fd;
  // Creating socket file descriptor
  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {  // internet & udp
    ROS_ERROR("[UPD Bridge] Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  // Setting Socket
  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast,
                 sizeof(so_broadcast)) < 0) {  // setting Broadcast
    ROS_ERROR("[UPD Bridge] Error in setting Broadcast option");
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  // Convert ip from string to in_addr
  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0) {
    ROS_ERROR("[UPD Bridge] Invalid Address");
    return -1;
  }

  return fd;
}

int udpBindToPort(const char *ip, const int port, int &server_fd) {
  struct sockaddr_in address;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {  // internet & udp
    ROS_ERROR("[UPD Bridge] Socket receiver creation error!");
    exit(EXIT_FAILURE);
  }

  // Setting Socket
  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt,
                 sizeof(opt)) < 0) {  // setting reuse port and reuse addr
    ROS_ERROR("[UPD Bridge] Error in setting Reuse Addr and Reuse Port option");
    exit(EXIT_FAILURE);
  }

  address.sin_family = AF_INET;
  // address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Convert ip from string to in_addr
  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0) {
    ROS_ERROR("[UPD Bridge] Invalid Address");
    return -1;
  }

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    ROS_ERROR("[UPD Bridge] Bind Failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

template <typename T>
int serializeMessage(const MESSAGE_TYPE msg_type, const T &msg) {
  auto ptr = (uint8_t *)(udp_send_buf_);

  // serialize type
  *((MESSAGE_TYPE *)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  // serialize message
  uint32_t msg_size = ros::serialization::serializationLength(msg);

  *((uint32_t *)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  ros::serialization::OStream stream(ptr, msg_size);
  ros::serialization::serialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeMessage(T &msg) {
  auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  ros::serialization::IStream stream(ptr, msg_size);
  ros::serialization::deserialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

void odomRosCallback(const nav_msgs::OdometryPtr &msg) {
  if (use_pose_) return;

  ros::Time t_now = ros::Time::now();
  if (is_groundstation_ || (t_now - odom_stamp_).toSec() * odom_fre_ < 1.0)
    return;

  odom_stamp_ = t_now;
  swarm_bridge_msgs::SwarmOdometry udp_msg;
  udp_msg.drone_id = drone_id_;
  udp_msg.odometry = *msg;

  int len = serializeMessage(MESSAGE_TYPE::ODOM, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Odom from Drone %d error.", drone_id_);
  }
}

void poseRosCallback(const geometry_msgs::PoseStampedPtr &msg) {
  if (!use_pose_) return;

  ros::Time t_now = ros::Time::now();
  if (is_groundstation_ || (t_now - pose_stamp_).toSec() * pose_fre_ < 1.0)
    return;

  pose_stamp_ = t_now;
  swarm_bridge_msgs::SwarmPoseStamped udp_msg;
  udp_msg.drone_id = drone_id_;
  udp_msg.pose = *msg;

  int len = serializeMessage(MESSAGE_TYPE::POSE, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Pose from Drone %d error.", drone_id_);
  }
}

void stateRosCallback(const swarm_bridge_msgs::SwarmStatePtr &msg) {
  ros::Time t_now = ros::Time::now();
  if (is_groundstation_ || (t_now - state_stamp_).toSec() * state_fre_ < 1.0) {
    return;
  }
  state_stamp_ = t_now;
  swarm_bridge_msgs::SwarmState udp_msg;
  udp_msg = *msg;
  udp_msg.drone_id = drone_id_;

  int len = serializeMessage(MESSAGE_TYPE::STATE, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error State from Drone %d error.", drone_id_);
  }
}

void trajRosCallback(const communication_msgs::BsplinePtr &msg) {
  ros::Time t_now = ros::Time::now();
  if (is_groundstation_ || (t_now - traj_stamp_).toSec() * traj_fre_ < 1.0) {
    return;
  }
  traj_stamp_ = t_now;
  communication_msgs::Bspline udp_msg;
  udp_msg = *msg;
  udp_msg.drone_id = drone_id_;

  int len = serializeMessage(MESSAGE_TYPE::TRAJECTORY, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Trajectory from Drone %d error.", drone_id_);
  }
}

void goalRosCallback(const geometry_msgs::PoseStampedPtr &msg) {
  ros::Time t_now = ros::Time::now();
  if (is_groundstation_ || (t_now - goal_stamp_).toSec() * goal_fre_ < 1.0) {
    return;
  }
  goal_stamp_ = t_now;
  swarm_bridge_msgs::SwarmPoseStamped udp_msg;
  udp_msg.pose = *msg;
  udp_msg.drone_id = drone_id_;

  int len = serializeMessage(MESSAGE_TYPE::GOAL, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Goal from Drone %d error.", drone_id_);
  }
}

void mapRosCallback(const sensor_msgs::PointCloud2Ptr &msg) {
  ros::Time t_now = ros::Time::now();
  if (is_groundstation_ || (t_now - map_stamp_).toSec() * map_fre_ < 1.0) {
    return;
  }
  map_stamp_ = t_now;
  swarm_bridge_msgs::SwarmPointCloud2 udp_msg;
  udp_msg.pointcloud = *msg;
  udp_msg.drone_id = drone_id_;

  int len = serializeMessage(MESSAGE_TYPE::MAP, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Map from Drone %d error.", drone_id_);
  }
}

void imgRosCallback(const sensor_msgs::CompressedImagePtr &msg) {
  ros::Time t_now = ros::Time::now();
  if (!broadcast_img_ || is_groundstation_ ||
      (t_now - img_stamp_).toSec() * img_fre_ < 1.0) {
    return;
  }
  img_stamp_ = t_now;
  swarm_bridge_msgs::SwarmCompressedImage udp_msg;
  udp_msg.compressed_image = *msg;
  udp_msg.drone_id = drone_id_;

  int len = serializeMessage(MESSAGE_TYPE::IMAGE, udp_msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Image from Drone %d error.", drone_id_);
  }
}

void joyRosCallback(const swarm_bridge_msgs::SwarmJoyPtr &msg) {
  ros::Time t_now = ros::Time::now();
  if (!broadcast_joy_ || !is_groundstation_ ||
      (t_now - joy_stamp_).toSec() * joy_fre_ < 1.0) {
    return;
  }
  joy_stamp_ = t_now;
  // swarm_bridge_msgs::SwarmJoy udp_msg;
  // udp_msg.joy = *msg;
  // udp_msg.drone_id = drone_id_;

  int len = serializeMessage(MESSAGE_TYPE::JOY, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
             (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0) {
    ROS_ERROR("[UPD Bridge] Error Joy from Ground Station %d error.",
              groundstation_id_);
  }
}

void reqjoyRosCallback(const std_msgs::BoolPtr &msg) {
  if (is_groundstation_) broadcast_joy_ = msg->data;
}

void reqimgRosCallback(const std_msgs::BoolPtr &msg) {
  if (!is_groundstation_) broadcast_img_ = msg->data;
}

void udpReceiveThread() {
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len;

  // Connect
  if (udpBindToPort(udp_ip_.c_str(), UDP_PORT, udp_server_fd_) < 0) {
    ROS_ERROR("[UPD Bridge] Socket receiver creation error:");
    exit(EXIT_FAILURE);
  }

  while (true) {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0,
                            (struct sockaddr *)&addr_client,
                            (socklen_t *)&addr_len)) < 0) {
      perror("[UPD Bridge] receive error:");
      exit(EXIT_FAILURE);
    }
    char *ptr = udp_recv_buf_;
    if (is_groundstation_) {
      switch (*((MESSAGE_TYPE *)ptr)) {
        case MESSAGE_TYPE::ODOM: {
          swarm_bridge_msgs::SwarmOdometry odom_msg;
          if (valread == deserializeMessage(odom_msg))
            odom_pub_.publish(odom_msg);
          else
            ROS_ERROR("Received message length not matches the ODOM.");
          break;
        }
        case MESSAGE_TYPE::POSE: {
          swarm_bridge_msgs::SwarmPoseStamped pose_msg;
          if (valread == deserializeMessage(pose_msg))
            pose_pub_.publish(pose_msg);
          else
            ROS_ERROR("Received message length not matches the POSE.");
          break;
        }
        case MESSAGE_TYPE::STATE: {
          swarm_bridge_msgs::SwarmState state_msg;
          if (valread == deserializeMessage(state_msg))
            state_pub_.publish(state_msg);
          else
            ROS_ERROR("Received message length not matches the STATE.");
          break;
        }
        case MESSAGE_TYPE::TRAJECTORY: {
          communication_msgs::Bspline traj_msg;
          if (valread == deserializeMessage(traj_msg))
            traj_pub_.publish(traj_msg);
          else
            ROS_ERROR("Received message length not matches the TRAJECTORY.");
          break;
        }
        case MESSAGE_TYPE::GOAL: {
          swarm_bridge_msgs::SwarmPoseStamped goal_msg;
          if (valread == deserializeMessage(goal_msg))
            goal_pub_.publish(goal_msg);
          else
            ROS_ERROR("Received message length not matches the GOAL.");
          break;
        }
        case MESSAGE_TYPE::MAP: {
          swarm_bridge_msgs::SwarmPointCloud2 pointcloud_msg;
          if (valread == deserializeMessage(pointcloud_msg))
            map_pub_.publish(pointcloud_msg);
          else
            ROS_ERROR("Received message length not matches the MAP.");
          break;
        }
        case MESSAGE_TYPE::IMAGE: {
          swarm_bridge_msgs::SwarmCompressedImage img_msg;
          if (valread == deserializeMessage(img_msg))
            img_pub_.publish(img_msg);
          else
            ROS_ERROR("Received message length not matches the IMAGE.");
          break;
        }
        case MESSAGE_TYPE::JOY:
          break;

        default: {
          ROS_ERROR("Unknown received message type.");
          break;
        }
      }
    } else {
      switch (*((MESSAGE_TYPE *)ptr)) {
        case MESSAGE_TYPE::ODOM:
        case MESSAGE_TYPE::POSE:
        case MESSAGE_TYPE::STATE:
        case MESSAGE_TYPE::TRAJECTORY:
        case MESSAGE_TYPE::GOAL:
        case MESSAGE_TYPE::MAP:
        case MESSAGE_TYPE::IMAGE:
          break;
        case MESSAGE_TYPE::JOY: {
          swarm_bridge_msgs::SwarmJoy joy_msg;
          if (valread == deserializeMessage(joy_msg)) {
            if (joy_msg.drone_id == drone_id_) joy_pub_.publish(joy_msg.joy);
          } else
            ROS_ERROR("Received message length not matches the JOY.");
          break;
        }

        default: {
          ROS_ERROR("Unknown received message type.");
          break;
        }
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_bridge_udp");
  ros::NodeHandle nh("~");

  // 1 param list
  nh.param("is_ground_station", is_groundstation_, true);
  nh.param("max_tolerable_delay", max_tolerable_delay_, 0.3);
  nh.param("broadcast_ip", udp_ip_, string("127.0.0.255"));
  nh.param("use_pose", use_pose_, false);

  if (is_groundstation_) {
    nh.param("groundstation_id", groundstation_id_, -1);

    nh.param("joy_max_frequency", joy_fre_, 1000.0);

    joy_stamp_ = ros::Time::now();

    if (use_pose_)
      pose_pub_ =
          nh.advertise<swarm_bridge_msgs::SwarmPoseStamped>("swarm_poses", 10);
    else
      odom_pub_ =
          nh.advertise<swarm_bridge_msgs::SwarmOdometry>("swarm_odoms", 10);
    state_pub_ =
        nh.advertise<swarm_bridge_msgs::SwarmState>("swarm_states", 10);
    traj_pub_ = nh.advertise<communication_msgs::Bspline>("swarm_trajs", 10);
    goal_pub_ =
        nh.advertise<swarm_bridge_msgs::SwarmPoseStamped>("swarm_goals", 10);
    map_pub_ =
        nh.advertise<swarm_bridge_msgs::SwarmPointCloud2>("swarm_maps", 10);
    img_pub_ =
        nh.advertise<swarm_bridge_msgs::SwarmCompressedImage>("swarm_imgs", 10);

    req_joy_sub_ = nh.subscribe("req_joy", 10, reqjoyRosCallback,
                                ros::TransportHints().tcpNoDelay());
    joy_sub_ = nh.subscribe("joy", 10, joyRosCallback,
                            ros::TransportHints().tcpNoDelay());
  } else {
    nh.param("drone_id", drone_id_, -1);

    nh.param("odom_max_frequency", odom_fre_, 1000.0);
    nh.param("pose_max_frequency", pose_fre_, 1000.0);
    nh.param("state_max_frequency", state_fre_, 1000.0);
    nh.param("traj_max_frequency", traj_fre_, 1000.0);
    nh.param("image_max_frequency", img_fre_, 1000.0);
    nh.param("goal_max_frequency", goal_fre_, 1000.0);
    nh.param("map_max_frequency", map_fre_, 1000.0);

    odom_stamp_ = pose_stamp_ = state_stamp_ = traj_stamp_ = goal_stamp_ =
        map_stamp_ = img_stamp_ = ros::Time::now();

    joy_pub_ = nh.advertise<sensor_msgs::Joy>("drone_joy", 10);

    req_img_sub_ = nh.subscribe("req_img", 10, reqimgRosCallback,   
                                ros::TransportHints().tcpNoDelay());

    if (use_pose_)
      pose_sub_ = nh.subscribe("pose", 10, poseRosCallback,         //
                               ros::TransportHints().tcpNoDelay());
    else
      odom_sub_ = nh.subscribe("odom", 10, odomRosCallback,         //
                               ros::TransportHints().tcpNoDelay());
    state_sub_ = nh.subscribe("state", 10, stateRosCallback,        //
                              ros::TransportHints().tcpNoDelay());
    traj_sub_ = nh.subscribe("trajectory", 10, trajRosCallback,     //
                             ros::TransportHints().tcpNoDelay());
    goal_sub_ = nh.subscribe("goal", 10, goalRosCallback,           //
                             ros::TransportHints().tcpNoDelay());
    map_sub_ = nh.subscribe("map", 10, mapRosCallback,              //
                            ros::TransportHints().tcpNoDelay());
    img_sub_ = nh.subscribe("img", 10, imgRosCallback,              //
                            ros::TransportHints().tcpNoDelay());
  }

  // UDP Receiver
  boost::thread udp_recv_thd(udpReceiveThread);
  udp_recv_thd.detach();
  ros::Duration(0.1).sleep();

  // UDP Sender
  udp_send_fd_ = initBroadcast(udp_ip_.c_str(), UDP_PORT);

  ROS_INFO("[UDP Bridge] Start running");

  ros::spin();

  close(udp_server_fd_);
  close(udp_send_fd_);

  return 0;
}
