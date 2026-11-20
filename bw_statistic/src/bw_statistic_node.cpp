// Bandwidth statistics for GVP: subscribes every drone's *_send topics and
// counts transmitted bytes per message kind, mirroring the FAME/RACER
// bw_statistic output format (per-channel value file + _t file, 1 Hz dump).
// The clock starts on the master trigger (remap /kick_start -> /takeoff).

#include "data_statistics/computation_statistician.h"

#include <ros/ros.h>

#include <exp_comm_msgs/MapC.h>
#include <exp_comm_msgs/MapReqC.h>
#include <exp_comm_msgs/SwarmJobC.h>
#include <exp_comm_msgs/SwarmStateC.h>
#include <swarm_exp_msgs/DtgBag.h>
#include <swarm_exp_msgs/IdPose.h>
#include <swarm_exp_msgs/SwarmTraj.h>

#include <std_msgs/Empty.h>

using namespace std;

int uav_num;
bool start_ = false;
double start_t_;

// channel map: 0 pose, 1 DTG, 2 traj, 3 job, 4 state, 5 map, 6 mapreq, 7 finish
vector<uint64_t> item_communication_volume_;

ComputationStatistician *Cs_;
ros::Timer record_timer_;
ros::Subscriber trigger_sub_;
ros::Publisher trigger_pub_;
vector<ros::Subscriber> subs_;

void Trigger(const std_msgs::EmptyConstPtr &msg) {
    if (start_) return;
    start_ = true;
    start_t_ = ros::WallTime::now().toSec();
    if (Cs_) Cs_->start_t_ = start_t_;
    ROS_WARN("bw stat triggered");
}

// Receiver-count convention (unicast model, matches GVPs original
// receive-side accounting): messages carrying to_uavs are charged per listed
// receiver; messages without a target field are broadcast, charged (N-1).

// [0] pose
void PoseSub(const swarm_exp_msgs::IdPoseConstPtr &msg) {
    if (!start_) return;
    uint64_t v = sizeof(*msg) + sizeof(uint8_t) * msg->to_uavs.size();
    item_communication_volume_[0] += v * std::max(1, (int)msg->to_uavs.size());
}

// [1] DTG
void DtgSub(const swarm_exp_msgs::DtgBagConstPtr &msg) {
    if (!start_) return;
    uint64_t v = sizeof(*msg);
    v += sizeof(swarm_exp_msgs::DtgHFEdge) * msg->HFedges.size();
    v += sizeof(swarm_exp_msgs::DtgHHEdge) * msg->HHedges.size();
    v += sizeof(swarm_exp_msgs::DtgHNode) * msg->Hnodes.size();
    v += sizeof(swarm_exp_msgs::DtgFNode) * msg->Fnodes.size();
    for (auto &hfe : msg->HFedges) v += sizeof(uint32_t) * hfe.points_idx.size();
    for (auto &hhe : msg->HHedges) v += sizeof(uint32_t) * hhe.points_idx.size();
    for (auto &fn : msg->Fnodes) v += sizeof(uint8_t) * fn.vp_flags.size();
    item_communication_volume_[1] += v * std::max(1, (int)msg->to_uavs.size());
}

// [2] traj
void TrajSub(const swarm_exp_msgs::SwarmTrajConstPtr &msg) {
    if (!start_) return;
    uint64_t v = sizeof(*msg) + sizeof(float) * msg->t_p.size() +
                 sizeof(geometry_msgs::Point) * msg->coef_p.size() +
                 sizeof(uint8_t) * msg->to_uavs.size();
    item_communication_volume_[2] += v * std::max(1, (int)msg->to_uavs.size());
}

// [3] job (no target field: broadcast to all other drones)
void JobSub(const exp_comm_msgs::SwarmJobCConstPtr &msg) {
    if (!start_) return;
    item_communication_volume_[3] += sizeof(*msg) * std::max(1, uav_num);  // other drones + ground station
}

// [4] state (broadcast)
void StateSub(const exp_comm_msgs::SwarmStateCConstPtr &msg) {
    if (!start_) return;
    uint64_t v = sizeof(*msg);
    v += sizeof(uint16_t) * msg->local_fn.size();
    v += sizeof(float) * msg->dist_to_local_fn.size();
    v += sizeof(uint16_t) * msg->connect_fn.size();
    v += sizeof(float) * msg->dist_to_connect_fn.size();
    v += sizeof(uint32_t) * msg->connect_hn.size();
    item_communication_volume_[4] += v * std::max(1, uav_num);  // other drones + ground station
}

// [5] map (broadcast; refine if the map protocol is unicast per requester)
void MapSub(const exp_comm_msgs::MapCConstPtr &msg) {
    if (!start_) return;
    uint64_t v = sizeof(*msg) + sizeof(uint8_t) * msg->flags.size();
    item_communication_volume_[5] += v * std::max(1, uav_num);  // other drones + ground station
}

// [6] mapreq
void MapReqSub(const exp_comm_msgs::MapReqCConstPtr &msg) {
    if (!start_) return;
    uint64_t v = sizeof(*msg) + sizeof(uint16_t) * msg->f_id.size() +
                 sizeof(uint8_t) * msg->block_id.size();
    item_communication_volume_[6] += v * std::max(1, uav_num);  // other drones + ground station
}

void RecordTimer(const ros::TimerEvent &e) {
    if (!start_ || !Cs_) return;
    for (int i = 0; i < (int)item_communication_volume_.size(); i++) {
        Cs_->SetVolume(item_communication_volume_[i], i);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bw_statistic");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    string ns = ros::this_node::getName();
    nh_private.param(ns + "/Exp/drone_num", uav_num, 1);
    bool statistic_;
    nh_private.param(ns + "/Exp/statistic", statistic_, false);
    cout << "bw drone num:" << uav_num << endl;
    if (statistic_) {
        // master trigger of this system (remapped from /kick_start)
        trigger_sub_ = nh.subscribe("/kick_start", 10, &Trigger);

        item_communication_volume_.resize(7, 0);
        subs_.reserve(uav_num + 8);
        // Outgoing swarm traffic lives on the shared /Communication bus
        // (murder_single_lite.launch remaps every drone_N/xxx_send onto it),
        // except mapreq_send which keeps its per-drone name.
        subs_.push_back(nh.subscribe("/Communication/pose_send", 100, &PoseSub));
        subs_.push_back(nh.subscribe("/Communication/DTG_send", 100, &DtgSub));
        subs_.push_back(nh.subscribe("/Communication/traj_send", 100, &TrajSub));
        subs_.push_back(nh.subscribe("/Communication/job_send", 100, &JobSub));
        subs_.push_back(nh.subscribe("/Communication/state_send", 100, &StateSub));
        subs_.push_back(nh.subscribe("/Communication/map_send", 1000, &MapSub));
        for (int id = 1; id <= uav_num; ++id) {
            subs_.push_back(
                nh.subscribe("/drone_" + to_string(id) + "/mapreq_send", 100, &MapReqSub));
        }

        record_timer_ = nh.createTimer(ros::Duration(1.0), &RecordTimer);

        Cs_ = new ComputationStatistician;
        Cs_->init(nh, nh_private);
    }
    // Master trigger publisher, same role as FAME/RACER: after 5 s, once the
    // subscribers (traj_exc takeoff, swarm_data clock, this node) are up,
    // publish one latched kick. The launch remaps /kick_start onto /takeoff.
    ros::Duration(15.0).sleep();
    trigger_pub_ = nh.advertise<std_msgs::Empty>("/kick_start", 100, true);
    std_msgs::Empty m;
    while (trigger_pub_.getNumSubscribers() <= uav_num*2 && ros::ok())
        ros::Duration(0.05).sleep();
    trigger_pub_.publish(m);
    ROS_WARN("pubpubpubpub");
    ros::spin();

    delete Cs_;
    return 0;
}
