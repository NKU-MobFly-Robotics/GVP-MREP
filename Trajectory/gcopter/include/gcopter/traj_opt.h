#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/fastcheck_traj.hpp"
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

using namespace std;
class AtoTraj{
public:
    void Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    bool Optimize(  const vector<Eigen::Vector3d> &path, 
                    const vector<Eigen::MatrixX4d> &corridors,
                    const vector<Eigen::Matrix3Xd> &corridorVs,
                    const double &min_t,
                    const Eigen::Matrix3d &initState,
                    const Eigen::Matrix3d &endState,
                    const int &self_id,
                    vector<Trajectory<5>> &trajs,
                    vector<double> &swarm_t,
                    vector<Eigen::Vector3d> &poses,
                    double start_t);
                                             
    Trajectory<5> traj;    
    vector<double> weightVec_; //[pos, vel, acc, avoidT]
    vector<double> upboundVec_; //[maxvel, maxacc, swarm]
private:
    void Debug(list<Eigen::Vector3d> &debug_list);
    ros::Publisher swarm_traj_pub_;
    ros::NodeHandle nh_, nh_private_;
    double smoothingEps_;
    double weightT_, weight_minT_;
    int integralIntervs_;

    double trajlength_;

    double relCostTol_;

    double trajStamp;



};

