#include <gcopter/traj_opt.h>

void AtoTraj::Init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    weightVec_.resize(5);
    upboundVec_.resize(4);
    string ns = ros::this_node::getName();
    nh_private.param(ns + "/opt/MaxVel", upboundVec_[0], 2.0);
    nh_private.param(ns + "/opt/MaxAcc", upboundVec_[1], 2.0);
    nh_private.param(ns + "/opt/MaxJer", upboundVec_[3], 15.0);
    nh_private.param(ns + "/opt/SwarmAvoid", upboundVec_[2], 1.0);
    nh_private.param(ns + "/opt/WeiPos", weightVec_[0], 1000.0);
    nh_private.param(ns + "/opt/WeiVel", weightVec_[1], 100.0);
    nh_private.param(ns + "/opt/WeiAcc", weightVec_[2], 100.0);
    nh_private.param(ns + "/opt/WeiJer", weightVec_[4], 50.0);
    nh_private.param(ns + "/opt/WeiSwarm", weightVec_[3], 100.0);
    nh_private.param(ns + "/opt/WeiT", weightT_, 20.0);
    nh_private.param(ns + "/opt/WeiminT", weight_minT_, 500.0);
    nh_private.param(ns + "/opt/smoothingEps", smoothingEps_, 0.01);
    nh_private.param(ns + "/opt/integralIntervs", integralIntervs_, 16);
    nh_private.param(ns + "/opt/RelCostTol", relCostTol_, 0.00001);
    // cout<<"max VVV:"<<upboundVec_[0]<<endl;
    swarm_traj_pub_ = nh.advertise<visualization_msgs::Marker>(ns+"/opt/debug", 10);
}

bool AtoTraj::Optimize(const vector<Eigen::Vector3d> &path,
                    const vector<Eigen::MatrixX4d> &corridors,
                    const vector<Eigen::Matrix3Xd> &corridorVs,
                    const double &min_t,
                    const Eigen::Matrix3d &initState,
                    const Eigen::Matrix3d &endState,
                    const int &self_id,
                    vector<Trajectory<5>> &trajs,
                    vector<double> &swarm_t,
                    vector<Eigen::Vector3d> &poses,
                    double start_t){

    gcopter::GCOPTER_PolytopeSFC gcopter;
    
    // magnitudeBounds = [v_max, a_max]^T
    // penaltyWeights = [pos_weight, vel_weight, acc_weight]^T
    // initialize some constraint parameters
    std::vector<FastCheckTraj> swarm_trajs;
    double dt = 0.05;
    // list<Eigen::Vector3d> debug_pts;
    for(int i = 0; i < trajs.size(); i++){
        if(i+1 == self_id) continue;
        if(trajs[i].getPieceNum() == 0){
            swarm_trajs.push_back(FastCheckTraj(10.0, dt, poses[i]));
        }

        if(trajs[i].getTotalDuration() + swarm_t[i] > start_t){
            double st = start_t - swarm_t[i];
            swarm_trajs.push_back(FastCheckTraj(trajs[i], dt, st));
        }
        else{
            // std::cout<<i<<"  --  "<<poses[i].transpose()<<std::endl;
            swarm_trajs.push_back(FastCheckTraj(10.0, dt, poses[i]));
        }

    }
    // Debug(debug_pts);

    Eigen::VectorXd magnitudeBounds(4);
    Eigen::VectorXd penaltyWeights(5);
    magnitudeBounds(0) = upboundVec_[0];
    magnitudeBounds(1) = upboundVec_[1];
    magnitudeBounds(2) = upboundVec_[2];
    magnitudeBounds(3) = upboundVec_[3];
    penaltyWeights(0) = weightVec_[0];
    penaltyWeights(1) = weightVec_[1];
    penaltyWeights(2) = weightVec_[2];
    penaltyWeights(3) = weightVec_[3];
    penaltyWeights(4) = weightVec_[4];

    vector<Eigen::Matrix3Xd> corridorVstemp;
    corridorVstemp.resize(corridorVs.size()*2 - 1);
    
    for(int i = 0; i < int(corridorVs.size()) - 1; i++){
        corridorVstemp[i*2 + 1].resize(3, 8);
        Eigen::Vector3d up_corn, down_corn;
        for(int dim = 0; dim < 3; dim++){
            down_corn(dim) = max(corridorVs[i](dim, 7), corridorVs[i + 1](dim, 7));
            up_corn(dim) = min(corridorVs[i](dim, 0), corridorVs[i + 1](dim, 0));
        }
        for(int dim1 = 0; dim1 <= 1; dim1++){
            for(int dim2 = 0; dim2 <= 1; dim2++){
                for(int dim3 = 0; dim3 <= 1; dim3++){
                    corridorVstemp[i*2 + 1](0, 4*dim3 + 2*dim2 + dim1) = dim1 ? up_corn(0) : down_corn(0);
                    corridorVstemp[i*2 + 1](1, 4*dim3 + 2*dim2 + dim1) = dim2 ? up_corn(1) : down_corn(1);
                    corridorVstemp[i*2 + 1](2, 4*dim3 + 2*dim2 + dim1) = dim3 ? up_corn(2) : down_corn(2);
                }
            }
        }
        for(int j = 1; j < 8; j++){
            corridorVstemp[i*2 + 1].col(j) = corridorVstemp[i*2 + 1].col(j) - corridorVstemp[i*2 + 1].col(0);
        }
        // corridorVstemp[i*2 + 1].col(0) = 
    }
    for(int i = 0; i < int(corridorVs.size()); i++){
        corridorVstemp[i*2] = corridorVs[i];
        Eigen::Vector3d start_p = corridorVstemp[i*2].col(7);
        for(int j = 1; j < 7; j++){
            corridorVstemp[i*2].col(7-j) = corridorVstemp[i*2].col(7-j) - corridorVstemp[i*2].col(7);
        }
        corridorVstemp[i*2].col(7) = corridorVstemp[i*2].col(0) - corridorVstemp[i*2].col(7);
        corridorVstemp[i*2].col(0) = start_p;
    }
    const int quadratureRes = integralIntervs_;
    Trajectory<5> traj_tempt = traj;    
    traj.clear();

    if (!gcopter.setup(weightT_, weight_minT_, min_t,
                        initState, endState,
                        corridors, corridorVstemp,
                        INFINITY,
                        smoothingEps_,
                        quadratureRes,
                        magnitudeBounds,
                        penaltyWeights,
                        swarm_trajs))
    {
        return false;
    }

    if (std::isinf(gcopter.optimize(traj, relCostTol_)))
    {
        ROS_ERROR("inf");
        cout<<"min_t:"<<min_t<<endl;
        cout<<"corridors:"<<corridors.size()<<endl;
        traj = traj_tempt;
        return false;
    }

    if (traj.getPieceNum() > 0)
    {
        trajStamp = ros::Time::now().toSec();
        return true;
    }
    else return false;
}

void AtoTraj::Debug(list<Eigen::Vector3d> &debug_list){
    visualization_msgs::Marker mk;
    mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = -2;
    mk.action = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;
    mk.color.a = 1.0;
    mk.color.r = 0.9;
    mk.color.g = 0.0;
    mk.color.b = 0.1;
    for(auto &pt : debug_list){
        geometry_msgs::Point p;
        p.x = pt(0);
        p.y = pt(1);
        p.z = pt(2);
        mk.points.emplace_back(p);
    }
    swarm_traj_pub_.publish(mk);
}
