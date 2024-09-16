#ifndef FASTCHECK_TRAJ_HPP
#define FASTCHECK_TRAJ_HPP
#include <Eigen/Eigen>

#include <iostream>
#include <vector>
#include "gcopter/trajectory.hpp"

class FastCheckTraj{
public:
    FastCheckTraj(){};
    ~FastCheckTraj(){};

    FastCheckTraj(Trajectory<5> &traj, const double &dt, const double &st){
        // std::cout<<"FastCheckTraj0"<<std::endl;
        dt_ = dt;
        double t = dt / 2 + st;
        Eigen::Vector3d pt;

        while(t < traj.getTotalDuration()){
            pt = traj.getPos(t);
            traj_pts_.emplace_back(pt);
            t += dt;
        }
        // std::cout<<"FastCheckTraj1"<<std::endl;
    }

    FastCheckTraj(const double &tt, const double &dt, Eigen::Vector3d &pos){
        dt_ = dt;
        double t = dt / 2;

        while(t < tt){
            traj_pts_.emplace_back(pos);
            t += dt;
        }
    }
    
    inline bool GetPos(const double &t, Eigen::Vector3d &pt){
        if(t < 0.0) return false;
        int id = floor(t / dt_);
        if(id >= traj_pts_.size() && traj_pts_.size() == 0) return false;
        else if(id >= traj_pts_.size()) pt = traj_pts_.back();
        else pt = traj_pts_[id];
        return true;
    }

    std::vector<Eigen::Vector3d> traj_pts_;
    double dt_;

};
#endif