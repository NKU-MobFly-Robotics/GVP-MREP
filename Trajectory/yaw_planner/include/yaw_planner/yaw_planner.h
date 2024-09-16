#ifndef YAW_PLANNER_H_
#define YAW_PLANNER_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <list>
#include <memory>
#include <math.h>
using namespace std;
class YawPlanner{
public:
    void init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    double GetMinT(const double &yaw_start, const double &yaw_end);
    bool Plan(const Eigen::VectorXd &yaw, Eigen::VectorXd &t, const double &vs, const double &as,
                 const double &ve, const double &ae);
    void GetCmd(const double &t, double &yaw_p, double &yaw_v, double &yaw_a);
    void SampleT(const double &total_t, Eigen::VectorXd &t);
    double GetClosestYaw(const double &t, const double &yaw_s, const double &yaw_v, const double &yaw_t);

    /**
     * @brief 
     * 
     * @param yaw1 
     * @param yaw2 
     * @return get yaw distance [-PI, PI], yaw1 - yaw2
     */
    inline double Dyaw(const double &yaw1, const double &yaw2);
    inline double Normyaw(const double &yaw);
    Eigen::VectorXd A_;
    Eigen::VectorXd T_;
    double v_max_, a_max_;

private:

    double safe_t_;
};

inline double YawPlanner::Normyaw(const double &yaw){
    double yawn;
    int c = yaw / M_PI / 2;
    yawn = yaw - c * M_PI * 2;
    
    if(yawn < -M_PI) yawn += M_PI * 2;
    if(yawn > M_PI) yawn -= M_PI * 2;
    return yawn;
}

inline double YawPlanner::Dyaw(const double &yaw1, const double &yaw2){
    return Normyaw(yaw1 - yaw2);
}

#endif