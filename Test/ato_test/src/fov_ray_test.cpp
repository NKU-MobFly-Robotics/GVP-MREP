#include <block_map/block_map.h>
#include <frontier_grid/frontier_grid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <glog/logging.h>

FrontierGrid *FG_;
int main(int argc, char** argv){
    ros::init(argc, argv, "fov_ray_test");
    ros::NodeHandle nh, nh_private("~");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InstallFailureSignalHandler();
    FrontierGrid FG;
    SwarmDataManager SDM_;

    BlockMap BM;
    BM.init(nh, nh_private);
    lowres::LowResMap LRM;

    ColorManager CM;
    CM.init(nh, nh_private);
    LRM.SetColorManager(&CM);

    LRM.SetMap(&BM);
    LRM.init(nh, nh_private);

    FG_ = &FG;
    FG_->SetMap(BM);
    FG_->SetLowresMap(LRM);
    FG_->SetSwarmDataManager(&SDM_);
    FG_->init(nh, nh_private);


    FG_->ShowGainDebug();
    ros::spin();
    return 0;
}