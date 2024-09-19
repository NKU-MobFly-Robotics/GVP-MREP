#include <multisim_gazebo/multi_rotors.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "block_test");
    ros::NodeHandle nh, nh_private("~");

    MultiRotors MRS_;
    MRS_.init(nh, nh_private);
    ros::spin();
    return 0;
}