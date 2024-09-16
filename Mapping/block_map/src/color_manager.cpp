#include <block_map/color_manager.h>

void ColorManager::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    vector<double> CR, CB, CG;
    std::string ns = ros::this_node::getName();

    nh_private.param(ns + "/block_map/colorR", 
        CR, {});
    nh_private.param(ns + "/block_map/colorB", 
        CG, {});
    nh_private.param(ns + "/block_map/colorG", 
        CB, {});

    std_msgs::ColorRGBA color;
    for(int i = 0; i < CG.size(); i++){
        color.a = 1.0;
        cout<<CR[i]/255<<endl;
        color.r = CR[i]/255;
        color.g = CG[i]/255;
        color.b = CB[i]/255;
        c_l_.push_back(color);
    }
}