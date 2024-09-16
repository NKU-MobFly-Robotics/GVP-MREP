#include "data_statistics/computation_statistician.h"


void ComputationStatistician::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
    std::string ns = ros::this_node::getName();
    nh_private_ = nh_private;
    int id, num;
    nh_private_.param(ns + "/Exp/UAV_id", 
        id, 1);
    nh_private_.param(ns + "/Exp/drone_num", 
        num, 1);
    nh_private_.param(ns + "/Computation/data_num", 
        data_size_, 1);
    nh_private_.param(ns + "/Computation/vdata_num", 
        vdata_size_, 1);
    nh_private_.param(ns + "/Computation/dir", dir_, dir_);
    // dir_ = dir_ + 
    cost_list_.resize(data_size_);              
    cost_temp_list_.resize(data_size_);         
    starttime_list_.resize(data_size_);
    max_list_.resize(data_size_);
    name_list_.resize(data_size_);
    ofdata_list_.resize(data_size_);
    oft_list_.resize(data_size_);

    volume_list_.resize(vdata_size_, 0);
    ofvdata_list_.resize(vdata_size_);
    ofvt_list_.resize(vdata_size_);
    vname_list_.resize(vdata_size_);

    time_t now = time(0);
    std::string Time_ = ctime(&now);
    tm* t=localtime(&now);
    string command;
    path_ = dir_+"/"+to_string(num)+"/"+to_string(t->tm_year+1900)+"_"+to_string(t->tm_mon+1)+"_"+to_string(t->tm_mday)
    +"_"+to_string(t->tm_hour)+"_"+to_string(t->tm_min)+"_"+to_string(t->tm_sec)+"_"+to_string(id);
    command = "mkdir "+path_;
    cout<<"command:"<<command<<endl;
    cout<<"path_:"<<path_<<endl;
    system(command.c_str());
    start_t_ = ros::WallTime::now().toSec();

    for(int i = 0; i < data_size_; i++){
        cost_list_[i] = 0;
        max_list_[i] = 0;
        cost_temp_list_[i] = 0;
        starttime_list_[i] = ros::WallTime::now();
        nh_private_.param(ns + "/Computation/data_name"+to_string(i), name_list_[i], name_list_[i]);
        ofdata_list_[i].open(path_+"/"+name_list_[i]+to_string(id)+".txt", std::ios::out);
        oft_list_[i].open(path_+"/"+name_list_[i]+to_string(id)+"_t"+".txt", std::ios::out);

    }
    for(int i = 0; i < vdata_size_; i++){
        nh_private_.param(ns + "/Computation/vdata_name"+to_string(i), vname_list_[i], vname_list_[i]);
        ofvdata_list_[i].open(path_+"/"+vname_list_[i]+to_string(id)+".txt", std::ios::out);
        ofvt_list_[i].open(path_+"/"+vname_list_[i]+"_t"+to_string(id)+".txt", std::ios::out);
    }
}

void ComputationStatistician::StartTimer(int dataid){
    if(dataid < 0 || dataid >= data_size_){
        printf("\033[0;31m ERROR dataid! dataid should be less than %d\033[0m\n", dataid);
        return;
    }
    starttime_list_[dataid] = ros::WallTime::now();
}

void ComputationStatistician::EndTimer(int dataid){
    if(dataid < 0 || dataid >= data_size_){
        printf("\033[0;31m ERROR dataid! The dataid should be less than %d\033[0m\n", data_size_);
        return;
    }
    double cost = (ros::WallTime::now() - starttime_list_[dataid]).toSec();
    double  t = ros::WallTime::now().toSec() - start_t_;
    if(cost > max_list_[dataid]) max_list_[dataid] = cost;
    cost_temp_list_[dataid] = cost;
    cost_list_[dataid] += cost;
    ofdata_list_[dataid]<<cost<<endl;
    oft_list_[dataid]<<t<<endl;
}


void ComputationStatistician::AddVolume(double v, int id){
    if(id < 0 || id >= vdata_size_){
        printf("\033[0;31m ERROR dataid! The dataid should be less than %d\033[0m\n", vdata_size_);
        return;
    }
    double  t = ros::WallTime::now().toSec() - start_t_;
    volume_list_[id] += v;
    ofvdata_list_[id]<<to_string(volume_list_[id])<<endl;
    ofvt_list_[id]<<t<<endl;
}

void ComputationStatistician::SetVolume(double v, int id){
    if(id < 0 || id >= vdata_size_){
        printf("\033[0;31m ERROR dataid! The dataid should be less than %d\033[0m\n", vdata_size_);
        return;
    }
    double  t = ros::WallTime::now().toSec() - start_t_;
    // ROS_ERROR("id:%d %d",id, vdata_size_);
    volume_list_[id] = v;
    ofvdata_list_[id]<<to_string(volume_list_[id])<<endl;
    ofvt_list_[id]<<t<<endl;
}
// void ComputationStatistician::ShowTotalCost(int dataid){
//     if(dataid < 0 || dataid >= data_size_){
//         printf("\033[0;31m ERROR dataid! The dataid should be less than %d\033[0m\n", data_size_);
//     }
//     printf("\033[0;32m %s spend %lfs\033[0m\n", name_list_[dataid].c_str(), cost_list_[dataid]);
// }

// void ComputationStatistician::ShowCost(int dataid){
//     if(dataid < 0 || dataid >= data_size_){
//         printf("\033[0;31m ERROR dataid! The dataid should be less than %d\033[0m\n", data_size_);
//     }
//     cost_temp_list_[dataid] = (starttime_list_[dataid] - ros::WallTime::now()).toSec();
//     printf("\033[0;32m %s spend %lfs\033[0m\n", name_list_[dataid].c_str(), cost_temp_list_[dataid]);
// }

// void ComputationStatistician::ShowAllCost(){
//     for(int i = 0; i < data_size_; i++){
//         printf("\033[0;32m %s spend %lfs\033[0m\n", name_list_[i].c_str(), cost_temp_list_[i]);
//     }
// }