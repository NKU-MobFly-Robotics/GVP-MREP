#ifndef COMPUTATION_STATISTICIAN_H_
#define COMPUTATION_STATISTICIAN_H_
#include <ros/ros.h>
#include<fstream>
#include<list>
#include<string>
#include<vector>
using namespace std;

class ComputationStatistician{
public:
    ComputationStatistician(){};
    ~ComputationStatistician(){
                    // oft_list_[i]<<"max:"<<max_list_[i]<<endl;
        double v = 0;
        for(int i = 0; i < vdata_size_ - 1; i++){
            if(i > 0) v += volume_list_[i];
            ofvdata_list_.back()<<vname_list_[i]+":"<<to_string(volume_list_[i])<<endl;
            ofvdata_list_[i].close();
		    ofvt_list_[i].close();
        }
        ofvdata_list_.back()<<to_string(v)<<endl;
        ofvt_list_.back().close();
        ofvdata_list_.back().close();


        for(int i = 0; i < data_size_; i++){
            // ofdata_list_[i]<<"max:"<<max_list_[i]<<endl;

            ofdata_list_[i].close();
		    oft_list_[i].close();
        }
    };
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private); 
    void StartTimer(int dataid);
    void EndTimer(int dataid);
    void AddVolume(double v, int id);
    void SetVolume(double v, int id);

    // void ShowTotalCost(int dataid);
    // void ShowCost(int dataid);
    // void ShowAllCost();
    string dir_;
private:
    double start_t_;
    vector<double> cost_list_;              //total cost
    vector<double> max_list_;
    vector<double> cost_temp_list_;       

    vector<double> volume_list_; 

    vector<ros::WallTime> starttime_list_;
    vector<std::string> name_list_;
    vector<std::string> vname_list_;

    vector<ofstream> ofdata_list_;
    vector<ofstream> oft_list_;

    vector<ofstream> ofvdata_list_;
    vector<ofstream> ofvt_list_;
    string path_;
    int data_size_;
    int vdata_size_;
    ros::NodeHandle nh_private_, nh_;
};

#endif