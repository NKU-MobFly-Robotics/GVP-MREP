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
    ComputationStatistician(): data_size_(0), vdata_size_(0){};
    ~ComputationStatistician(){
        // 修复:原实现按 data_size_/vdata_size_ 下标索引 ofstream 向量,但这两个计数
        // 在未调用 init 时是未初始化垃圾值(向量可能为空)——空向量越界读出垃圾对象
        // 后 close() 必然段错误(实测 ~SwarmDataManager 崩溃)。改为按向量实际尺寸关闭。
        for(size_t i = 0; i + 1 < ofvdata_list_.size() && i + 1 < ofvt_list_.size(); i++){
            ofvdata_list_[i].close();
            ofvt_list_[i].close();
        }
        if(!ofvt_list_.empty()) ofvt_list_.back().close();
        if(!ofvdata_list_.empty()) ofvdata_list_.back().close();

        for(size_t i = 0; i < ofdata_list_.size() && i < oft_list_.size(); i++){
            ofdata_list_[i].close();
            oft_list_[i].close();
        }
    };
    void init(ros::NodeHandle &nh, ros::NodeHandle &nh_private); 
    void init(int id, int num, int vdata_size, string dir, vector<std::string> vname_list); 
    void StartTimer(int dataid);
    void EndTimer(int dataid);
    void AddVolume(double v, int id);
    void SetVolume(double v, int id);
    void SetVolume(uint64_t v, int id);

    // void ShowTotalCost(int dataid);
    // void ShowCost(int dataid);
    // void ShowAllCost();
    string dir_;
    double start_t_;
private:
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