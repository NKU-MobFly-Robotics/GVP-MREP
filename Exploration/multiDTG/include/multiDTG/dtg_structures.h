#ifndef DTG_STRUCTURES_H_
#define DTG_STRUCTURES_H_
#include <ros/ros.h>
#include <thread>
#include <Eigen/Eigen>
#include <vector>
#include <list>
#include <memory>
#include <queue>

#include <frontier_grid/frontier_grid.h>
using namespace std;

namespace DTG{
struct H_node;
struct F_node;
enum Fstate{
    l_active,
    g_active,
    uncoverable
};

enum Hstate{
    L_ACTIVE,
    G_ACTIVE,
    L_FREE,
    BLOCKED
};

template <typename HeadNode, typename TailNode>
struct DTG_edge{
    DTG_edge(){
        length_ = 2e5;
        length_s_ = 1e5;
        head_ = 0;
        tail_ = 0;
        // head_s_ = 0;
        // tail_s_ = 0;
        e_flag_ = 0;
    }
    DTG_edge(shared_ptr<HeadNode> &h, shared_ptr<TailNode> &t){
        head_ = h->id_;
        // head_s_ = h->id_;
        tail_ = t->id_;
        // tail_s_ = t->id_;
        head_n_ = h;
        // head_n_s_ = h;
        tail_n_ = t;
        // tail_n_s_ = t;

        length_ = 2e5;
        length_s_ = 1e5;
        e_flag_ = 0;
    }
    double length_, length_s_;
    uint8_t e_flag_;                     //00(have global)(have local) (fake edge)(???)(showed)(checked)
    uint32_t head_, tail_;
    // uint32_t head_swarm_, tail_swarm_;
    list<Eigen::Vector3d> path_;//, path_swarm_;    //head--->tail
    shared_ptr<HeadNode> head_n_;//, head_n_s_;
    shared_ptr<TailNode> tail_n_;//, tail_n_s_;
};
struct DTG_sch_node{
    DTG_sch_node(const float &g, const float &f, const uint32_t &id, const Eigen::Vector3d &p){
        g_ = g;
        f_ = f;
        id_ = id;
        pos_ = p;
        flag_ = 0;
        parent_ = NULL;
    }
    Eigen::Vector3d pos_;
    float g_, f_;
    uint32_t id_;
    uint8_t root_id_;
    uint8_t flag_;                      //0000 00(close)(h:1 f:0)
    shared_ptr<DTG_sch_node> parent_;
};

class ACompare {
public:
  bool operator()(shared_ptr<DTG_sch_node> node1, shared_ptr<DTG_sch_node> node2) {
    return node1->f_ > node2->f_;
  }
};

class DCompare {
public:
  bool operator()(shared_ptr<DTG_sch_node> node1, shared_ptr<DTG_sch_node> node2) {
    return node1->g_ > node2->g_;
  }
};

typedef priority_queue<shared_ptr<DTG_sch_node>, vector<shared_ptr<DTG_sch_node>>, DCompare> prio_D;
typedef priority_queue<shared_ptr<DTG_sch_node>, vector<shared_ptr<DTG_sch_node>>, ACompare> prio_A;

struct F_node{
    F_node(){
        f_flag_ = 0;
        vp_id_ = -1;
        exploring_id_ = 0;
    }
    uint32_t id_;
    int vp_id_;
    Eigen::Vector3d center_;
    CoarseFrontier *cf_;
    uint8_t exploring_id_;
    // Eigen::Vector3d upbd_, lowbd_;
    float g_, h_;
    shared_ptr<DTG_edge<H_node, F_node>> hf_edge_;
    list<shared_ptr<DTG_edge<F_node, F_node>>> edges_;
    // Fstate state_;
    uint8_t f_flag_;     //0000 0(local gvp send)(local gvp)0
    shared_ptr<DTG_sch_node> sch_node_;
};


struct H_node{
    H_node(){
        h_flags_ = 0;
        work_num_ = 0;
    }
    uint32_t id_;
    Eigen::Vector3d pos_;
    list<shared_ptr<DTG_edge<H_node, F_node>>> hf_edges_;
    list<shared_ptr<DTG_edge<H_node, H_node>>> hh_edges_;
    uint8_t work_num_;
    // uint8_t exploring_id_;
    // list<pair<uint8_t, float>> uav_dist_;
    Hstate state_;
    //for path search
    uint8_t h_flags_;     //0000 0(global gvp)(local gvp)(close)
    shared_ptr<DTG_sch_node> sch_node_;
};
}



#endif