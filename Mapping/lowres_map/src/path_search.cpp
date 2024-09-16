#include <lowres_map/lowres_map.h>
using namespace std;
namespace lowres{
bool LowResMap::GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, vector<Eigen::Vector3d> &path, bool local, int search_num){
    int workid;
    // ros::WallTime startT = ros::WallTime::now();
    while(!GetWorker(workid)){
        ros::Duration(1e-5).sleep();
    }
    // ROS_WARN("start a*");
    //  std::cout << "\033[0;34m GetWorker: \033[0m"<<(ros::WallTime::now() - startT).toSec() << std::endl;
    return Astar(start, end, path, workid, search_num, local);
    
}

bool LowResMap::GetPath(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &path, bool local, int search_num){
    int workid;
    // ros::WallTime startT = ros::WallTime::now();
    while(!GetWorker(workid)){
        ros::Duration(1e-5).sleep();
    }
    // ROS_WARN("start a*");
    //  std::cout << "\033[0;34m GetWorker: \033[0m"<<(ros::WallTime::now() - startT).toSec() << std::endl;
    return Astar(start, end, path, workid, search_num, local);
    
}

bool LowResMap::Astar(const Eigen::Vector3d &start, const Eigen::Vector3d &end, vector<Eigen::Vector3d> &path, const int &workid, int search_num, bool local){
    //check two points
    vector<shared_ptr<sch_node>> node_list; //to maintain nodes  
    if(local){
        if(!IsLocalFeasible(start) || !IsLocalFeasible(end)){
            FreeWorker(workid, node_list);
            cout<<IsLocalFeasible(start)<<" "<<IsLocalFeasible(end)<<endl;
            ROS_WARN("error IsLocalFeasible");
            return false;
        }
    }
    else{
        if(!IsFeasible(start) || !IsFeasible(end)){
            FreeWorker(workid, node_list);
            cout<<"A* feas"<<IsFeasible(start)<<" "<<IsFeasible(end)<<endl;
            ROS_WARN("error IsFeasible");
            return false;
        }
    }
    // priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, ACompare> empty_set;
    // open_set_[workid].swap(empty_set);
    shared_ptr<sch_node> c_node, ep_node;
    shared_ptr<LR_node> lr_node;

    //standarlize
    Eigen::Vector3i std_start, std_end;
    PostoId3(start, std_start);
    PostoId3(end, std_end);

    //init
    c_node = make_shared<sch_node>();
    c_node->pos_ = std_start;
    c_node->g_score_ = 0;
    c_node->f_score_ = c_node->g_score_ + lambda_heu_*GetHue(c_node->pos_, std_end);
    node_list.push_back(c_node);
    lr_node = GetNode(IdtoPos(c_node->pos_));
    if(lr_node == NULL || lr_node == Outnode_) ROS_WARN("ERROR Astar"); //debug
    lr_node->topo_sch_ = c_node;
    open_set_[workid].push(c_node);
    int search_iter = 0;
    while(!open_set_[workid].empty() && search_num > search_iter){
        ep_node = open_set_[workid].top();
        open_set_[workid].pop();

        if(ep_node->status_ == in_close) continue;/**/
        search_iter++;

        ep_node->status_ = in_close;
        //get end?
        if(std_end(0) - ep_node->pos_(0) == 0 && std_end(1) - ep_node->pos_(1) == 0 &&
            std_end(2) - ep_node->pos_(2) == 0){
            path.clear();
            path.push_back(end);
            RetrievePath(path, ep_node);
            path.push_back(start);
            reverse(path.begin(), path.end());
            FreeWorker(workid, node_list);
            // cout<<"search_iter:"<<search_iter<<endl;//debug
            return true;
        }
        //expand
        Eigen::Vector3i diff(0, 0, 0);
        for(int dim = 0; dim < 3; dim++){
            diff(0) = 0;
            diff(1) = 0;
            diff(2) = 0;
            for(int off = -1; off <= 1; off += 2){
                diff(dim) = off;
                    // if(x == 0 && y == 0 && z == 0) continue;    //the same node
                lr_node = GetNode(IdtoPos(diff+ep_node->pos_));
                if(lr_node == NULL || lr_node == Outnode_) continue;    //bad lrnode
                if(lr_node->flags_[0]) continue;
                c_node = lr_node->topo_sch_;
                
                if(c_node == NULL){                         //create a new node
                    c_node = make_shared<sch_node>();
                    c_node->pos_ = ep_node->pos_ + diff;
                    c_node->g_score_ = ep_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                    c_node->f_score_ = c_node->g_score_ + GetHue(c_node->pos_, std_end)*lambda_heu_;
                    c_node->parent_ = ep_node;
                    c_node->status_ = in_open;
                    lr_node->topo_sch_ = c_node;
                    node_list.push_back(c_node);
                    open_set_[workid].push(c_node);
                }
                else{                                       //new parent?
                    if(c_node->status_ == in_close) continue;

                    double g_tmp = ep_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                    double f_tmp = g_tmp + GetHue(c_node->pos_, std_end)*lambda_heu_*1.001;
                    if(g_tmp < c_node->g_score_){
                        c_node->status_ = in_close;
                        c_node = make_shared<sch_node>();/**/
                        lr_node->topo_sch_ = c_node; /**/
                        c_node->status_ = in_open;  /**/
                        c_node->pos_ = ep_node->pos_ + diff;/**/
                        c_node->f_score_ = g_tmp + GetHue(c_node->pos_, std_end)*lambda_heu_;
                        c_node->g_score_ = g_tmp;
                        c_node->parent_ = ep_node;
                        open_set_[workid].push(c_node); /**/
                    }
                }
            }
        }
    }
    if(search_iter >= search_num){
        cout<<"search_iter:"<<search_iter<<endl;
        ROS_WARN("too long");
    }
    else{
        ROS_WARN("empty!");
    }
    FreeWorker(workid, node_list);
    return false;
}


bool LowResMap::Astar(const Eigen::Vector3d &start, const Eigen::Vector3d &end, list<Eigen::Vector3d> &path, const int &workid, int search_num, bool local){
    //check two points
    vector<shared_ptr<sch_node>> node_list; //to maintain nodes  
    if(local){
        if(!IsLocalFeasible(start) || !IsLocalFeasible(end)){
            FreeWorker(workid, node_list);
            cout<<IsLocalFeasible(start)<<" "<<IsLocalFeasible(end)<<endl;
            ROS_WARN("error IsLocalFeasible");
            return false;
        }
    }
    else{
        if(!IsFeasible(start) || !IsFeasible(end)){
            FreeWorker(workid, node_list);
            cout<<IsFeasible(start)<<" "<<IsFeasible(end)<<endl;
            ROS_WARN("error IsFeasible");
            return false;
        }
    }
    // priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, ACompare> empty_set;
    // open_set_[workid].swap(empty_set);
    shared_ptr<sch_node> c_node, ep_node;
    shared_ptr<LR_node> lr_node;

    //standarlize
    Eigen::Vector3i std_start, std_end;
    PostoId3(start, std_start);
    PostoId3(end, std_end);

    //init
    c_node = make_shared<sch_node>();
    c_node->pos_ = std_start;
    c_node->g_score_ = 0;
    c_node->f_score_ = c_node->g_score_ + lambda_heu_*GetHue(c_node->pos_, std_end);
    node_list.push_back(c_node);
    lr_node = GetNode(IdtoPos(c_node->pos_));
    if(lr_node == NULL || lr_node == Outnode_) ROS_WARN("ERROR Astar"); //debug
    lr_node->topo_sch_ = c_node;
    open_set_[workid].push(c_node);
    int search_iter = 0;
    while(!open_set_[workid].empty() && search_num > search_iter){
        search_iter++;
        ep_node = open_set_[workid].top();
        open_set_[workid].pop();
        ep_node->status_ = in_close;
        //get end?
        if(std_end(0) - ep_node->pos_(0) == 0 && std_end(1) - ep_node->pos_(1) == 0 &&
            std_end(2) - ep_node->pos_(2) == 0){
            path.clear();
            path.push_back(end);
            RetrievePath(path, ep_node);
            path.push_front(start);
            FreeWorker(workid, node_list);
            // cout<<"search_iter:"<<search_iter<<endl;//debug
            return true;
        }
        //expand
        Eigen::Vector3i diff(0, 0, 0);
        for(int dim = 0; dim < 3; dim++){
            diff(0) = 0;
            diff(1) = 0;
            diff(2) = 0;
            for(int off = -1; off <= 1; off += 2){
                diff(dim) = off;
                    // if(x == 0 && y == 0 && z == 0) continue;    //the same node
                lr_node = GetNode(IdtoPos(diff+ep_node->pos_));
                if(lr_node == NULL || lr_node == Outnode_) continue;    //bad lrnode
                if(lr_node->flags_[0]) continue;
                c_node = lr_node->topo_sch_;
                
                if(c_node == NULL){                         //create a new node
                    c_node = make_shared<sch_node>();
                    c_node->pos_ = ep_node->pos_ + diff;
                    c_node->g_score_ = ep_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                    c_node->f_score_ = c_node->g_score_ + GetHue(c_node->pos_, std_end)*lambda_heu_;
                    c_node->parent_ = ep_node;
                    c_node->status_ = in_open;
                    lr_node->topo_sch_ = c_node;
                    node_list.push_back(c_node);
                    open_set_[workid].push(c_node);
                }
                else{                                       //new parent?
                    if(c_node->status_ == in_close) continue;
                    double g_tmp = ep_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                    double f_tmp = g_tmp + GetHue(c_node->pos_, std_end)*lambda_heu_;
                    if(g_tmp < c_node->g_score_){
                        c_node->f_score_ = f_tmp;
                        c_node->g_score_ = g_tmp;
                        c_node->parent_ = ep_node;
                    }
                }
            }
        }
    }
    if(search_iter >= search_num){
        cout<<"search_iter:"<<search_iter<<endl;
        ROS_WARN("too long");
    }
    else{
        ROS_WARN("empty!");
    }
    FreeWorker(workid, node_list);
    return false;
}

void LowResMap::Djkstra(int workid, Eigen::Vector3d start, list<Eigen::Vector3d> targets){
    target_dict t_dict;
    vector<shared_ptr<sch_node>> node_list; //to maintain nodes 
    list<vector<Eigen::MatrixXd>>::iterator ans_it = Inc_list_.begin();
    list<pair<int, Eigen::Vector3d>> check_list;
    for(int i = 0; i < workid; i++ ) ans_it++;
    ans_it->resize(targets.size());
    if(IsLocalFeasible(start)){
        int i = 0;
        for(list<Eigen::Vector3d>::iterator tar_it = targets.begin(); tar_it != targets.end(); tar_it++, i++){
            if(!IsLocalFeasible(*tar_it)){
                FreeWorker(workid, node_list);
                return;
            }
            check_list.push_back({i, (*tar_it)});
            Eigen::Vector3i tar_id;
            PostoId3(*tar_it, tar_id);
            int id = tar_id(2)*voxel_num_(0)*voxel_num_(1) + tar_id(1)*voxel_num_(0) + tar_id(0);
            if(t_dict.find(id) == t_dict.end())
                t_dict.insert({id, {i, tar_id}});
        }
    }
    else{
        FreeWorker(workid, node_list);
    }
    shared_ptr<sch_node> c_node, ep_node;
    shared_ptr<LR_node> lr_node;

    Eigen::Vector3i std_start, std_end;
    PostoId3(start, std_start);

    //init
    std_end = std_start;
    GetClosestTarget(t_dict, std_start, std_end);
    c_node = make_shared<sch_node>();
    c_node->pos_ = std_start;
    c_node->g_score_ = 0;
    c_node->f_score_ = c_node->g_score_;
    node_list.push_back(c_node);
    lr_node = GetNode(IdtoPos(c_node->pos_));

    if(lr_node == NULL || lr_node == Outnode_) ROS_WARN("ERROR Astar"); //debug
    lr_node->topo_sch_ = c_node;
    open_set_[workid].push(c_node);
    int search_iter = 0;//debug
    while(!open_set_[workid].empty()){
        search_iter++;//debug
        ep_node = open_set_[workid].top();
        open_set_[workid].pop();
        ep_node->status_ = in_close;
        //get end?
        int epid = ep_node->pos_(2)*voxel_num_(0)*voxel_num_(1) + ep_node->pos_(1)*voxel_num_(0) + ep_node->pos_(0);
        target_dict::iterator c_leaf = t_dict.find(epid);

        if(c_leaf != t_dict.end()){
            Eigen::MatrixXd path;
            RetrievePath(path, ep_node);
            for(list<pair<int, Eigen::Vector3d>>::iterator tar_it = check_list.begin(); tar_it != check_list.end(); tar_it++){
                Eigen::Vector3i tar_id;
                PostoId3(tar_it->second, tar_id);
                int id = tar_id(2)*voxel_num_(0)*voxel_num_(1) + tar_id(1)*voxel_num_(0) + tar_id(0);
                if(id == epid){
                    (*ans_it)[tar_it->first] = path;
                    (*ans_it)[tar_it->first].row(0) = start.transpose();
                    (*ans_it)[tar_it->first].bottomRows(1) = tar_it->second.transpose();
                    list<pair<int, Eigen::Vector3d>>::iterator erase_it = tar_it;
                    tar_it--;
                    check_list.erase(erase_it);
                }
            }
            t_dict.erase(c_leaf);
            if(t_dict.size() == 0) {
                FreeWorker(workid, node_list);
                cout<<"search_iter:"<<search_iter<<endl;
                return;
            }
        }

        //expand
        Eigen::Vector3i diff(0, 0, 0);
        for(int dim = 0; dim < 3; dim++){
            diff(0) = 0;
            diff(1) = 0;
            diff(2) = 0;
            for(int off = -1; off <= 1; off += 2){
                diff(dim) = off;
                    // if(x == 0 && y == 0 && z == 0) continue;    //the same node
                lr_node = GetNode(IdtoPos(diff+ep_node->pos_));
                if(lr_node == NULL || lr_node == Outnode_) continue;    //bad lrnode
                c_node = lr_node->topo_sch_;
                
                if(c_node == NULL){                         //create a new node
                    c_node = make_shared<sch_node>();
                    c_node->pos_ = ep_node->pos_ + diff;
                    c_node->g_score_ = ep_node->g_score_ + GetDistL1(diff);
                    c_node->f_score_ = c_node->g_score_;
                    c_node->parent_ = ep_node;
                    c_node->status_ = in_open;
                    lr_node->topo_sch_ = c_node;
                    node_list.push_back(c_node);
                    open_set_[workid].push(c_node);
                }
                else{                                       //new parent?
                    if(c_node->status_ == in_close) continue;
                    double g_tmp = ep_node->g_score_ + GetDistL1(diff);
                    double f_tmp = g_tmp;
                    if(g_tmp < c_node->g_score_){
                        c_node->f_score_ = f_tmp;
                        c_node->g_score_ = g_tmp;
                        c_node->parent_ = ep_node;
                    }
                }
            }
        }
    }
    FreeWorker(workid, node_list);
}

bool LowResMap::DjkstraLocalDist(const Eigen::Vector3d &start, list<list<Eigen::Vector3d>> &paths, 
                list<list<Eigen::Vector3d>> &pruned_paths, list<Eigen::Vector3d> &p_l, list<double> &d_l){
    d_l.clear();

    prio_D pr_l;
    list<list<Eigen::Vector3d>> prune_paths;
    list<Eigen::Vector3d> empty_path;
    shared_ptr<LR_node> lr_node;
    vector<shared_ptr<sch_node>> node_list;
    shared_ptr<sch_node> c_node, n_node;
    Eigen::Vector3d pos;
    pair<int, int> b_n_id;
    lr_node = GetNode(start);
    if(lr_node == NULL || lr_node == Outnode_ || lr_node->flags_[0]){
        return false;
    }
    lr_node->topo_sch_ = make_shared<sch_node>();
    node_list.emplace_back(lr_node->topo_sch_);
    PostoId3(start, lr_node->topo_sch_->pos_);
    lr_node->topo_sch_->status_ = in_open;
    pr_l.push(lr_node->topo_sch_);
    while(!pr_l.empty()){
        c_node = pr_l.top();
        pr_l.pop();
        if(c_node->status_ == in_close) continue;
        c_node->status_ = in_close;

        for(int dim = 0; dim < 3; dim++){
            Eigen::Vector3i diff(0, 0, 0);
            for(int off = -1; off <= 1; off += 2){
                diff(dim) = off;
                pos = IdtoPos(diff+c_node->pos_);
                lr_node = GetNode(pos);
                double g = c_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                if((pos - start).norm() > eu_range_ || g > max_g_cost_) continue;
                if(lr_node == NULL || lr_node->flags_[0] || lr_node == Outnode_) continue;
                if(NULL == lr_node->topo_sch_) {
                    lr_node->topo_sch_ = make_shared<sch_node>();
                    lr_node->topo_sch_->pos_ = c_node->pos_ + diff;
                    node_list.emplace_back(lr_node->topo_sch_);
                }
                n_node = lr_node->topo_sch_;
                if(n_node->status_ == in_close) continue;
                g = c_node->g_score_ + GetDist(diff(0), diff(1), diff(2)) * 1.0001;
                if(n_node->status_ == in_open && g < n_node->g_score_){
                    n_node->status_ = in_close;
                    lr_node->topo_sch_ = make_shared<sch_node>();
                    lr_node->topo_sch_->status_ = in_open;
                    lr_node->topo_sch_->pos_ = c_node->pos_ + diff;
                    lr_node->topo_sch_->g_score_ = c_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                    lr_node->topo_sch_->parent_ = c_node;
                }
                else if(n_node->status_ == not_expand){
                    n_node->status_ = in_open;
                    n_node->g_score_ = c_node->g_score_ + GetDist(diff(0), diff(1), diff(2));
                    n_node->parent_ = c_node;
                    pr_l.push(n_node);
                }
            }
        }
    }
    for(auto &p : p_l){
        lr_node = GetNode(p);
        if(lr_node == NULL  || lr_node->flags_[0] || lr_node == Outnode_){
            paths.push_back(empty_path);
            pruned_paths.push_back(empty_path);
            d_l.emplace_back(-2.0);
            continue;
        }
        if(lr_node->topo_sch_ == NULL) {
            paths.push_back(empty_path);
            pruned_paths.push_back(empty_path);
            d_l.emplace_back(-1.0);
            continue;
        }
        list<Eigen::Vector3d> path, path_prune;
        double length, length_f, length_e;
        RetrievePath(path, lr_node->topo_sch_);
        length_f = (path.back() - start).norm();
        length_e = (path.front() - p).norm();
        path.emplace_back(start);
        path.emplace_front(p);
        if(PrunePath(path, path_prune, length)){
            reverse(path.begin(), path.end());
            paths.emplace_back(path);
            prune_paths.emplace_back(path_prune);
            pruned_paths.push_back(path_prune);
            d_l.emplace_back(min(length, lr_node->topo_sch_->g_score_ + length_f + length_e));
        }
        else{
            ROS_WARN("fail prune????? how?");
            d_l.emplace_back(lr_node->topo_sch_->g_score_);
        }
    }
    Debug2(prune_paths);
    FreeWorker(0, node_list);
    return true;
}


void LowResMap::DjkstraLocal(Eigen::Vector3d start){
    prio_D pr_l;
    shared_ptr<LR_node> lr_node;
    shared_ptr<sch_node> c_node, n_node;
    Eigen::Vector3d pos;
    pair<int, int> b_n_id;
    lr_node = GetNode(start);
    if(lr_node->topo_sch_ == NULL || !lr_node->flags_[1] || lr_node == Outnode_){
        ROS_ERROR("DjkstraLocal0");
        ros::shutdown();
    }
    lr_node->topo_sch_->h_parent_ = NULL;
    lr_node->topo_sch_->h_status_ = in_open;
    lr_node->topo_sch_->h_g_score_ = 0;
    pr_l.push(lr_node->topo_sch_);
    while(!pr_l.empty()){
        c_node = pr_l.top();
        pr_l.pop();
        if(c_node->h_status_ == in_close) continue;
        pos = IdtoPos(c_node->pos_);
        b_n_id.first = GetBlockId(pos);
        // cout<<c_node->pos_.transpose()<<"  "<<b_n_id.first<<endl;
        b_n_id.second = GetNodeId(c_node->pos_, gridBLK_[b_n_id.first]);
        H_Topolist_.emplace_back(b_n_id);
        c_node->h_status_ = in_close;
        for(int dim = 0; dim < 3; dim++){
            Eigen::Vector3i diff(0, 0, 0);
            for(int off = -1; off <= 1; off += 2){
                // ROS_WARN("DjkstraLocal4");
                diff(dim) = off;
                pos = IdtoPos(diff+c_node->pos_);
                lr_node = GetNode(pos);
                if(lr_node == NULL || !lr_node->flags_[1] || lr_node == Outnode_ || lr_node->flags_[0]) continue;
                double g = c_node->h_g_score_ + GetDist(diff(0), diff(1), diff(2))*1.0001;
                n_node = lr_node->topo_sch_;
                if(n_node == NULL || n_node->h_status_ == in_close) continue;
                if(n_node->h_status_ == in_open && g < n_node->h_g_score_){
                    n_node->h_status_ = in_close;
                    lr_node->topo_sch_ = make_shared<sch_node>(n_node);
                    lr_node->topo_sch_->h_status_ = in_open;
                    lr_node->topo_sch_->h_g_score_ = c_node->h_g_score_ + GetDist(diff(0), diff(1), diff(2));
                    lr_node->topo_sch_->h_parent_ = c_node;
                    pr_l.push(lr_node->topo_sch_);
                }
                else if(n_node->h_status_ == not_expand){
                    n_node->h_status_ = in_open;
                    n_node->h_g_score_ = c_node->h_g_score_ + GetDist(diff(0), diff(1), diff(2));
                    n_node->h_parent_ = c_node;
                    pr_l.push(n_node);
                }
            }
        }
    }
}

void LowResMap::GetClosestTarget(LowResMap::target_dict &t_dict, Eigen::Vector3i std_start, Eigen::Vector3i &std_end){
    double min_dist = 9999;
    // Eigen::Vector3d best_end;
    for(target_dict::iterator tar_it = t_dict.begin(); tar_it != t_dict.end(); tar_it++){ 
        double dist = GetDistL1(std_end - tar_it->second.second);
        if(dist < min_dist){
            std_end = tar_it->second.second;
            min_dist = dist;
        }
    }
}

void LowResMap::ReSortOpenset(int &workid, Eigen::Vector3i &std_end){
    list<shared_ptr<sch_node>> node_list;
    while (!open_set_[workid].empty())
    {
        node_list.push_back(open_set_[workid].top());
        open_set_[workid].pop();
        node_list.back()->f_score_ = node_list.back()->g_score_ + GetHueL1(node_list.back()->pos_, std_end);
    }
    
    while (!node_list.empty())
    {
        open_set_[workid].push(node_list.back());
        node_list.pop_back();
    }
}

void LowResMap::RetrievePath(vector<Eigen::Vector3d> &path, shared_ptr<sch_node> &end_node){
    shared_ptr<sch_node> c_node = end_node;
    while(c_node->parent_ != NULL){
        path.push_back(IdtoPos(c_node->pos_));
        c_node = c_node->parent_;
    }
    path.push_back(IdtoPos(c_node->pos_));
}

void LowResMap::RetrievePath(list<Eigen::Vector3d> &path, shared_ptr<sch_node> &end_node){
    shared_ptr<sch_node> c_node = end_node;
    while(c_node->parent_ != NULL){
        path.push_back(IdtoPos(c_node->pos_));
        c_node = c_node->parent_;
    }
    path.push_back(IdtoPos(c_node->pos_));
}

void LowResMap::RetrievePath(Eigen::MatrixXd &path, shared_ptr<sch_node> &end_node){
    shared_ptr<sch_node> c_node = end_node;
    int path_size = 2;
    while(c_node->parent_ != NULL){
        path_size++;
        c_node = c_node->parent_;
    }
    path_size++;
    path.resize(path_size, 3);
    int node_id = path_size - 2;
    c_node = end_node;
    while(c_node->parent_ != NULL){
        path.row(node_id) = (IdtoPos(c_node->pos_)).transpose();
        c_node = c_node->parent_;
        node_id--;
    }
    path.row(node_id) = (IdtoPos(c_node->pos_)).transpose();

}

bool LowResMap::GetWorker(int &workid){
    Astar_mtx_.lock();
    for(int i = 0; i < 4; i++){
        if(Astar_worktable_[i]){
            Astar_worktable_[i] = false;
            workid = i;
            Astar_mtx_.unlock();
            return true;
        }
    }
    workable_ = false;
    Astar_mtx_.unlock();
    return true;
}

void LowResMap::FreeWorker(const int &workid, vector<shared_ptr<sch_node>> &nodelist){
    Astar_mtx_.lock();
    for(vector<shared_ptr<sch_node>>::iterator node_it = nodelist.begin(); node_it != nodelist.end(); node_it++){
        shared_ptr<LR_node> node = GetNode(IdtoPos((*node_it)->pos_));
        if(node != Outnode_ && node != NULL){
            node->topo_sch_ = NULL;
        }
        else{//debug
            cout<<(*node_it)->pos_.transpose()<<endl;
            cout<<(*node_it)->g_score_<<endl;
            ROS_WARN("ERROR FreeWorker");
        }
    }
    Astar_worktable_[workid] = true;
    priority_queue<shared_ptr<sch_node>, vector<shared_ptr<sch_node>>, ACompare> empty_set;
    open_set_[workid].swap(empty_set);
    Astar_mtx_.unlock();
}
}