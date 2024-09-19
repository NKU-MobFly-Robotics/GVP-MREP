#include <yaw_planner/yaw_planner.h>

void YawPlanner::init(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private){
    std::string ns = ros::this_node::getName();
    nh_private.param(ns + "/opt/YawVel", v_max_, 2.0);
    nh_private.param(ns + "/opt/YawAcc", a_max_, 2.0);
    safe_t_ = GetMinT(0.0, M_PI);
}

double YawPlanner::GetMinT(const double &yaw_start, const double &yaw_end){
    double dyaw = abs(Dyaw(yaw_end, yaw_start));
    double d_acc = v_max_ / a_max_ * 0.5 * v_max_;
    if(dyaw < 2 * d_acc){
        return sqrt(4 * dyaw / a_max_);
    }
    else{
        return 2 * v_max_ / a_max_ + (dyaw - 2 * d_acc) / v_max_;
    }
}

bool YawPlanner::Plan(const Eigen::VectorXd &yaw, Eigen::VectorXd &t, const double &vs, const double &as,
                 const double &ve, const double &ae){
    if(yaw.size() != t.size() + 1) return false;
    else{
        int m = t.size();
        A_.resize(yaw.size());
        T_ = t;
        Eigen::MatrixXd M(m * 6, m * 6);
        Eigen::VectorXd b(m*6);
        b.setZero();
        M.setZero();
        Eigen::MatrixXd F0t(3, 6);
        F0t<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 2.0, 0.0, 0.0, 0.0;
        M.block(0, 0, 3, 6) = F0t;
        // cout<<"M:\n"<<M<<endl;

        A_(0) = yaw(0);
        b.head(3)<<A_(0), vs, as;
        for(int i = 0; i < m; i++){
            double dyaw = Dyaw(yaw(i+1), A_(i));
            A_(i+1) = A_(i) + dyaw;
        }
        //F
        for(int i = 1; i < m; i++){
            b(i * 6 - 3) = A_(i);
            Eigen::MatrixXd Fit(6, 6);
            Fit << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, -2.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, -6.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, -24.0, 0.0;
            M.block(6 * (i-1) + 3, 6 * i, 6, 6) = Fit;
        }
        // cout<<"M:\n"<<M<<endl;

        //E
        for(int i = 1; i < m; i++){
            Eigen::MatrixXd Eit(6, 6);
            Eit << 1.0, T_(i-1), pow(T_(i-1), 2), pow(T_(i-1), 3), pow(T_(i-1), 4), pow(T_(i-1), 5),
            1.0, T_(i-1), pow(T_(i-1), 2), pow(T_(i-1), 3), pow(T_(i-1), 4), pow(T_(i-1), 5),
            0.0, 1.0, 2.0 * pow(T_(i-1), 1), 3.0 * pow(T_(i-1), 2), 4.0 * pow(T_(i-1), 3), 5.0 * pow(T_(i-1), 4),
            0.0, 0.0, 2.0, 3.0 * 2.0 * pow(T_(i-1), 1), 4.0 * 3.0 * pow(T_(i-1), 2), 5.0 * 4.0 * pow(T_(i-1), 3),
            0.0, 0.0, 0.0, 3.0 * 2.0, 4.0 * 3.0 * 2.0 * pow(T_(i-1), 1), 5.0 * 4.0 * 3.0 * pow(T_(i-1), 2),
            0.0, 0.0, 0.0, 0.0 * 0.0, 4.0 * 3.0 * 2.0, 5.0 * 4.0 * 3.0 * 2.0 * pow(T_(i-1), 1);
            M.block(6 * (i-1) + 3, 6 * i - 6, 6, 6) = Eit;
        }
        // cout<<"M:\n"<<M<<endl;

        Eigen::MatrixXd EMt(3, 6);
        EMt<<1.0, T_(m-1), pow(T_(m-1), 2), pow(T_(m-1), 3), pow(T_(m-1), 4), pow(T_(m-1), 5),
        0.0, 1.0, 2.0 * T_(m-1), 3.0 * pow(T_(m-1), 2), 4.0 * pow(T_(m-1), 3), 5.0 * pow(T_(m-1), 4),
        0.0, 0.0, 2.0, 3.0 * 2.0* pow(T_(m-1), 1), 4.0 * 3.0 * pow(T_(m-1), 2), 5.0 * 4.0 * pow(T_(m-1), 3);

        M.block(m*6 - 3, m*6 - 6, 3, 6) = EMt;
        // cout<<"M:\n"<<M<<endl;

        // b.head()
        b.tail(3)<<A_(m), ve, ae;
        Eigen::FullPivLU<Eigen::Ref<Eigen::MatrixXd>> lu(M);
        A_ = lu.solve(b);
        // cout<<"A_:"<<A_.transpose()<<endl;
        return true;
    }
}

void YawPlanner::GetCmd(const double &t, double &yaw_p, double &yaw_v, double &yaw_a){
    Eigen::VectorXd coef;
    double T;
    if(t >= T_.sum() - 1e-3){
        coef = A_.tail(6);
        T = T_.tail(1)(0);
        yaw_p = coef(0) + coef(1) * T + coef(2)*T*T + coef(3) * pow(T,3) + coef(4) * pow(T,4) + coef(5) * pow(T,5); 
        yaw_v = 0.0; 
        yaw_a = 0.0; 
    }
    else{
        double T_sum = T_(0);
        double T_bot = 0;
        int i = 0;
        while(t > T_sum){
            T_bot += T_(i);
            i++;
            T_sum += T_(i);
        }
        T = t - T_bot;
        coef = A_.block(6*i, 0, 6, 1);
        yaw_p = coef(0) + coef(1) * T + coef(2)*T*T + coef(3) * pow(T,3) + coef(4) * pow(T,4) + coef(5) * pow(T,5); 
        yaw_v = coef(1) + 2*coef(2)*T + 3*coef(3) * pow(T,2) + 4*coef(4) * pow(T,3) + 5*coef(5) * pow(T,4); 
        yaw_a = 2*coef(2) + 3*2*coef(3) * T + 4*3*coef(4) * pow(T,2) + 5*4*coef(5) * pow(T,3); 
    }
}

void YawPlanner::SampleT(const double &total_t, Eigen::VectorXd &t){
    int seg = ceil(total_t / safe_t_);
    t.resize(seg);
    for(int i = 0; i < seg; i++){
        t(i) = safe_t_;
    }
    t(0) = total_t - (seg-1) * safe_t_;
}

double YawPlanner::GetClosestYaw(const double &t, const double &yaw_s, const double &yaw_v, const double &yaw_t){
    if(GetMinT(yaw_s, yaw_t) <= t){
        return yaw_t;
    }
    else{
        double dyaw = Dyaw(yaw_t, yaw_s);
        double acc_t, vmax_t, d_acc;
        double dacc_t, dvmax_t, d_dacc;
        double a = a_max_ * 0.8;
        acc_t = abs((v_max_ - yaw_v) / a);
        vmax_t = t - acc_t;
        if(vmax_t > 0){
            d_acc = yaw_v * acc_t + 0.5 * a * acc_t * acc_t;
            d_acc += vmax_t * v_max_;
        }
        else{
            d_acc = yaw_v * t + 0.5 * a * t * t;
        }

        dacc_t = abs((-v_max_ - yaw_v) / a);
        dvmax_t = t - dacc_t;
        if(dvmax_t > 0){
            d_dacc = yaw_v * dacc_t - 0.5 * a * dacc_t * dacc_t;
            d_dacc -= dvmax_t * v_max_;
        }
        else{
            d_dacc = yaw_v * t - 0.5 * a * t * t;
        }
        if(d_dacc < dyaw && dyaw < d_acc){
            return yaw_t;
        }
        else{
            if(abs(dyaw - d_acc) < abs(dyaw - d_dacc)){
                return yaw_s + d_acc;
            }
            else 
                return yaw_s + d_dacc;
        }
        // if(t > (v_max_ - abs(yaw_v)) / a_max_){

        //     // if(abs(dyaw) < yaw_v * t + 0.5 * a_max_ * t * t)
        //     //     return 
        //     // return yaw_s + dyaw / abs(dyaw) * ((v_max_*v_max_/a_max_) + (t - v_max_ / a_max_*2) * v_max_);
        // }
        // else{
        //     return yaw_s + dyaw / abs(dyaw) * (a_max_*0.25*t*t);
        // }
    }
}