//
// Created by ubuntu on 24-10-25.
//
#include <algorithm>
#include "matplotlibcpp.h"
#include "KinematicModel.h"
#include <vector>
#include <cmath>


using namespace std;
namespace plt = matplotlibcpp;
#define PI 3.1415926

/**
 * find the index of the point on the reference path which is closest to robot_state
 * @param robot_state robot state（x,y）
 * @param refer_path  reference path
 * @return the index of the closest point
 */
double calTargetIndex(const vector<double>&robot_state, const vector<vector<double>>&refer_path){
    vector<double>dists;
    for (vector<double>xy:refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
}

/**
 *
 * @return the cloest point on refer path depend on l_d
 */
double calTargetIndex_pp(const vector<double>& robot_state, const vector<vector<double>>& refer_path, double ld) {
    vector<double> dists;
    double robot_x = robot_state[0];
    double robot_y = robot_state[1];

    for (size_t i = 0; i < refer_path.size(); i++) {
        double path_x = refer_path[i][0];
        double path_y = refer_path[i][1];
        double dx = path_x - robot_x;
        double dy = path_y - robot_y;
        double dist = sqrt(dx * dx + dy * dy);

        // 判断路径点是否在前方
        if (dx >= 0 && dist >= ld) {  // 如果 dx >= 0 则点在前方
            dists.push_back(dist);
        } else {
            dists.push_back(std::numeric_limits<double>::max()); // 忽略后方点
        }
    }

    // 找到最接近预瞄距离的点的索引
    return min_element(dists.begin(), dists.end()) - dists.begin();
}


#define EPS 1.0e-4
MatrixXd CalRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) {
    MatrixXd Qf= Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for(int i=0;i<100;i++){
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        if((P_-P).maxCoeff()<EPS&&(P-P_).maxCoeff()<EPS)break;
        P = P_;
    }
    return P_;
}

vector<double> PIDController(const vector<double>& robot_state, const vector<vector<double>>&refer_path,
                             const KinematicModel& ugv) {
    double Kp = 0.5;
    double Ki = 0;
    double Kd = 12;
    static double sum_error = 0;
    static double pre_error = 0;
    std :: cout << "robot state 1" << robot_state[1] << std::endl;
    std :: cout << "robot state 0" << robot_state[0] << std::endl;

    double min_ind = calTargetIndex(robot_state,refer_path);
    double alpha = atan2(refer_path[min_ind][1]-robot_state[1], refer_path[min_ind][0]-robot_state[0]);
    double l_d = sqrt(pow(refer_path[min_ind][0]-robot_state[0],2)+pow(refer_path[min_ind][1]-robot_state[1],2));
    double theta_e = alpha-ugv.psi;
    double e_y = -l_d*sin(theta_e);

    double error = 0 - e_y;
    double u = error * Kp + sum_error * Ki + (error - pre_error) * Kd;
    u = u > PI/6 ? PI/6 : u;
    u = u < -PI/6 ? -PI/6 : u;
    pre_error = error;
    sum_error += error;

    double delta_f = u;
    vector<double> output;
    output.emplace_back(2); // constant velocity 2m/s
    output.emplace_back(delta_f); //steering angle
    return output;
}

#include <cmath>




vector<double> PurePursuitController(const vector<double>& robot_state, const vector<vector<double>>&refer_path,
                                     const KinematicModel& ugv){
    vector<double> output;
    // Your code
    double l_d =1; //预瞄距离
    double min_ind = calTargetIndex_pp(robot_state,refer_path, l_d);
    std::cout<< "min" << min_ind << std::endl;
    double e_ld;
    double dx = refer_path[min_ind][0] - robot_state[0];
    double dy = refer_path[min_ind][1] - robot_state[1];
    double alpha = atan2(dy , dx);
    double theta_e = alpha - ugv.psi;
    e_ld = sqrt(pow(dy,2)+pow(dx,2) )* sin (theta_e);
    double delta_f = atan(2 * ugv.L * e_ld / pow(l_d,2) );





    output.emplace_back(2);
    output.emplace_back(delta_f);
    return output;
}


vector<double> StanlyController(const vector<double>& robot_state, const vector<vector<double>>&refer_path,
                                const KinematicModel& ugv){
    vector<double> output;
    // Your code
    double k = 0.15;
    double min_ind = calTargetIndex(robot_state,refer_path);
    double d = 2/k;
    double yaw = ugv.psi;
    double f_x = robot_state[0] + ugv.L * cos(yaw);
    double f_y = robot_state[1] + ugv.L * sin(yaw);
    double dx =refer_path[min_ind][0]-f_x;
    double dy =refer_path[min_ind][1]-f_y;
    double e_y = (dy * cos(yaw) - dx * sin(yaw)) > 0 ? sqrt(dx * dx + dy * dy) : -sqrt(dx * dx + dy * dy);
    double path_yaw;
    if (min_ind < refer_path.size() - 1) {

        double next_dx = refer_path[min_ind + 1][0] - refer_path[min_ind][0];
        double next_dy = refer_path[min_ind + 1][1] - refer_path[min_ind][1];
        path_yaw = atan2(next_dy, next_dx);
    } else {

        double prev_dx = refer_path[min_ind][0] - refer_path[min_ind - 1][0];
        double prev_dy = refer_path[min_ind][1] - refer_path[min_ind - 1][1];
        path_yaw = atan2(prev_dy, prev_dx);
    }
    double yaw_error = path_yaw - yaw;
    while (yaw_error > PI) yaw_error -= 2.0 * PI;
    while (yaw_error < -PI) yaw_error += 2.0 * PI;
    double theta_e = yaw_error;
    double theta_d = atan2(k * e_y, 2);
    double delta_f = theta_e + theta_d;
    output.emplace_back(2);
    output.emplace_back(delta_f);
    return output;
}

vector<double> LQRController(const vector<double>& robot_state, const vector<vector<double>>& refer_path, KinematicModel& ugv) {
    vector<double> output;


    int min_ind = calTargetIndex(robot_state, refer_path);
    double dx, dy, ddx, ddy;
    if (min_ind ==0) {
        dx = refer_path[min_ind+1][0] - refer_path[min_ind][0];
        dy = refer_path[min_ind+1][1] - refer_path[min_ind][1];
        ddx = refer_path[min_ind + 2][0] + refer_path[min_ind][0]-2 * refer_path[min_ind+1][0];
        ddy = refer_path[min_ind + 2][1] + refer_path[min_ind][1] -2* refer_path[min_ind+1][1];
    } else if ( min_ind == refer_path.size() - 1) {
        dx = refer_path[min_ind][0] - refer_path[min_ind-1][0];
        dy = refer_path[min_ind][1] - refer_path[min_ind-1][1];
        ddx = refer_path[min_ind][0] + refer_path[min_ind-2][0] -2* refer_path[min_ind-1][0];
        ddy = refer_path[min_ind][1] + refer_path[min_ind-2][1] -2* refer_path[min_ind-1][1];
    } else {
        dx = refer_path[min_ind +1][0] - refer_path[min_ind][0];
        dy = refer_path[min_ind+ 1][1] - refer_path[min_ind][1];
        ddx = refer_path[min_ind +1][0] + refer_path[min_ind -1][0] -2*refer_path[min_ind][0];
        ddy = refer_path[min_ind +1][1] + refer_path[min_ind -1][1] -2*refer_path[min_ind][1];
    }
    double c = atan2( dy, dx);
    double k = ( ddy * dx - ddx * dy) /  pow((dx *dx + dy*dy), 3.0/2);
    double ref_delta = atan2(ugv.L * c,1);
    double ref_yaw = k;

    // double ref_delta = 0.0;
    vector<MatrixXd> state_space = ugv.stateSpace(ref_delta, ref_yaw);
    MatrixXd A = state_space[0];
    MatrixXd B = state_space[1];

    MatrixXd Q(3, 3);
    Q << 3, 0.0, 0.0,
         0.0, 3, 0.0,
         0.0, 0.0, 3.0;

    MatrixXd R(2, 2);
    R << 2.0, 0.0,
        0.0, 2.0;

    MatrixXd P = CalRicatti(A, B, Q, R);

    MatrixXd K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    double x_e = robot_state[0] -refer_path[min_ind][0];
    double y_e = robot_state[1] - refer_path[min_ind][1];
    double error_yaw = ugv.psi - ref_yaw;
    while (error_yaw > PI) error_yaw -= 2.0 * PI;
    while (error_yaw < -PI) error_yaw += 2.0 * PI;
    VectorXd x_error(3);
    x_error << x_e, y_e, error_yaw;

    VectorXd u = -K * x_error;
    // VectorXd u = -K * x;

    double delta = u[1];
    delta += ref_delta;
    output.emplace_back(2);
    output.emplace_back(delta);

    return output;
}


int main(){
    vector<vector<double>>refer_path(1000,vector<double>(2));
    vector<double>refer_x,refer_y;
    // generate reference path
    for(int i=0;i<1000;i++){
        refer_path[i][0]=0.1*i;
        refer_path[i][1]=2*sin(refer_path[i][0]/3.0)+2.5*cos(refer_path[i][0]/2.0);
        refer_x.push_back(refer_path[i][0]);
        refer_y.push_back(refer_path[i][1]);
    }

    // kinematic model intital state [0, -1, 0.5]
    KinematicModel ugv(0,-1,0.5,2,2,0.1);

    vector<double>x_,y_;
    vector<double>robot_state(2);
    for(int i = 0; i < 700; i++){
        plt::clf();
        robot_state[0]=ugv.x;
        robot_state[1]=ugv.y;

        vector<double> control_output;
        // control_output = PIDController(robot_state, refer_path, ugv);
        // control_output = PurePursuitController(robot_state, refer_path, ugv);
        // control_output = StanlyController(robot_state, refer_path, ugv);
        control_output = LQRController(robot_state, refer_path, ugv);

        // update state
        ugv.updateState(control_output[0],control_output[1]);
        x_.push_back(ugv.x);
        y_.push_back(ugv.y);
        // picture drawing
        plt::plot(refer_x,refer_y,"b--");
        plt::plot(x_, y_,"r");
        plt::grid(true);
        plt::ylim(-5,5);
        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./controller_demo.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}