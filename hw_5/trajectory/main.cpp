#include <iostream>
// #include <Eigen/Core>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


Vector2d BezierTrajectoryPoint(const vector<Vector2d>& way_point, double time){
    Vector2d p(0, 0);
    double t = time;
    p = pow((1-t), 4) * way_point[0] + 4 * pow((1-t), 3) * t * way_point[1] + 6 * pow((1-t), 2) * t * t * way_point[2] + 4 * (1-t) * t * t * t *way_point[3] + pow(t, 4) * way_point[4];
    return p;
}



Vector2d PolynomialTrajectoryPoint(const vector<Vector2d>& way_point, double time){
    Vector2d p(0, 0);
    std::cout << way_point[0] <<std::endl;
    Eigen::VectorXd t_via(5);
    t_via <<  0.0, 0.25, 0.5, 0.75, 1.0;
    // for(int i=0; i<4; i++) {
    //     Vector2d p0 = way_point[i];
    //     Vector2d p1 = way_point[i+1];
    //     Vector2d v0(0,0);
    //     Vector2d v1(0,0);
    //     Vector2d acc0(0,0);
    //     Vector2d acc1(0,0);
    //     double t0 = t_via[i];
    //     double t1 = t_via[i+1];
    //     double a0, b0 , a1, b1, a2, b2, a3, b3, a4, b4, a5, b5 = polynomial(p0,p1,v0,v1,acc0,acc1,t0,t1);
    //     std::cout << a0 <<  b0 << a1 << b1 << a2 <<  b2 << a3 <<  b3 <<  a4 << b4 << a5 << b5 << std::endl;
    //     double q_x =  a0 + a1*(time - t0) + a2*pow((time -t0),2) + a3*pow((time - t0),3 )+ a4*pow((time -  t0),4) + a5*pow((time - t0),5);
    //     double q_y =  b0 + b1*(time - t0) + b2*pow((time -t0),2) + b3*pow((time - t0),3 )+ b4*pow((time -  t0),4) + b5*pow((time - t0),5);
    //     Vector2d p(q_x,q_y);
    // }
    //
    //
    // Eigen::MatrixXd T(6,6);
    // int t00 = 0 ;
    // int t01 = 0.2;
    // T << pow(0,5),pow(0,4),pow(0,3),pow(0,2),pow(0,1),1,
    //     5* pow(0,4),4* pow(0,3),3* pow(0,2),2* pow(0,1),1,0,
    //     20* pow(0,3),12* pow(0,2),6* pow(0,1),2,0,0,
    //     pow(t01,5),pow(t01,4),pow(t01,3),pow(t01, 2),pow(t01,1),1,
    //     5* pow(t01,4),4* pow(t01,3),3* pow(t01, 2),2* pow(t01,1),1,0,
    //     20* pow(t01,3),12* pow(t01,2),6* pow(t01,1),2,0,0;
    // Eigen::VectorXd X(6);
    // Eigen::VectorXd Y(6);
    // X << way_point[0][0],1,0,way_point[1][0],1,0;
    // Y << way_point[0][1],0,0,way_point[1][1],0,0;
    // VectorXd A = (T.transpose() * T).inverse() * T.transpose() * X;
    // VectorXd B = (T.transpose() * T).inverse() * T.transpose() * Y;
    //
    // std::cout << "A coefficients:\n" << A << "\n";
    // std::cout << "B coefficients:\n" << B << "\n";
    //
    // p[0] = A[5] + A[4] * time + A[3] * pow(time, 2) + A[2] * pow(time, 3) + A[1] * pow(time, 4) + A[0] * pow(time, 5);
    // p[1] = B[5] + B[4] * time + B[3] * pow(time, 2) + B[2] * pow(time, 3) + B[1] * pow(time, 4) + B[0] * pow(time, 5);

    MatrixXd T(5, 6);
    VectorXd X(5), Y(5);
    vector<double> t_values = {0.0, 0.25, 0.5, 0.75, 1.0};

    for (int i = 0; i < 5; ++i) {
        double t = t_values[i];
        T(i, 0) = pow(t, 5);
        T(i, 1) = pow(t, 4);
        T(i, 2) = pow(t, 3);
        T(i, 3) = pow(t, 2);
        T(i, 4) = t;
        T(i, 5) = 1;

        X(i) = way_point[i][0];
        Y(i) = way_point[i][1];
    }

    VectorXd A = T.colPivHouseholderQr().solve(X);
    std::cout << A << std::endl;
    VectorXd B = T.colPivHouseholderQr().solve(Y);

    double t = time;
    p[0] = A[0] * pow(t, 5) + A[1] * pow(t, 4) + A[2] * pow(t, 3) + A[3] * pow(t, 2) + A[4] * t + A[5];
    p[1] = B[0] * pow(t, 5) + B[1] * pow(t, 4) + B[2] * pow(t, 3) + B[3] * pow(t, 2) + B[4] * t + B[5];

    return p;
}




int main(){
    vector<Vector2d>Ps{Vector2d (0,0),Vector2d(1,1),Vector2d(2,1),Vector2d(3,0),Vector2d(4,2)};
    //vector<Vector2d>Ps{Vector2d (0,0),Vector2d(1,1)};

    vector<double>x_ref,y_ref;
    for(int i=0;i<Ps.size();i++){
        x_ref.push_back(Ps[i][0]);
        y_ref.push_back(Ps[i][1]);

    }
    std::cout<< Ps.size() << std::endl;
    vector<double>x_,y_;
    for(int t=0;t<100;++t){
        plt::clf();
        // Vector2d pos = PolynomialTrajectoryPoint(Ps ,(double)t/100);

        // std::cout << "x_pos" << pos[0] << "y_pos" << pos[1] << std::endl;
        Vector2d pos = BezierTrajectoryPoint(Ps,(double)t/100);
        //cout<<pos[0]<<","<<pos[1]<<endl;
        x_.push_back(pos[0]);
        y_.push_back(pos[1]);

        //画图
        //plt::xlim(0,1);
        plt::plot(x_, y_,"r");
        plt::plot(x_ref,y_ref);

        plt::pause(0.01);
    }
    // save figure
    const char* filename = "./trajectory.png";
    cout << "Saving result to " << filename << std::endl;
    plt::save(filename);
    plt::show();
    return 0;
}