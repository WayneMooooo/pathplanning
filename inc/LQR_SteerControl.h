#ifndef LQR_STEERCONTROL
#define LQR_STEERCONTROL

#include <iostream>
#include<math.h>
#include<vector>
#include <numeric>
#include <algorithm>
#include<cstdlib>
#include <random>
#include <ctime>
#include "common.h"
#include "vehicle.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

#define Kp 1

class Lqr_steer
{
public:
    Lqr_steer(vector<double> rx, vector<double> ry, vector<double> ryaw, vector<double> rk)
    :rx(rx), ry(ry), ryaw(ryaw), rk(rk){
    }

    void _set_Q(MatrixXd q){
        Q = q;
    }
    void _set_R(MatrixXd r){
        R = r;
    }

    MatrixXd _solve_DARE();
    MatrixXd _dlqr();
    double _lqr_steering_control(double &pe, double &pth_e, int &target_ind);
    int _match_point(double &md);
    void _simulation_loop(Point goal);
    vector<double> _get_x(){return x;}
    vector<double> _get_y(){return y;}
    vector<double> _get_v(){return v;}
    vector<double> _get_t(){return t;}
    
private:
    vector<double> rx, ry, ryaw, rk, x = {}, y = {}, yaw = {}, v = {}, t = {};
    Vehicle_State car;
    Matrix4d A;
    Matrix<double, 4, 1> B;
    MatrixXd Q = MatrixXd::Identity(4, 4);
    MatrixXd R = MatrixXd::Identity(1, 1);
};


inline double PIDControl(double target, double current)
{
    return Kp * (target - current);
}

#endif
