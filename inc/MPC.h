#ifndef MPC_H
#define MPC_H

#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <Eigen/Dense>
#include "vehicle.h"

using namespace std;
using namespace Eigen;

#define NX 4 //状态量维数
#define NU 2 //控制量维数
#define T 5 //预测时域（共六个周期）

//MPC迭代器参数
#define MAX_ITER 3 //最大迭代次数
#define DU_TH  0.1 //迭代结束参数

#define TARGET_SPEED 10.0/3.6 //[m/s]目标速度
#define N_IND_SEARCH 10 //search index number

#define DT 0.2 //[s] 离散采样周期

//车辆参数
#define LENGTH 4.5
#define LENGTH  4.5 
#define WIDTH  2.0 
#define BACKTOWHEEL  1.0 
#define WHEEL_LEN  0.3 
#define WHEEL_WIDTH  0.2  
#define TREAD  0.7  
#define WB  2.5 

#define MAX_STEER deg2rad(45.0) //最大转角[rad]
#define MAX_DSTEER deg2rad(30.0) //最大角速度[rad/s]
#define MAX_SPEED 55.0/3.6 //最大速度[m/s]
#define MIN_SPEED -20.0/3.6 //最小速度[m/s]
#define MAX_ACCEL 1.0 //最大加速度[m/ss]

class MPC_vehicle
{
public:
    MPC_vehicle (vector<double> rx, vector<double> ry, vector<double> ryaw, vector<double> rk)
    :rx(rx), ry(ry), ryaw(ryaw), rk(rk), car(car){
        pdelta = 0.0;
    }
    double x, y, yaw, pdelta;
    void get_linear_model_matrix(double v, double phi, double delta);
    int calc_nearest_index(double &md, int pind);
    int calc_ref_trajectory(vector<double> sp, double dl, int pind, MatrixXd &xref, MatrixXd &dref);
    MatrixXd predict_motion(vector<double> x0, MatrixXd xref);
    void iterative_linear_mpc_control(MatrixXd xref, vector<double> x0, vector<double> dref);
    void linear_mpc_control(MatrixXd xref, vector<double> x0, vector<double> dref, MatrixXd xbar);
    void do_simulation(vector<double> sp, double dl);
    vector<double> smooth_yaw(vector<double> yaw);

private:
    vector<double> rx, ry, ryaw, rk, x = {}, y = {}, yaw = {}, v = {}, t = {};
    Vehicle_State car;
    MatrixXd A = MatrixXd::Zero(NX, NX); //状态矩阵
    MatrixXd B = MatrixXd::Zero(NX, NU); //输入矩阵
    MatrixXd C = MatrixXd::Zero(NX, 1);
    vector<double> ox = {}, oy = {}, oyaw = {}, ov = {}, oa = {}, od = {};
};

vector<double> get_array_from_matrix(MatrixXd x, int n)
{
    
}

#endif