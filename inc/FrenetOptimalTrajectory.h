#ifndef FRENETOPTIMALTRAJECTORY_H
#define FRENETOPTIMALTRAJECTORY_H

#include "CubicSpline.h"
#include "Quintic_polynomial.h"

#define SIM_LOOP 500    //仿真时长

//默认参数
#define MAX_SPEED 50.0/3.6
#define MAX_ACCEL 2.0
#define MAX_CURVATURE 1.0
#define MAX_ROAD_WIDTH 7.0
#define D_ROAD_W 1.0
#define DT 0.2 
#define MAX_T_F 5.0
#define MIN_T_F 4.0
#define TARGET_SPEED 30.0 / 3.6
#define D_T_S 5.0 / 3.6
#define N_S_SAMPLE 1
#define ROBOT_RADIUS 2.0

//代价函数权重
#define K_J 0.1
#define K_T 0.1
#define K_D 1.0
#define K_LAT 1.0
#define K_LON 1.0

class QuarticPolynomial
{
public:
    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time);
    double _calc_point(double t);
    double _calc_first_derivative(double t);
    double _calc_second_derivative(double t);
    double _calc_third_derivative(double t);
private:
    double a0, a1, a2, a3, a4;
};

class FrenetPath
{
public:
    FrenetPath() = default;
    double cd = 0, cf = 0, cv = 0;
    vector<double> t = {}, d = {}, d_d = {}, d_dd = {}, d_ddd = {}, s = {}, s_d = {}, s_dd = {}, s_ddd = {};
    
    vector<double> x = {}, y = {}, yaw = {}, ds = {}, c = {};
};

#endif