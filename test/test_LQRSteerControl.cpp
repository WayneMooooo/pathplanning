#include<fstream>
#include "LQR_SteerControl.h"
#include "vehicle.h"
#include "LQR_SteerControl.cpp"
#include "CubicSpline.h"
#include "CubicSpline.cpp"

int main()
{
    vector<double> ax = {0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0};
    vector<double> ay = {0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0};
    Point goal = {ax.back(), ay.back()};

    CubicSpline2D course(ax, ay);
    course._pathsample();
    vector<double> cx = course.x;
    vector<double> cy = course.y;
    vector<double> cyaw = course.yaw;
    vector<double> ck = course.k;
    // double target_speed = 10.0/3.6;

    Lqr_steer lqr_controller(cx, cy, cyaw, ck);
    lqr_controller._simulation_loop(goal);
    vector<double> x = lqr_controller._get_x();
    vector<double> y = lqr_controller._get_y();
    vector<double> v = lqr_controller._get_v();
    vector<double> t = lqr_controller._get_t();
    fstream f("C:/Users/Moweimin/Desktop/C++2Python/lqr.txt",ios::out);

    for(int i=0;i<x.size();i++)
    {
        f<< x[i] << " " << y[i] << " " << v[i] << " " << t[i] <<endl;
    }
}