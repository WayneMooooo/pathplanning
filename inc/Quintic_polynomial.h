#pragma once

#include <iostream>
#include <graphics.h>    // Reference graphics library header file
#include <conio.h>
#include<math.h>
#include<vector>
#include "common.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
#define PI 3.141592653589793

class QuinticPolynomial
{
public:
    QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time);
    double _calc_point(double t);
    double _calc_first_derivative(double t);
    double _calc_second_derivative(double t);
    double _calc_third_derivative(double t);
    void show(){
        cout << a0 << " " << a1 << " " << a2 << " " << a3 << " " << a4 << " " << a5 << endl;
    }
    double a0, a1, a2, a3, a4, a5;
};

class QuinticPolynomialPlanner
{
public:
    QuinticPolynomialPlanner(double sx, double sy, double syaw, double sv, double sa, 
                             double gx, double gy, double gyaw, double gv, double ga, 
                             double max_accel, double max_jerk, double dt);
    vector<Point> _get_path(){return path_sequence;}
    vector<double> _get_time(){return time;}
    vector<double> _get_ryaw(){return ryaw;}
    vector<double> _get_rv(){return rv;}
    vector<double> _get_ra(){return ra;}
    vector<double> _get_rj(){return rj;}
private:
    vector<double> time, ryaw, rv, ra, rj;
    vector<Point> path_sequence;
};