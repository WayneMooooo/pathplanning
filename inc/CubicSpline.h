#ifndef CUBICSPLINE
#define CUBICSPLINE

#include <iostream>
#include<math.h>
#include<vector>
#include <numeric>
#include <algorithm>
#include<cstdlib>
#include "common.h"
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

#define Default_sampling_interval 0.1

class CubicSpline1D
{
public:
    CubicSpline1D() = default;
    CubicSpline1D(vector<double> x, vector<double> y, double ds=Default_sampling_interval); 
    double _calc_position(double x);
    double _calc_first_derivative(double x);    //一阶导
    double _calc_second_derivative(double x);   //二阶导
    vector<Point> _pathsample();
private:
    double ds;
    vector<double> x_seq, y_seq;
    vector<double> a = {}, b = {}, c = {}, d = {};          //四项系数
    int np; //型值点个数
    MatrixXd _calc_A(vector<double> h);                 //计算方程齐次系数
    MatrixXd _calc_B(vector<double> h);                 //计算方程非齐次系数
};

class CubicSpline2D
{
public:
    CubicSpline2D() = default;
    CubicSpline2D(vector<double> x, vector<double> y, double ds=Default_sampling_interval);
    vector<double> _calc_s(vector<double> x, vector<double> y);
    Point _calc_position(double s);
    double _calc_curvature(double s);       //计算曲率
    double _calc_yaw(double s);             //计算航向角
    vector<Point> _pathsample();
    vector<double> s_seq;

    vector<double> x = {}, y = {}, yaw = {}, k = {}, s = {};
private:
    double ds;
    CubicSpline1D sx, sy;
};

//计算vector<double>中相邻元素之间的差值，并返回一个差值序列
inline vector<double> diff(vector<double> x)
{
    vector<double> b(x.size());
	adjacent_difference(x.begin(), x.end(), b.begin());
    b.erase(b.begin());
    return b;
}

//查找x在有序vector<double>中所在区间左边的数
inline int _search_index(vector<double>::iterator begin, vector<double>::iterator end, double x)
{
    vector<double>::iterator iter1;
	iter1 = upper_bound(begin, end, x);
    --iter1;
    int i = 0;
    for(auto iter = begin; iter != iter1; iter++)
        i++;
    return i;
}

//计算vector<double>从起始元素到每个元素的累积和，返回vector<double>
vector<double> cumsum(vector<double> s)
{
    for(auto iter = s.begin()+1;iter != s.end(); iter++)
        *iter += *(iter-1);
    return s;
}

#endif