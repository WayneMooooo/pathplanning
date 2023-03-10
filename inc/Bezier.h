#pragma once

#include <graphics.h>    // Reference graphics library header file
#include <conio.h>
#include<iostream>
#include<math.h>
#include<vector>
#include "common.h"
using namespace std;
/*
贝塞尔曲线算法，通过控制点和曲线阶数实例化对象，调用类中的SolveTrajectory函数即可求得贝塞尔曲线
通过调用get_curve函数即可得到曲线的点序列
*/
class Bezier//贝塞尔曲线
{
public:
  Bezier(Point *a, int p):pts(a), n(p){};
  void SolveTrajectory();                 //计算贝塞尔曲线
  vector<Point> get_curve(){   //获取所求贝塞尔曲线点序列
    return path_sequence;
  }    
  void draw_path();

private:
  int n;  //Bezier曲线阶数
  Point *pts; //Bezier曲线控制点
  vector<Point> path_sequence;//Bezier曲线轨迹点序列
};
