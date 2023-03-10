#pragma once

#include "common.h"
#include <cmath>
#include <vector>
#include <list>
#include <iostream>
#include <graphics.h>
#include <conio.h>
#include <string.h>

using namespace std;

typedef struct circle1
{
    Point center_pos;//圆心坐标
    double radius;//半径
    char type; //'1'为顺时针 '0'为逆时针
}circle2;

typedef struct tangent
{
    vector<Point> clockwise_tangent, counterclockwise_tangent;
}tangents;

class Dubins
{
private:
    //车辆初始状态及目标状态
    vec initial_state;
    vec goal_state;

    //对于速度向量的初始圆
    circle2 init_circle_clockwise;           //右转
    circle2 init_circle_counterclockwise;    //左转

    //对于速度向量的终点圆
    circle2 goal_circle_clockwise;
    circle2 goal_circle_counterclockwise;

    class path{
    public: 
        list<Point> path_sequence;
        string path_type;

        Point start1;
        Point start2;
        Point start3;
    };

    list<Point> path_sequence;
    string path_type;
    double cost;
    Point start1;
    Point start2;
    Point start3;
    //计算起始圆及终点圆的圆心
    void GenerateInitCircle();
    void GenerateGoalCircle();
    tangents getTangents(circle2 c1, circle2 c2); //计算两圆之间的切线
    double getCSC(path &p);//计算CSC轨迹
    double getCCC(path &p);//计算CCC轨迹
    double getRelativeAngle(Point circle_center, Point taget);//计算点相对于x轴正向的角度（顺时针）（相对坐标系：以圆心为原点）
    double getArcAngle(Point pts1, Point pts2);//计算圆上两点间的角度（0~180°）
    double JudgeArc(circle2 c, Point pointcut, char flag, Point mid={0, 0});//通过起点终点及轨迹方向判断优弧劣弧
    double pathlength_CCC(circle2 start_circle_selected, circle2 end_circle_selected, Point &pt1, Point &pt2, circle2 &public_circle);
public:
    Dubins(double init_x, double init_y, double init_theta, double init_R, double goal_x, double goal_y, double goal_theta, double goal_R);
    void InitialDubinsCurve();
    void draw_path();
    list<Point> get_curve(){   //获取所求Dubins轨迹点序列
    return path_sequence;
    }  
};


