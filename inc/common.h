#ifndef COMMON_H
#define COMMON_H

struct Point
{
    double x;
    double y;
};

struct vec
{
    Point position;     //位置（x, y）
    double theta;       //偏航角
    double vel;         //速度
    double R;           //转弯半径
};

#endif