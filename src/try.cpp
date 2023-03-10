#include<iostream>
#include<cmath>
#include <graphics.h>
#include <conio.h>
#include "common.h"

#define PI 3.1415926535
#define deta_theta 0.1*PI/18

using namespace std;

double getArcAngle(Point pts1, Point pts2)
{
    double ax = pts1.x;  double ay = pts1.y;
    double bx = pts2.x;  double by = pts2.y;
    double theta = acos((ax * bx  + ay * by)/(sqrt(ax*ax+ay*ay) * sqrt(bx*bx + by* by)));

    return theta;
}

double getRelativeAngle(Point circle_center, Point target){
    double theta;//目标点与圆心点之间的相对角度
    //计算目标点与圆心点的相对坐标
    Point relative;
    relative.x = target.x - circle_center.x;
    relative.y = target.y - circle_center.y;
    if(relative.y <= 0)
        theta = -atan2(relative.y, relative.x);
    else
        theta = 2*PI - atan2(relative.y, relative.x);
    return theta;
}
/*JudgeArc
    [parameter: circle_center:圆心坐标  start:圆弧起点坐标  end：圆弧终点坐标  type：方向标记，1为顺时针，0为逆时针  R：圆半径]
    [description:通过将起点按type方向增加deta_theta角度，得到新点Point_judge。若新点与终点间角度大于起点与终点间角度，则判断为优弧，反之则为劣弧]
*/
double JudgeArc(Point circle_center, Point start, Point end, char type, double R)
{
    Point Point_judge;
    double theta_start = atan2(start.y, start.x);
    double theta_clockwise;
    double theta_counterclockwise;
    double arcAngle;

    switch (type)
    {
    case '1':
        theta_clockwise = theta_start - deta_theta;
        Point_judge = {R*cos(theta_clockwise), R*sin(theta_clockwise)};

        // cout << "original: " << getArcAngle(circle_center, start, end)*180/PI << endl;
        // cout << "new: " << getArcAngle(circle_center, Point_judge, end)*180/PI << endl;
        if(getArcAngle(Point_judge, end) > getArcAngle(start, end))
            {
                cout << "youhuooooo~" << endl;
                arcAngle =  2*PI - getArcAngle(start, end);
            }
        else 
            {
                cout << "liehuooooo~" << endl;
                arcAngle =  getArcAngle(start, end);
            }
        
        break;
    case '0':
        theta_counterclockwise = theta_start + deta_theta;
        Point_judge = {R*cos(theta_counterclockwise), R*sin(theta_counterclockwise)};

        // cout << "original: " << getArcAngle(circle_center, start, end)*180/PI << endl;
        // cout << "new: " << getArcAngle(circle_center, Point_judge, end)*180/PI << endl;
        if(getArcAngle(Point_judge, end) > getArcAngle(start, end))
            {
                cout << "youhuooooo~" << endl;
                arcAngle =  2*PI - getArcAngle(start, end);
            }
        else 
            {
                cout << "liehuooooo~" << endl;
                arcAngle = getArcAngle(start, end);
            }

        break;

    
    default:
        break;
    }
    return arcAngle;
}

int main()
{
    initgraph(1000,800,EX_SHOWCONSOLE);  // 创建画布
    setfillcolor(RED);//设置画笔颜色、填充颜色
    setlinecolor(RED);
    Point center, p1, p2;
    center = {200, 60};
    p1 = {150, 60};
    p2 = {192.1, 109.37};
    double R = 50.0;
    setfillcolor(WHITE);//设置画笔颜色、填充颜色
    setlinecolor(WHITE);

    arc(center.x - R, center.y + R, center.x + R, center.y - R, getRelativeAngle(center, p1), getRelativeAngle(center, p2));
    
    _getch();        // Press any key to continue
    closegraph();      // Close the graphics window
    // cout << atan2(-1, 1) << endl;
    // system("pause");
    return 0;
}
