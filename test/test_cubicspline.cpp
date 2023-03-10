#include "CubicSpline.h"
#include "CubicSpline.cpp"
#include<fstream>

int main()
{
    vector<double> x = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    vector<double> y = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    // vector<double> x = {0, 1, 2, 3, 4};
    // vector<double> y = {1.7, -6, 5, 6.5, 0.0};
    CubicSpline2D eg1(x, y);
    vector<Point> path_sequence;
    vector<double> s_seq, ryaw, rk;
    path_sequence = eg1._pathsample();
    fstream f("C:/Users/DELL/Desktop/C++2Python/cubic.txt",ios::out);
    for(int i=0;i<path_sequence.size();i++)
    {
        f<<path_sequence[i].x<<" "<<path_sequence[i].y<<endl;
    }
    f.close();
    s_seq = eg1.s_seq;
    ryaw = eg1.yaw;
    rk = eg1.k;
    fstream f1("C:/Users/DELL/Desktop/C++2Python/cubic_yaw.txt",ios::out);
    for(int i=0;i<ryaw.size();i++)
    {
        f1<< 0.1*i <<" "<< ryaw[i] << rk[i] <<endl;
    }
    //system("pause");
}