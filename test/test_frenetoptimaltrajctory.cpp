#include "FrenetOptimalTrajectory.h"
#include "FrenetOptimalTrajectory.cpp"
#include<fstream>


int main()
{
    vector<double> wx = {0.0, 10.0, 20.5, 35.0, 70.5};
    vector<double> wy = {0.0, -6.0, 5.0, 6.5, 0.0};

    vector<Point> ob = {{20.0, 10.0}, {30.0, 6.0}, {30.0, 8.0}, {35.0, 8.0}, {50.0, 3.0}};
    //vector<Point> ob = {{80.0, 80.0}};
    vector<double> tx, ty, tyaw, tc;
    CubicSpline2D csp;
    _generate_target_course(wx, wy, csp, tx, ty, tyaw, tc);

    //初始状态
    double c_speed = 10.0/3.6;
    double c_accel = 0.0;
    double c_d = 1.0;
    double c_d_d = 0.0;
    double c_d_dd = 0.0;
    double s0 = 0.0;

    vector<double> x = {};
    vector<double> y = {};
    double sum = 0.0;
    for(int i=0;i<SIM_LOOP;i++){
        FrenetPath path;
        path = _frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob);
        sum += path.cf;
        cout << path.cf << "  "<< i << endl;
        s0 = path.s[1];
        cout << s0 << endl;
        x.push_back(path.x[0]);
        y.push_back(path.y[0]);
        c_d = path.d[1];
        c_d_d = path.d_d[1];
        c_d_dd = path.d_dd[1];
        c_speed = path.s_d[1];
        c_accel = path.s_dd[1];
        if (hypot(path.x[1] - tx.back(), path.y[1] - ty.back()) <= 2.0){
            printf("Goal");
            break;
        }
    }
    printf("Finish\n");
    fstream f1("C:/Users/DELL/Desktop/C++2Python/frenet_path.txt",ios::out);
    for(int i=0;i<x.size();i++)
    {
        f1<<x[i]<<" "<<y[i]<<endl;
    }
    f1.close();
    fstream f2("C:/Users/DELL/Desktop/C++2Python/frenet_course.txt",ios::out);
    for(int i=0;i<tx.size();i++)
    {
        f2<<tx[i]<<" "<<ty[i]<<endl;
    }
    f2.close();
    system("pause");
    return 0;
}