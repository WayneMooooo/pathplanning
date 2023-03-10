#include "Dubins.h"
#include<fstream>
#include "DubinsCurve.cpp"
using namespace std;

int main()
{
    Dubins d(400.0, 400.0, PI/2, 100.0, 300.0, 400.0, -PI/2, 100.0);
    d.InitialDubinsCurve();
    //d.draw_path();
    list<Point> path_sequence = d.get_curve();
    fstream f("C:/Users/MoWeimin/Desktop/2.txt",ios::out);

    for(auto point : path_sequence)
    {

           f<< point.x <<" "<< point.y <<" "<<endl;
    }
    f.close();
    return 0;
} 