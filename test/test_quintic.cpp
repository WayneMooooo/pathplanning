#include"Quintic_polynomial.h"
#include"Quintic_polynomial.cpp"
#include<fstream>

int main()
{
    double sx = 10.0;  
    double sy = 10.0; 
    double syaw = 10.0/180*PI;
    double sv = 1.0; 
    double sa = 0.1; 
    double gx = 30.0; 
    double gy = -10.0;  
    double gyaw = 20.0/180*PI;  
    double gv = 1.0;  
    double ga = 0.1;  
    double max_accel = 1.0;
    double max_jerk = 0.5;
    double dt = 0.1;
    
    QuinticPolynomialPlanner Q(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt); 
    vector<Point> path_sequence;
    path_sequence = Q._get_path();
    fstream f("C:/Users/DELL/Desktop/C++2Python/quintic.txt",ios::out);
    for(int i=0;i<path_sequence.size();i++)
    {
        f<<path_sequence[i].x<<" "<<path_sequence[i].y<<endl;
    }
    
    return 0;
}