#include <iostream>
#include "ReedsSheppPath.h"
#include "ReedsShepp.cpp"
#include<fstream>
#define pi 3.1415926
using namespace std;

int main()
{
    double q0[3]={400.0,400.0,pi/2};
    double q1[3]={300,400,-pi/2};

    ReedsSheppStateSpace   *r=new ReedsSheppStateSpace;
    ReedsSheppStateSpace::ReedsSheppPath *rr=new ReedsSheppStateSpace::ReedsSheppPath;
//-------------------------------------------get each curve length  +:forword   -back--------------------------------------
    for(int i=0;i<5;i++)
   {
       cout<<r->reedsShepp(q0,q1).length_[i]<<endl;
   }

 //-------------------------------------------q0 to q1 discrete point---------------------------------------
    vector<vector<double > >finalpath;
    finalpath=r->pathsample(q0,q1,0.1);
    fstream f("C:/Users/DELL/Desktop/1.txt",ios::out);

    for(int i=0;i<finalpath.size();i++)
    {

           f<<finalpath[i][0]<<" "<<finalpath[i][1]<<" "<<finalpath[i][2]<<endl;
    }
f.close();


}