#pragma once

#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <typeinfo>
#include <vector>


class ReedsSheppStateSpace
{
public:

    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };//路径段类型 0倒车 1左转 2直行 3右转
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];           //路径类型
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
            double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
            double w=0., double x=0.);

        double length() const { return totalLength_; }
        const ReedsSheppPathSegmentType* type_;
        double length_[5];
        double totalLength_;
    };
   double rho_=100;//TURNNING RADIUS

    double distance(double q0[3], double q1[3]);

    std::vector <int> pathtype(double q0[3], double q1[3]);

    std::vector<std::vector<double> > pathsample(double q0[3], double q1[3], double step_size);

    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

public:
    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]);
};
