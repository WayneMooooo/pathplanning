#include "CubicSpline.h"
using namespace std;
using namespace Eigen;
#define margin  0.001

CubicSpline1D::CubicSpline1D(vector<double> x, vector<double> y, double ds):x_seq(x), y_seq(y), ds(ds)
{
    np = x.size();  //型值点个数
    vector<double> h(np);
    h = diff(x);
    if (any_of(h.begin(), h.end(), [](double i){return i<0;})){
        cout << "x coordinates must be sorted in ascending order." << endl;
        system("pause");
        exit(0);
    }
    a = y;//计算系数a
    MatrixXd A = _calc_A(h);
    MatrixXd B = _calc_B(h);
    MatrixXd solution = A.lu().solve(B);
    for(int i=0;i<solution.rows();i++)//计算系数c
        c.push_back(solution(i,0));
    double b_temp, d_temp;
    for(int i=0;i<np-1;i++){
        b_temp = 1.0 / h[i] * (a[i+1] - a[i]) - h[i] / 3.0 * (2.0 * c[i] + c[i+1]);
        d_temp = (c[i+1] - c[i]) / (3.0 * h[i]);
        b.push_back(b_temp);
        d.push_back(d_temp);
    }
}

MatrixXd CubicSpline1D::_calc_A(vector<double> h)
{
    MatrixXd A = MatrixXd::Zero(np, np);
    A(0, 0) = 1;
    for(int i=0;i<np-1;i++){
        if(i != np-2)
            A(i+1, i+1) = 2.0 * (h[i] + h[i+1]);
        A(i+1, i) = h[i];
        A(i, i+1) = h[i];
    }
    A(0, 1) = 0.0;
    A(np-1, np-2) = 0.0;
    A(np -1, np -1) = 1.0;
    return A;
}

MatrixXd CubicSpline1D::_calc_B(vector<double> h)
{
    MatrixXd B = MatrixXd::Zero(np, 1);
    for(int i=0;i<np-2;i++){
        B(i + 1, 0) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
}

double CubicSpline1D::_calc_position(double x)
{
    if(x < x_seq.front() || x > (x_seq.back()+margin))
        return __DBL_MAX__;
    int i = _search_index(x_seq.begin(), x_seq.end(), x);
    double dx = x - x_seq[i];
    double position = a[i] + b[i] * dx + c[i] * pow(dx, 2.0) + d[i] * pow(dx, 3.0);

    return position;
}

double CubicSpline1D::_calc_first_derivative(double x)
{
    if(x < x_seq.front() || x > (x_seq.back()+margin))
        return __DBL_MAX__;
    int i = _search_index(x_seq.begin(), x_seq.end(), x);
    double dx = x - x_seq[i];
    double dy = b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow(dx, 2.0);
    
    return dy;
}

double CubicSpline1D::_calc_second_derivative(double x)
{
    if(x < x_seq.front() || x > (x_seq.back()+margin))
        return __DBL_MAX__;
    int i = _search_index(x_seq.begin(), x_seq.end(), x);
    double dx = x - x_seq[i];
    double ddy = 2.0 * c[i] + 6.0 * d[i] * dx;
    
    return ddy;
}

vector<Point> CubicSpline1D::_pathsample()
{
    vector<Point> path_sequence;
    for(double i = x_seq.front(); i<(x_seq.back()+ds); i+=ds)
        path_sequence.push_back({i, _calc_position(i)});
    return path_sequence;
}

CubicSpline2D::CubicSpline2D(vector<double> x, vector<double> y, double ds):ds(ds)
{
    s_seq = _calc_s(x, y);
    sx = CubicSpline1D(s_seq, x);
    sy = CubicSpline1D(s_seq, y);
}

vector<double> CubicSpline2D::_calc_s(vector<double> x, vector<double> y)
{
    vector<double> dx = diff(x);
    vector<double> dy = diff(y);
    vector<double> ds = {};
    for(int i=0; i < dx.size(); i++){
        ds.push_back(hypot(dx[i], dy[i]));
    }
    vector<double> s = cumsum(ds);
    s.insert(s.begin(), 0.0);

    return s;
}

Point CubicSpline2D::_calc_position(double s)
{
    Point p;
    p.x = sx._calc_position(s);
    p.y = sy._calc_position(s);

    return p;
}

double CubicSpline2D::_calc_curvature(double s)
{
    double dx = sx._calc_first_derivative(s);
    double ddx = sx._calc_second_derivative(s);
    double dy = sy._calc_first_derivative(s);
    double ddy = sy._calc_second_derivative(s);
    double k = (ddy * dx - ddx * dy) / (pow((pow(dx, 2) + pow(dy, 2)),3 / 2));

    return k;
}

double CubicSpline2D::_calc_yaw(double s)
{
    double dx = sx._calc_first_derivative(s);
    double dy = sy._calc_first_derivative(s);
    double yaw = atan2(dy, dx);

    return yaw;
}

vector<Point> CubicSpline2D::_pathsample()
{
    vector<Point> path_sequence;
    for(double i = 0.0; i<s_seq.back(); i+=ds){
        path_sequence.push_back(_calc_position(i));
        s.push_back(i);
        x.push_back(_calc_position(i).x);
        y.push_back(_calc_position(i).y);
        yaw.push_back(_calc_yaw(i));
        k.push_back(_calc_curvature(i));
    }
    return path_sequence;
}