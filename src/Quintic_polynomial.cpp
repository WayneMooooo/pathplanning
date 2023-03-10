#include"Quintic_polynomial.h"
#define MIN_T 5.0
#define MAX_T 100

QuinticPolynomial::QuinticPolynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double time){
    a0 = xs;
    a1 = vxs;
    a2 = axs/2.0;
    Matrix<double, 3, 3> A;
    Matrix<double, 3, 1> B, solution;
    A << pow(time, 3), pow(time, 4), pow(time, 5),
         3*pow(time, 2), 4*pow(time, 3), 5*pow(time, 4),
         6*time, 12*pow(time, 2), 20*pow(time, 3);
    B << xe - a0 - a1 * time - a2 * pow(time, 2),
         vxe - a1 - 2 * a2 * time,
         axe - 2 * a2;
    solution = A.lu().solve(B);
    a3 = solution(0, 0);
    a4 = solution(1, 0);
    a5 = solution(2, 0);
}

double QuinticPolynomial::_calc_point(double t)
{
    double xt = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);
    return xt;
}

double QuinticPolynomial::_calc_first_derivative(double t)
{
    double xt = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);
    return xt;
}

double QuinticPolynomial::_calc_second_derivative(double t)
{
    double xt = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
    return xt;
}

double QuinticPolynomial::_calc_third_derivative(double t)
{
    double xt = 6 * a3 + 24 * a4 * t + 60 * a5 * pow(t, 2);
    return xt;
}

QuinticPolynomialPlanner::QuinticPolynomialPlanner(double sx, double sy, double syaw, double sv, double sa, 
                             double gx, double gy, double gyaw, double gv, double ga, 
                             double max_accel, double max_jerk, double dt)
{
    double vxs = sv * cos(syaw);
    double vys = sv * sin(syaw);
    double vxg = gv * cos(gyaw);
    double vyg = gv * sin(gyaw);
    double axs = sa * cos(syaw);
    double ays = sa * sin(syaw);
    double axg = ga * cos(gyaw);
    double ayg = ga * sin(gyaw);
    double vx, vy, v;
    double ax, ay, a;
    double jx, jy, j;
    //从MIN_T开始寻找满足规划条件的最短规划时间
    for(int T = MIN_T; T < MAX_T; T += MIN_T){
        QuinticPolynomial xqp(sx, vxs, axs, gx, vxg, axg, T);
        QuinticPolynomial yqp(sy, vys, ays, gy, vyg, ayg, T);

        time = {};path_sequence = {};ryaw = {};rv = {};ra = {};rj = {};
        for(double t=0.0;t<T+dt;t+=dt){
            time.push_back(t);
            path_sequence.push_back({xqp._calc_point(t), yqp._calc_point(t)});
            
            vx = xqp._calc_first_derivative(t);
            vy = yqp._calc_first_derivative(t);
            v = hypot(vx, vy);
            rv.push_back(v);
            ryaw.push_back(atan2(vy, vx));

            ax = xqp._calc_second_derivative(t);
            ay = yqp._calc_second_derivative(t);
            a = hypot(ax, ay);
            if((rv.size() >= 2) && (rv[rv.size()-1] - rv[rv.size()-2])<0.0)
                a *= -1;
            ra.push_back(a);

            jx = xqp._calc_third_derivative(t);
            jy = yqp._calc_third_derivative(t);
            j = hypot(jx, jy);
            if((ra.size() >= 2) && (ra[ra.size()-1] - ra[ra.size()-2])<0.0)
                j *= -1;
            rj.push_back(j);
        }
        if(*max_element(ra.begin(), ra.end()) <= max_accel && *max_element(rj.begin(), rj.end()) <= max_jerk){
            printf("find path!!");
            break;
        }
    }
}