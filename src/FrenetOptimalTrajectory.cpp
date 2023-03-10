#include "FrenetOptimalTrajectory.h"
#include "CubicSpline.cpp"
#include"Quintic_polynomial.cpp"

QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe, double time)
{
    a0 = xs;
    a1 = vxs;
    a2 = axs/2.0;
    Matrix<double, 2, 2> A;
    Matrix<double, 2, 1> B, solution;
    A << 3*pow(time, 2), 4*pow(time, 3),
         6*time, 12*pow(time, 2);
    B << vxe - a1 - 2 * a2 * time,
         axe - 2 * a2;
    solution = A.lu().solve(B);
    a3 = solution(0, 0);
    a4 = solution(1, 0);
}

double QuarticPolynomial::_calc_point(double t)
{
    return a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4);
}

double QuarticPolynomial::_calc_first_derivative(double t)
{
    return a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3);
}

double QuarticPolynomial::_calc_second_derivative(double t)
{
    return 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2);
}

double QuarticPolynomial::_calc_third_derivative(double t)
{
    return 6 * a3 + 24 * a4 * t;
}

double fun(double acc, double num){
    return acc + pow(num, 2);
}

vector<FrenetPath> _calc_frenet_paths(double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, double s0)
{
    vector<FrenetPath> frenet_paths = {};

    //采样，并对每个目标配置生成轨迹
    for(double di=-MAX_ROAD_WIDTH;di<MAX_ROAD_WIDTH;di+=D_ROAD_W){
        //横向动作规划
        for(double Ti=MIN_T_F;Ti<MAX_T_F;Ti+=DT){
            FrenetPath fp;

            //五次多项式求解横向轨迹
            QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for(double ti=0.0;ti<Ti;ti+=DT){
                fp.t.push_back(ti);
                fp.d.push_back(lat_qp._calc_point(ti));
                fp.d_d.push_back(lat_qp._calc_first_derivative(ti));
                fp.d_dd.push_back(lat_qp._calc_second_derivative(ti));
                fp.d_ddd.push_back(lat_qp._calc_third_derivative(ti));
            }

            //纵向动作规划
            for(double dv=TARGET_SPEED-D_T_S*N_S_SAMPLE;dv<=TARGET_SPEED+D_T_S*N_S_SAMPLE;dv+=D_T_S){
                FrenetPath temp_fp = fp;
                QuarticPolynomial lon_qp(s0, c_speed, c_accel, dv, 0.0, Ti);

                for(auto ti:fp.t){
                    temp_fp.s.push_back(lon_qp._calc_point(ti));
                    temp_fp.s_d.push_back(lon_qp._calc_first_derivative(ti));
                    temp_fp.s_dd.push_back(lon_qp._calc_second_derivative(ti));
                    temp_fp.s_ddd.push_back(lon_qp._calc_third_derivative(ti));
                }
                //两个方向上的各时刻jerk平方和
                double Jd = accumulate(temp_fp.d_ddd.begin(), temp_fp.d_ddd.end(), 0.0, fun);
                double Js = accumulate(temp_fp.s_ddd.begin(), temp_fp.s_ddd.end(), 0.0, fun);

                //规划速度与目标速度差平方
                double ds = pow((TARGET_SPEED - temp_fp.s_d.back()), 2);

                temp_fp.cd = K_J * Jd + K_T * Ti + K_D * pow(temp_fp.d.back(), 2);
                temp_fp.cv = K_J * Js + K_T * Ti + K_D * ds;
                temp_fp.cf = K_LAT * temp_fp.cd + K_LON * temp_fp.cv;

                frenet_paths.push_back(temp_fp);
            }
        }
    }
    return frenet_paths;
}

vector<FrenetPath> _calc_global_path(vector<FrenetPath> fplist, CubicSpline2D csp)
{
    for(auto &fp:fplist){
        //计算全局位置
        for(int i=0;i<fp.s.size();i++){
            double ix = csp._calc_position(fp.s[i]).x;
            double iy = csp._calc_position(fp.s[i]).y;
            if(ix == __DBL_MAX__)
                break;
            double i_yaw = csp._calc_yaw(fp.s[i]);
            double di = fp.d[i];
            double fx = ix + di * cos(i_yaw + PI / 2.0);
            double fy = iy + di * sin(i_yaw + PI / 2.0);
            fp.x.push_back(fx);
            fp.y.push_back(fy);
        }
        //计算航向角及ds
        for(int i=0;i<fp.x.size()-1;i++){
            double dx = fp.x[i+1] - fp.x[i];
            double dy = fp.y[i+1] - fp.y[i];
            fp.yaw.push_back(atan2(dy, dx));
            fp.ds.push_back(hypot(dx, dy));
        }
        fp.yaw.push_back(fp.yaw.back());
        fp.ds.push_back(fp.ds.back());

        //计算曲率
        for(int i=0;i<fp.yaw.size()-1;i++){
            fp.c.push_back((fp.yaw[i+1] - fp.yaw[i]) / fp.ds[i]);
        }
    }
    return fplist;
}

bool _check_collision(FrenetPath fp, vector<Point> ob)
{
    for(auto o:ob){
        vector<double> d ={};
        for(int i=0;i<fp.x.size();i++){
            d.push_back(pow((fp.x[i] - o.x), 2) + pow((fp.y[i] - o.y), 2));
        }
        if (any_of(d.begin(), d.end(), [](double i){return i<ROBOT_RADIUS;}))
            return FALSE;
    }
    return TRUE;
}

vector<FrenetPath> _check_path(vector<FrenetPath> fplist, vector<Point> ob)
{
    vector<FrenetPath> oklist;
    for(int i=0;i<fplist.size();i++){
        if (any_of(fplist[i].s_d.begin(), fplist[i].s_d.end(), [](double i){return i>MAX_SPEED;}))
            continue;
        else if(any_of(fplist[i].s_dd.begin(), fplist[i].s_dd.end(), [](double i){return i>MAX_ACCEL;}))
            continue;
        else if(any_of(fplist[i].c.begin(), fplist[i].c.end(), [](double i){return i>MAX_CURVATURE;}))
            continue;
        else if(not _check_collision(fplist[i], ob))
            continue;
        oklist.push_back(fplist[i]);
    }
    return oklist;
}

FrenetPath _frenet_optimal_planning(CubicSpline2D csp, double s0, double c_speed, double c_accel, double c_d, double c_d_d, double c_d_dd, vector<Point> ob)
{
    vector<FrenetPath> fplist = _calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0);
    //cout << fplist.size() << endl;
    // for(auto path:fplist)
    //     cout << path.cf << endl;
    // cout << "\n\n";
    fplist = _calc_global_path(fplist, csp);
    fplist = _check_path(fplist, ob);

    //寻找最优路径
    double min_cost = __DBL_MAX__;
    FrenetPath best_path;
    for(auto fp:fplist){
        if(min_cost >= fp.cf){
            min_cost = fp.cf;
            best_path = fp;
        }
    }
    return best_path;
}

void _generate_target_course(vector<double> x, vector<double> y, CubicSpline2D &csp, vector<double> &rx, vector<double> &ry, vector<double> &ryaw, vector<double> &rc)
{
    csp = CubicSpline2D(x, y);
    rx = {};ry = {};ryaw = {};rc = {};
    for(double is=0.0;is<=csp.s_seq.back();is+=0.1){
        double ix = csp._calc_position(is).x;
        double iy = csp._calc_position(is).y;
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(csp._calc_yaw(is));
        rc.push_back(csp._calc_curvature(is));
    } 
}