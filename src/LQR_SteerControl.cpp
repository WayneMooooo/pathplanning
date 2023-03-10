#include "LQR_SteerControl.h"


MatrixXd Lqr_steer::_solve_DARE()
{
    MatrixXd P = Q;
    int maxiter = 150;
    double eps = 0.01;
    MatrixXd Pn;

    for(int i=0;i<=maxiter;i++){
        Pn = Q + A.transpose()*P*A - A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
        if((Pn - P).array().abs().maxCoeff() < eps)
            break;
        P = Pn;
    }
    return Pn;
}

MatrixXd Lqr_steer::_dlqr()
{
    MatrixXd P = _solve_DARE();
    //cout << P << endl;
    MatrixXd K = (B.transpose()*P*B + R).inverse()*(B.transpose()*P*A);
    return K;
}

int Lqr_steer::_match_point(double &md)
{
    int n = rx.size();
    vector<double> dx(n), dy(n), d(n);
    for (int i = 0; i < n; ++i) {
        dx[i] = car.x - rx[i];
        dy[i] = car.y - ry[i];
        d[i] = dx[i] * dx[i] + dy[i] * dy[i];
    }
    
    int ind = min_element(d.begin(), d.end()) - d.begin();
    md = sqrt(d[ind]);

    double dxl = rx[ind] - car.x;
    double dyl = ry[ind] - car.y;

    double angle = mod2pi(ryaw[ind] - atan2(dyl, dxl));
    if(angle < 0)
        md *= -1;
    return ind;
}

double Lqr_steer::_lqr_steering_control(double &pe, double &pth_e, int &target_ind)
{
    double e = 0.0;
    int ind = _match_point(e);
    double k = rk[ind];
    double v = car.v;
    double th_e = mod2pi(car.yaw - ryaw[ind]);

    //cout << e << endl;

    A = MatrixXd::Zero(4, 4);
    A(0, 0) = 1.0;
    A(0, 1) = car.dt;
    A(1, 2) = v;
    A(2, 2) = 1.0;
    A(2, 3) = car.dt;

    B = MatrixXd::Zero(4, 1);
    B(3, 0) = v/L;

    MatrixXd K = _dlqr();

    MatrixXd x = MatrixXd::Zero(4, 1);

    x(0, 0) = e;
    x(1, 0) = (e - pe)/car.dt;
    x(2, 0) = th_e;
    x(3, 0) = (th_e - pth_e)/car.dt;
    //cout << x << endl;

    // cout << "K: " << K << "  x: " << x.transpose() << endl;

    double ff = atan2(L*k, 1);
    double fb = mod2pi((-K*x)(0, 0));

    double delta = ff + fb;
    pe = e;
    pth_e = th_e;
    target_ind = ind;
    return delta;
}

void Lqr_steer::_simulation_loop(Point goal)
{
    int T = 500;
    double goal_dis = 0.3;
    double stop_speed = 0.05;

    car = Vehicle_State(-0.0, -0.0, 0.0, 0.0, 0.01);

    double t1 = 0.0;
    double e = 0.0, e_th = 0.0;
    double speed = 5.0, t_flag = 0.0;
    
    int target_ind;

    while(T > t1){
        //cout << "time: " << t << endl;
        double delta = _lqr_steering_control(e, e_th, target_ind);
        speed += 5 * t1;
        //cout << speed << endl;
        double ai = PIDControl(speed/3.6, car.v);
        car._update_state(ai, delta);

        if(abs(car.v) <= stop_speed)
            target_ind += 1;
        
        t1 += car.dt;

        double dx = car.x - goal.x;
        double dy = car.y - goal.y;

        if(hypot(dx, dy) <= goal_dis)
            break;
        x.push_back(car.x);
        y.push_back(car.y);
        yaw.push_back(car.yaw);
        v.push_back(car.v);
        t.push_back(t1);
    }
}