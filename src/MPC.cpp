#include "MPC.h"

void MPC_vehicle::get_linear_model_matrix(double v, double phi, double delta) //获取线性的车辆动力学模型（自行车模型）
{
    A(0, 0) = 1.0;          //状态矩阵
    A(1, 1) = 1.0;  
    A(2, 2) = 1.0;
    A(3, 3) = 1.0;
    A(0, 2) = DT * cos(phi);
    A(0, 3) = - DT * v * sin(phi);
    A(1, 2) = DT * sin(phi);
    A(1, 3) = DT * v * cos(phi);
    A(3, 2) = DT * tan(delta) / WB;

    B[2, 0] = DT;           //输入矩阵
    B[3, 1] = DT * v / (WB * pow(cos(delta), 2));

    C[0] = DT * v * sin(phi) * phi;
    C[1] = - DT * v * cos(phi) * phi;
    C[3] = - DT * v * delta / (WB * pow(cos(delta), 2));
}

int MPC_vehicle::calc_nearest_index(double &md, int pind)   //寻找参考点序列中距离当前状态最近的点，只往前探索N_IND_SEARCH个点
{
    vector<double> dx(N_IND_SEARCH), dy(N_IND_SEARCH), d(N_IND_SEARCH);
    for (int i = pind; i < (pind + N_IND_SEARCH); ++i) {
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

int MPC_vehicle::calc_ref_trajectory(vector<double> sp, double dl, int pind, MatrixXd &xref, MatrixXd &dref)
{
    xref = MatrixXd::Zero(NX, T+1); //六个周期内的参考状态
    dref = MatrixXd::Zero(1, T+1);          //车辆参考转角
    int ncourse = rx.size();

    double mind;
    int ind = calc_nearest_index(mind, pind);
    if(pind >= ind)
        ind = pind;         //不允许往后寻找匹配点

    xref(0, 0) = rx[ind];
    xref(1, 0) = ry[ind];
    xref(2, 0) = sp[ind];
    xref(3, 0) = ryaw[ind];
    dref(0, 0) = 0.0;

    double travel = 0.0; //车在六个预测周期内走过的距离

    for(int i = 0;i < T+1;i++){
        travel += abs(car.v) * DT;
        int dind = int(round(travel/dl));   //车在每个周期内经过的参考轨迹索引增量

        if((ind + dind) < ncourse){         //车当前状态索引+增量<参考状态总量
            xref(0, i) = rx[ind + dind];
            xref(1, i) = ry[ind + dind];
            xref(2, i) = sp[ind + dind];
            xref(3, i) = ryaw[ind + dind];
            dref(0, i) = 0.0;
        }
        else{
            xref(0, i) = rx[ncourse - 1];
            xref(1, i) = ry[ncourse - 1];
            xref(2, i) = sp[ncourse - 1];
            xref(3, i) = ryaw[ncourse - 1];
            dref(0, i) = 0.0;
        }
    }
    return ind;
}

MatrixXd MPC_vehicle::predict_motion(vector<double> x0, MatrixXd xref) //以x0为初始状态，oa、od为控制量、对未来的各个预测周期（共六个）的状态进行计算并存在xbar中
{
    MatrixXd xbar = MatrixXd::Zero(NX, T+1);    //xref:4*6 xbar初始化，用于存储预测的轨迹状态（一共六个）
    for(int i=0;i<NX;i++)
        xbar(i, 0) = x0[i];     //将初始状态值赋给xbar的第一个状态

    Vehicle_State state(x0[0], x0[1], x0[3], x0[2], DT);
    for(int i=1;i<T+1;i++){     //对未来的各个预测周期（共六个）的状态进行预测并存在xbar中
        state._update_state(oa[i], od[i]);
        xbar(0, i) = state.x;
        xbar(1, i) = state.y;
        xbar(2, i) = state.v;
        xbar(3, i) = state.yaw;
    }

    return xbar;
}

void MPC_vehicle::iterative_linear_mpc_control(MatrixXd xref, vector<double> x0, vector<double> dref)
{
    if(oa.empty()||od.empty()){
        oa = {0.0, 0.0, 0.0, 0.0, 0.0};
        od = {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    for(int i=0;i<MAX_ITER;i++){
        MatrixXd xbar = predict_motion(x0, xref);   //初始状态为x0，控制量为oa、od，计算未来六个周期内的车辆状态
        vector<double> poa = oa;
        vector<double> pod = od;
        linear_mpc_control(xref, x0, dref, xbar);
        double du = accumulate(MPC_minus(oa, poa).begin(), MPC_minus(oa, poa).end(), 0.0) + accumulate(MPC_minus(od, pod).begin(), MPC_minus(od, pod).end(), 0.0);
        if(du <= DU_TH)
            break; 
    }
    cout << "Iterative is max iter" << endl;
}

vector<double> MPC_minus(vector<double> v1, vector<double> v2){
    int n = v1.size();
    vector<double> v3;
    for(int i=0;i<n;i++){
        v3.push_back(abs(v1[i] - v2[i]));
    }
    return v3;
}

void MPC_vehicle::linear_mpc_control(MatrixXd xref, vector<double> x0, vector<double> dref, MatrixXd xbar){}

void MPC_vehicle::do_simulation(vector<double> sp, double dl){}

vector<double> MPC_vehicle::smooth_yaw(vector<double> yaw)
{
    double dyaw;
    int n = yaw.size();
    for(int i=0;i<n;i++)
    {
        dyaw = yaw[i+1] - yaw[i];
        while(dyaw >= PI/2.0){
            yaw[i+1] -+ PI * 2.0;
            dyaw = yaw [i+1] - yaw[i];
        }
        while(dyaw <= -PI/2.0){
            yaw[i+1] += PI * 2.0;
            dyaw = yaw[i+1] - yaw[i];
        }
    }
    return yaw;
}