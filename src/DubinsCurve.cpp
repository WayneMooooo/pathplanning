#include"Dubins.h"

#define PI 3.1415926535
#define deta_theta 0.1*PI/18
#define redundancy 0.1

Dubins::Dubins(double init_x, double init_y, double init_theta, double init_R, double goal_x, double goal_y, double goal_theta, double goal_R){
    initial_state.position = {init_x, init_y};
    initial_state.R = init_R;
    initial_state.theta = init_theta;

    goal_state.position = {goal_x, goal_y};
    goal_state.R = goal_R;
    goal_state.theta = goal_theta;
}

void Dubins::GenerateInitCircle(){
    init_circle_clockwise.center_pos.x = initial_state.position.x + initial_state.R*sin(initial_state.theta);
    init_circle_clockwise.center_pos.y = initial_state.position.y - initial_state.R*cos(initial_state.theta);
    init_circle_clockwise.type = '1';//1为顺时针，0为逆时针
    init_circle_clockwise.radius = initial_state.R;

    init_circle_counterclockwise.center_pos.x = initial_state.position.x - initial_state.R*sin(initial_state.theta);
    init_circle_counterclockwise.center_pos.y = initial_state.position.y + initial_state.R*cos(initial_state.theta);
    init_circle_counterclockwise.type = '0';//1为顺时针，0为逆时针
    init_circle_counterclockwise.radius = initial_state.R;
}

void Dubins::GenerateGoalCircle(){
    goal_circle_clockwise.center_pos.x = goal_state.position.x + goal_state.R*sin(goal_state.theta);
    goal_circle_clockwise.center_pos.y = goal_state.position.y - goal_state.R*cos(goal_state.theta);
    goal_circle_clockwise.type = '1';//1为顺时针，0为逆时针
    goal_circle_clockwise.radius = goal_state.R;

    goal_circle_counterclockwise.center_pos.x = goal_state.position.x - goal_state.R*sin(goal_state.theta);
    goal_circle_counterclockwise.center_pos.y = goal_state.position.y + goal_state.R*cos(goal_state.theta);
    goal_circle_counterclockwise.type = '0';//1为顺时针，0为逆时针
    goal_circle_counterclockwise.radius = goal_state.R;
}

tangents Dubins::getTangents(circle2 c1, circle2 c2){
    Point c1_tangent, c2_tangent;
    tangents Tangents;
    double Dis = sqrt(pow((c1.center_pos.x - c2.center_pos.x),2)+pow((c1.center_pos.y - c2.center_pos.y),2));
    double Vlnx = (c2.center_pos.x - c1.center_pos.x)/Dis;
    double Vlny = (c2.center_pos.y - c1.center_pos.y)/Dis;
    if(c1.type == c2.type){
        double c = (c1.radius - c2.radius)/Dis;
        double nx = Vlnx*c + Vlny*sqrt(1-c*c);
        double ny = Vlny*c - Vlnx*sqrt(1-c*c);
        c1_tangent.x = c1.center_pos.x + c1.radius*nx;
        c1_tangent.y = c1.center_pos.y + c1.radius*ny;
        c2_tangent.x = c2.center_pos.x + c2.radius*nx;
        c2_tangent.y = c2.center_pos.y + c2.radius*ny;
        Tangents.counterclockwise_tangent.push_back(c1_tangent);
        Tangents.counterclockwise_tangent.push_back(c2_tangent);
        
        nx = Vlnx*c - Vlny*sqrt(1-c*c);
        ny = Vlny*c + Vlnx*sqrt(1-c*c);
        c1_tangent.x = c1.center_pos.x + c1.radius*nx;
        c1_tangent.y = c1.center_pos.y + c1.radius*ny;
        c2_tangent.x = c2.center_pos.x + c2.radius*nx;
        c2_tangent.y = c2.center_pos.y + c2.radius*ny;
        Tangents.clockwise_tangent.push_back(c1_tangent);
        Tangents.clockwise_tangent.push_back(c2_tangent);
    }
    else{
        double c = (-c1.radius - c2.radius)/Dis;
        double nx = Vlnx*c + Vlny*sqrt(1-c*c);
        double ny = Vlny*c - Vlnx*sqrt(1-c*c);
        c1_tangent.x = c1.center_pos.x - c1.radius*nx;
        c1_tangent.y = c1.center_pos.y - c1.radius*ny;
        c2_tangent.x = c2.center_pos.x + c2.radius*nx;
        c2_tangent.y = c2.center_pos.y + c2.radius*ny;
        Tangents.clockwise_tangent.push_back(c1_tangent);
        Tangents.clockwise_tangent.push_back(c2_tangent);

        nx = Vlnx*c - Vlny*sqrt(1-c*c);
        ny = Vlny*c + Vlnx*sqrt(1-c*c);
        c1_tangent.x = c1.center_pos.x - c1.radius*nx;
        c1_tangent.y = c1.center_pos.y - c1.radius*ny;
        c2_tangent.x = c2.center_pos.x + c2.radius*nx;
        c2_tangent.y = c2.center_pos.y + c2.radius*ny;
        Tangents.counterclockwise_tangent.push_back(c1_tangent);
        Tangents.counterclockwise_tangent.push_back(c2_tangent);
    }
    return Tangents;
}

double Dubins::getRelativeAngle(Point circle_center, Point target){
    double theta;//目标点与圆心点之间的相对角度
    //计算目标点与圆心点的相对坐标
    Point relative;
    relative.x = target.x - circle_center.x;
    relative.y = target.y - circle_center.y;
    if(relative.y <= 0)
        theta = -atan2(relative.y, relative.x);
    else
        theta = 2*PI - atan2(relative.y, relative.x);
    return theta;
}

double Dubins::getArcAngle(Point pts1, Point pts2)
{
    double ax = pts1.x;  double ay = pts1.y;
    double bx = pts2.x;  double by = pts2.y;
    double theta = acos((ax * bx  + ay * by)/(sqrt(ax*ax+ay*ay) * sqrt(bx*bx + by* by)));

    return theta;
}

/*JudgeArc
    [parameter: c:目标圆  start:圆弧起点坐标  end：圆弧终点坐标  flag：方向标记，'1'为起点圆，'0'为终点圆]
    [description:通过将起点按type方向增加deta_theta角度，得到新点Point_judge。若新点与终点间角度大于起点与终点间角度，则判断为优弧，反之则为劣弧]
*/
double Dubins::JudgeArc(circle2 c , Point pointcut, char flag, Point mid)
{
    Point start, end;
    switch (flag)
    {
    case '1':
        start = {initial_state.position.x - c.center_pos.x, initial_state.position.y - c.center_pos.y};
        end = {pointcut.x - c.center_pos.x, pointcut.y - c.center_pos.y};
        break;
    case '0':
        start = {pointcut.x - c.center_pos.x, pointcut.y - c.center_pos.y};
        end = {goal_state.position.x - c.center_pos.x, goal_state.position.y - c.center_pos.y};
        break;
    case '2':
        start = {pointcut.x - c.center_pos.x, pointcut.y - c.center_pos.y};
        end = {mid.x - c.center_pos.x, mid.y - c.center_pos.y};
    default:
        break;
    }
    Point Point_judge;
    double theta_start = atan2(start.y, start.x);
    double theta_clockwise;
    double theta_counterclockwise;
    double arcAngle;

    switch (c.type)
    {
    case '1':
        theta_clockwise = theta_start - deta_theta;
        Point_judge = {c.radius*cos(theta_clockwise), c.radius*sin(theta_clockwise)};

        // cout << "original: " << getArcAngle(circle_center, start, end)*180/PI << endl;
        // cout << "new: " << getArcAngle(circle_center, Point_judge, end)*180/PI << endl;
        if(getArcAngle(Point_judge, end) > getArcAngle(start, end))
            {
                //cout << "youhuooooo~" << endl;
                arcAngle =  2*PI - getArcAngle(start, end);
            }
        else 
            {
                //cout << "liehuooooo~" << endl;
                arcAngle =  getArcAngle(start, end);
            }
        
        break;
    case '0':
        theta_counterclockwise = theta_start + deta_theta;
        Point_judge = {c.radius*cos(theta_clockwise), c.radius*sin(theta_clockwise)};

        // cout << "original: " << getArcAngle(circle_center, start, end)*180/PI << endl;
        // cout << "new: " << getArcAngle(circle_center, Point_judge, end)*180/PI << endl;
        if(getArcAngle(Point_judge, end) > getArcAngle(start, end))
            {
                //cout << "youhuooooo~" << endl;
                arcAngle =  2*PI - getArcAngle(start, end);
            }
        else 
            {
                //cout << "liehuooooo~" << endl;
                arcAngle = getArcAngle(start, end);
            }

        break;

    
    default:
        break;
    }
    return arcAngle;
}

double Dubins::getCSC(path &p){
    //四种轨迹的路线长度
    double LSL_cost, RSR_cost, LSR_cost, RSL_cost;
    //计算起点圆及终点圆的内公切线（两圆方向相反）与外公切线（两圆方向相同）
    tangents outer_tangents_clockwise = getTangents(init_circle_clockwise, goal_circle_clockwise);        //外公切线(顺时针)
    tangents outer_tangents_counterclockwise = getTangents(init_circle_counterclockwise, goal_circle_counterclockwise);//外公切线(逆时针)
    tangents inner_tangents_R2L = getTangents(init_circle_clockwise, goal_circle_counterclockwise); //内公切线R->L
    tangents inner_tangents_L2R = getTangents(init_circle_counterclockwise, goal_circle_clockwise); //内公切线R->L
    //LSL
    LSL_cost = sqrt(pow(outer_tangents_counterclockwise.counterclockwise_tangent[0].x-outer_tangents_counterclockwise.counterclockwise_tangent[1].x,2)+
                    pow(outer_tangents_counterclockwise.counterclockwise_tangent[0].y-outer_tangents_counterclockwise.counterclockwise_tangent[1].y,2))//直线段长度
              +init_circle_counterclockwise.radius*JudgeArc(init_circle_counterclockwise , outer_tangents_counterclockwise.counterclockwise_tangent[0], '1')//起点圆
              +goal_circle_counterclockwise.radius*JudgeArc(goal_circle_counterclockwise , outer_tangents_counterclockwise.counterclockwise_tangent[1], '0');//起点圆
    if(isnan(LSL_cost))
        LSL_cost = __DBL_MAX__;
    //RSR
    RSR_cost = sqrt(pow(outer_tangents_clockwise.clockwise_tangent[0].x-outer_tangents_clockwise.clockwise_tangent[1].x,2)+
                    pow(outer_tangents_clockwise.clockwise_tangent[0].y-outer_tangents_clockwise.clockwise_tangent[1].y,2))//直线段长度
              +init_circle_clockwise.radius*JudgeArc(init_circle_clockwise , outer_tangents_clockwise.clockwise_tangent[0], '1')//起点圆
              +goal_circle_clockwise.radius*JudgeArc(goal_circle_clockwise , outer_tangents_clockwise.clockwise_tangent[1], '0');//起点圆
    if(isnan(RSR_cost))
        RSR_cost = __DBL_MAX__;
    //LSR
    LSR_cost = sqrt(pow(inner_tangents_L2R.counterclockwise_tangent[0].x-inner_tangents_L2R.counterclockwise_tangent[1].x,2)+
                    pow(inner_tangents_L2R.counterclockwise_tangent[0].y-inner_tangents_L2R.counterclockwise_tangent[1].y,2))//直线段长度
              +init_circle_clockwise.radius*JudgeArc(init_circle_clockwise , inner_tangents_L2R.counterclockwise_tangent[0], '1')//起点圆
              +goal_circle_clockwise.radius*JudgeArc(goal_circle_clockwise , inner_tangents_L2R.counterclockwise_tangent[1], '0');//起点圆
    if(isnan(LSR_cost))
        LSR_cost = __DBL_MAX__;
    //RSL
    RSL_cost = sqrt(pow(inner_tangents_R2L.clockwise_tangent[0].x-inner_tangents_R2L.clockwise_tangent[1].x,2)+
                    pow(inner_tangents_R2L.clockwise_tangent[0].y-inner_tangents_R2L.clockwise_tangent[1].y,2))//直线段长度
              +init_circle_clockwise.radius*JudgeArc(init_circle_clockwise , inner_tangents_R2L.clockwise_tangent[0], '1')//起点圆
              +goal_circle_clockwise.radius*JudgeArc(goal_circle_clockwise , inner_tangents_R2L.clockwise_tangent[1], '0');//起点圆
    if(isnan(RSL_cost))
        RSL_cost = __DBL_MAX__;
    double min = LSL_cost;
    int flag = 0;
    vector<double> path_csc = {LSL_cost, RSR_cost, LSR_cost, RSL_cost};
    for(int i=0;i<4;i++)
    {
        if(path_csc[i] < min){
            min = path_csc[i];
            flag =i;
        }
    }
    double a1, sa1, ea1, R1;
    Point center1, center2;
    double a2, sa2, ea2, R2;
    double ga;
    double x, y, theta, Dis;
    list<Point> temp;
    bool flag_3 = true;
    ga = 0.01*deta_theta;
    Point center;
    switch (flag)
    {
    case 0:
        p.path_type = "LSL";
        sa1 = getRelativeAngle(init_circle_counterclockwise.center_pos, outer_tangents_counterclockwise.counterclockwise_tangent[0]);
        ea1 = getRelativeAngle(init_circle_counterclockwise.center_pos, initial_state.position);
        center1 = init_circle_counterclockwise.center_pos;
        R1 = init_circle_counterclockwise.radius;
        
        theta = atan2(outer_tangents_counterclockwise.counterclockwise_tangent[1].y - outer_tangents_counterclockwise.counterclockwise_tangent[0].y, 
                      outer_tangents_counterclockwise.counterclockwise_tangent[1].x - outer_tangents_counterclockwise.counterclockwise_tangent[0].x);
        Dis = sqrt(pow(outer_tangents_counterclockwise.counterclockwise_tangent[0].x-outer_tangents_counterclockwise.counterclockwise_tangent[1].x,2)+
                   pow(outer_tangents_counterclockwise.counterclockwise_tangent[0].y-outer_tangents_counterclockwise.counterclockwise_tangent[1].y,2));
        x = outer_tangents_counterclockwise.counterclockwise_tangent[0].x; 
        y = outer_tangents_counterclockwise.counterclockwise_tangent[0].y;
        
        // sa2 = getRelativeAngle(goal_circle_counterclockwise.center_pos, goal_state.position);
        // ea2 = getRelativeAngle(goal_circle_counterclockwise.center_pos, outer_tangents_counterclockwise.counterclockwise_tangent[1]);
        sa2 = getRelativeAngle(goal_circle_counterclockwise.center_pos, goal_state.position);
        ea2 = getRelativeAngle(goal_circle_counterclockwise.center_pos, outer_tangents_counterclockwise.counterclockwise_tangent[1]);
        center2 = goal_circle_counterclockwise.center_pos;
        R2 = goal_circle_counterclockwise.radius;
        if(ea1 < sa1)
            ea1 += 2*PI;
        for(a1=sa1;a1<ea1;a1+=ga){
            p.path_sequence.push_front({center1.x + R1*cos(a1), center1.y - R1*sin(a1)});
        }
        p.start1 = p.path_sequence.back();
        for(a1=0;a1<=Dis;a1+=0.01){
            x += 0.01*cos(theta);
            y += 0.01*sin(theta);
            p.path_sequence.push_back({x, y});
        }
        p.start2 = p.path_sequence.back();
        if(ea2 < sa2)
                ea2 += 2*PI;
        for(a2=sa2;a2<ea2;a2+=ga)
            temp.push_front({center2.x + R2*cos(a2), center2.y - R2*sin(a2)});
        for(auto i : temp)
            p.path_sequence.push_back(i);
        p.start3 = p.path_sequence.back();
        
        break;
    case 1:
        p.path_type = "RSR";
        sa1 = getRelativeAngle(init_circle_clockwise.center_pos, initial_state.position);
        ea1 = getRelativeAngle(init_circle_clockwise.center_pos, outer_tangents_clockwise.clockwise_tangent[0]);
        center1 = init_circle_clockwise.center_pos;
        R1 = init_circle_clockwise.radius;
        
        theta = atan2(outer_tangents_clockwise.clockwise_tangent[1].y - outer_tangents_clockwise.clockwise_tangent[0].y, 
                      outer_tangents_clockwise.clockwise_tangent[1].x - outer_tangents_clockwise.clockwise_tangent[0].x);
        Dis = sqrt(pow(outer_tangents_clockwise.clockwise_tangent[0].x-outer_tangents_clockwise.clockwise_tangent[1].x,2)+
                   pow(outer_tangents_clockwise.clockwise_tangent[0].y-outer_tangents_clockwise.clockwise_tangent[1].y,2));
        x = outer_tangents_clockwise.clockwise_tangent[0].x; 
        y = outer_tangents_clockwise.clockwise_tangent[0].y;
        
        sa2 = getRelativeAngle(goal_circle_clockwise.center_pos, outer_tangents_clockwise.clockwise_tangent[1]);
        ea2 = getRelativeAngle(goal_circle_clockwise.center_pos, goal_state.position);
        center2 = goal_circle_clockwise.center_pos;
        R2 = goal_circle_clockwise.radius;
        if(ea1 < sa1)
            ea1 += 2*PI;
        for(a1=sa1;a1<ea1;a1+=ga){
            p.path_sequence.push_back({center1.x + R1*cos(a1), center1.y - R1*sin(a1)});
        }
        p.start1 = p.path_sequence.back();
        for(a1=0;a1<=Dis;a1+=0.01){
            x += 0.01*cos(theta);
            y += 0.01*sin(theta);
            p.path_sequence.push_back({x, y});
        }
        p.start2 = p.path_sequence.back();
        if(ea2 < sa2)
                ea2 += 2*PI;
        for(a2=sa2;a2<ea2;a2+=ga)
            p.path_sequence.push_back({center2.x + R2*cos(a2), center2.y - R2*sin(a2)});
        p.start3 = p.path_sequence.back();
        break;
    case 2:
        p.path_type = "LSR";
        sa1 = getRelativeAngle(init_circle_counterclockwise.center_pos, inner_tangents_L2R.counterclockwise_tangent[0]);
        ea1 = getRelativeAngle(init_circle_counterclockwise.center_pos, initial_state.position);
        center1 = init_circle_counterclockwise.center_pos;
        R1 = init_circle_counterclockwise.radius;
        
        theta = atan2(inner_tangents_L2R.counterclockwise_tangent[1].y - inner_tangents_L2R.counterclockwise_tangent[0].y, 
                      inner_tangents_L2R.counterclockwise_tangent[1].x - inner_tangents_L2R.counterclockwise_tangent[0].x);
        Dis = sqrt(pow(inner_tangents_L2R.counterclockwise_tangent[0].x-inner_tangents_L2R.counterclockwise_tangent[1].x,2)+
                   pow(inner_tangents_L2R.counterclockwise_tangent[0].y-inner_tangents_L2R.counterclockwise_tangent[1].y,2));
        x = inner_tangents_L2R.counterclockwise_tangent[0].x; 
        y = inner_tangents_L2R.counterclockwise_tangent[0].y;
        
        sa2 = getRelativeAngle(goal_circle_clockwise.center_pos, inner_tangents_L2R.counterclockwise_tangent[1]);
        ea2 = getRelativeAngle(goal_circle_clockwise.center_pos, goal_state.position);
        center2 = goal_circle_clockwise.center_pos;
        R2 = goal_circle_clockwise.radius;
        if(ea1 < sa1)
            ea1 += 2*PI;
        for(a1=sa1;a1<ea1;a1+=ga){
            p.path_sequence.push_front({center1.x + R1*cos(a1), center1.y - R1*sin(a1)});
        }
        p.start1 = p.path_sequence.back();
        for(a1=0;a1<=Dis;a1+=0.01){
            x += 0.01*cos(theta);
            y += 0.01*sin(theta);
            p.path_sequence.push_back({x, y});
        }
        p.start2 = p.path_sequence.back();
        if(ea2 < sa2)
                ea2 += 2*PI;
        for(a2=sa2;a2<ea2;a2+=ga)
            p.path_sequence.push_back({center2.x + R2*cos(a2), center2.y - R2*sin(a2)});
        p.start3 = p.path_sequence.back();
        break;
    case 3:
        p.path_type = "RSL";
        sa1 = getRelativeAngle(init_circle_clockwise.center_pos, initial_state.position);
        ea1 = getRelativeAngle(init_circle_clockwise.center_pos, inner_tangents_R2L.clockwise_tangent[0]);
        center1 = init_circle_clockwise.center_pos;
        R1 = init_circle_clockwise.radius;
        
        theta = atan2(inner_tangents_R2L.clockwise_tangent[1].y - inner_tangents_R2L.clockwise_tangent[0].y, 
                      inner_tangents_R2L.clockwise_tangent[1].x - inner_tangents_R2L.clockwise_tangent[0].x);
        Dis = sqrt(pow(inner_tangents_R2L.clockwise_tangent[0].x-inner_tangents_R2L.clockwise_tangent[1].x,2)+
                   pow(inner_tangents_R2L.clockwise_tangent[0].y-inner_tangents_R2L.clockwise_tangent[1].y,2));
        x = inner_tangents_R2L.clockwise_tangent[0].x; 
        y = inner_tangents_R2L.clockwise_tangent[0].y;
        
        sa2 = getRelativeAngle(goal_circle_counterclockwise.center_pos, goal_state.position);
        ea2 = getRelativeAngle(goal_circle_counterclockwise.center_pos, inner_tangents_R2L.clockwise_tangent[1]);
        center2 = goal_circle_counterclockwise.center_pos;
        R2 = goal_circle_counterclockwise.radius;
        if(ea1 < sa1)
            ea1 += 2*PI;
        for(a1=sa1;a1<ea1;a1+=ga){
            p.path_sequence.push_back({center1.x + R1*cos(a1), center1.y - R1*sin(a1)});
        }
        p.start1 = p.path_sequence.back();
        for(a1=0;a1<=Dis;a1+=0.01){
            x += 0.01*cos(theta);
            y += 0.01*sin(theta);
            p.path_sequence.push_back({x, y});
        }
        p.start2 = p.path_sequence.back();
        if(ea2 < sa2)
                ea2 += 2*PI;
        for(a2=sa2;a2<ea2;a2+=ga)
            temp.push_front({center2.x + R2*cos(a2), center2.y - R2*sin(a2)});
        for(auto i : temp)
            p.path_sequence.push_back(i);
        p.start3 = p.path_sequence.back();
        break;
    default:
        break;
    }
   return min;
}

double Dubins::pathlength_CCC(circle2 start_circle_selected, circle2 end_circle_selected, Point &pt1, Point &pt2, circle2 &public_circle)
{
    double Beta =atan2(end_circle_selected.center_pos.y - start_circle_selected.center_pos.y, end_circle_selected.center_pos.x - start_circle_selected.center_pos.x);
    double D12 = sqrt(pow(end_circle_selected.center_pos.x - start_circle_selected.center_pos.x, 2) + pow(end_circle_selected.center_pos.y - start_circle_selected.center_pos.y, 2));
    if(end_circle_selected.radius*2 > start_circle_selected.radius - D12 -end_circle_selected.radius)
        public_circle.radius = end_circle_selected.radius;
    else    
        public_circle.radius = (start_circle_selected.radius - D12 -end_circle_selected.radius)/2;
    double D13 = start_circle_selected.radius + public_circle.radius;
    double D23 = end_circle_selected.radius + public_circle.radius;
    
    //余弦定理求解三圆圆心连线的夹角
    double theta = acos((D13*D13 + D12*D12 - D23*D23)/(2*D13*D12));
    
    if((end_circle_selected.center_pos.y - start_circle_selected.center_pos.y) < 0){//上公切圆
        public_circle.center_pos.x = start_circle_selected.center_pos.x + D13*cos(Beta + theta);
        public_circle.center_pos.y = start_circle_selected.center_pos.y + D13*sin(Beta + theta);
        pt1.x = start_circle_selected.center_pos.x + start_circle_selected.radius*cos(Beta + theta);
        pt1.y = start_circle_selected.center_pos.y + start_circle_selected.radius*sin(Beta + theta);
    }
    else{//下公切圆
        public_circle.center_pos.x = start_circle_selected.center_pos.x + D13*cos(Beta - theta);
        public_circle.center_pos.y = start_circle_selected.center_pos.y + D13*sin(Beta - theta);
        pt1.x = start_circle_selected.center_pos.x + start_circle_selected.radius*cos(Beta - theta);
        pt1.y = start_circle_selected.center_pos.y + start_circle_selected.radius*sin(Beta - theta);
    }
    // setlinestyle(PS_DASH, 3);
    // circle(public_circle.center_pos.x, public_circle.center_pos.y, public_circle.radius);
    pt2.x = (public_circle.center_pos.x + end_circle_selected.center_pos.x)/2;
    pt2.y = (public_circle.center_pos.y + end_circle_selected.center_pos.y)/2;
    double cost;
    cost = start_circle_selected.radius*JudgeArc(start_circle_selected , pt1, '1')//起点圆
           +public_circle.radius*JudgeArc(start_circle_selected , pt1, '2', pt2)//公切圆
           +end_circle_selected.radius*JudgeArc(end_circle_selected , pt2, '0');//终点圆
    return cost;
}

double Dubins::getCCC(path &p){
    circle2 start_circle_selected;
    circle2 end_circle_selected;
    circle2 public_circle, public_circle_RLR, public_circle_LRL;
    
    Point center1, center2, center3;
    double a1, sa1, ea1, R1;
    double a2, sa2, ea2, R2;
    double a3, sa3, ea3, R3;
    double ga =ga = 0.01*deta_theta;
    
    Point pt1, pt1_RLR, pt1_LRL;
    Point pt2, pt2_RLR, pt2_LRL;
    double a, b, m;
    a = pathlength_CCC(init_circle_clockwise, goal_circle_clockwise, pt1_RLR, pt2_RLR, public_circle_RLR);
    b = pathlength_CCC(init_circle_counterclockwise, goal_circle_counterclockwise, pt1_LRL, pt2_LRL, public_circle);
    m = (a > b)?(b):(a);
    if(a < b){
        start_circle_selected = init_circle_clockwise;//起点类型：L
        end_circle_selected = goal_circle_clockwise;  //终点类型：L
        p.path_type = "RLR";
        pt1 = pt1_RLR;
        pt2 = pt2_RLR;
        public_circle = public_circle_RLR;
        public_circle.type = '0';
    }
    else{
        start_circle_selected = init_circle_counterclockwise;//起点类型：L
        end_circle_selected = goal_circle_counterclockwise;  //终点类型：L
        p.path_type = "LRL";
        pt1 = pt1_LRL;
        pt2 = pt2_LRL;
        public_circle = public_circle_LRL;
        public_circle.type = '1';
    }
    
    center1 = start_circle_selected.center_pos;
    R1 = start_circle_selected.radius;
    center2 = public_circle.center_pos;
    R2 = public_circle.radius;
    center3 = end_circle_selected.center_pos;
    R3 = end_circle_selected.radius;
    
    //setlinestyle(PS_DASH, 3);
    //circle(public_circle.center_pos.x, public_circle.center_pos.y, public_circle.radius);
    list<Point> temp1, temp2;
    if(public_circle.type == '1'){//LRL
        sa1 = getRelativeAngle(start_circle_selected.center_pos, pt1);
        ea1 = getRelativeAngle(start_circle_selected.center_pos, initial_state.position);
        sa2 = getRelativeAngle(public_circle.center_pos, pt1);
        ea2 = getRelativeAngle(public_circle.center_pos, pt2);
        sa3 = getRelativeAngle(end_circle_selected.center_pos, goal_state.position);
        ea3 = getRelativeAngle(end_circle_selected.center_pos, pt2);
        //获取第一段圆弧
        if(ea1 < sa1)
            ea1 += 2*PI;
        for(a1=sa1;a1<ea1;a1+=ga){
            temp1.push_front({center1.x + R1*cos(a1), center1.y - R1*sin(a1)});
        }
        for(auto i : temp1)
            p.path_sequence.push_back(i);
        p.start1 = p.path_sequence.back();
        //获取第二段圆弧
        if(ea2 < sa2)
            ea2 += 2*PI;
        for(a2=sa2;a2<ea2;a2+=ga){
            p.path_sequence.push_back({center2.x + R2*cos(a2), center2.y - R2*sin(a2)});
        }
        p.start2 = p.path_sequence.back();
        //获取第三段圆弧
        if(ea3 < sa3)
            ea3 += 2*PI;
        for(a3=sa3;a3<ea3;a3+=ga)
            temp2.push_front({center3.x + R3*cos(a3), center3.y - R3*sin(a3)});
        for(auto i : temp2)
            p.path_sequence.push_back(i);
        p.start3 = p.path_sequence.back();
        }   
    else if(public_circle.type == '0'){//RLR
        sa1 = getRelativeAngle(start_circle_selected.center_pos, initial_state.position);
        ea1 = getRelativeAngle(start_circle_selected.center_pos, pt1);
        sa2 = getRelativeAngle(public_circle.center_pos, pt2);
        ea2 = getRelativeAngle(public_circle.center_pos, pt1);
        sa3 = getRelativeAngle(end_circle_selected.center_pos, pt2);
        ea3 = getRelativeAngle(end_circle_selected.center_pos, goal_state.position);
        //获取第一段圆弧
        if(ea1 < sa1)
            ea1 += 2*PI;
        for(a1=sa1;a1<ea1;a1+=ga){
            p.path_sequence.push_back({center1.x + R1*cos(a1), center1.y - R1*sin(a1)});
        }
        p.start1 = p.path_sequence.back();
        //获取第二段圆弧
        if(ea2 < sa2)
            ea2 += 2*PI;
        double t1 = ea2*180/2/PI;
        double t2 = sa2*180/2/PI;
        for(a2=sa2;a2<ea2;a2+=ga)
            temp1.push_front({center2.x + R2*cos(a2), center2.y - R2*sin(a2)});
        for(auto i : temp1)
            p.path_sequence.push_back(i);
        p.start2 = p.path_sequence.back();
        //获取第三段圆弧
        if(ea3 < sa3)
            ea3 += 2*PI;
        for(a3=sa3;a3<ea3;a3+=ga)
            p.path_sequence.push_back({center3.x + R3*cos(a3), center3.y - R3*sin(a3)});        
        p.start3 = p.path_sequence.back();    
    }
    return m;
}

void Dubins::draw_path(){
    initgraph(1000,800,EX_SHOWCONSOLE);  // 创建画布
    //画起点速度方向
    setfillcolor(GREEN);//设置画笔颜色、填充颜色
    setlinecolor(GREEN);
    setlinestyle(PS_SOLID, 5);
    line(initial_state.position.x, initial_state.position.y, initial_state.position.x + 50*cos(initial_state.theta), initial_state.position.y + 50*sin(initial_state.theta));
    //画终点速度方向
    setfillcolor(RED);//设置画笔颜色、填充颜色
    setlinecolor(RED);
    setlinestyle(PS_SOLID, 5);
    line(goal_state.position.x, goal_state.position.y, goal_state.position.x + 50*cos(goal_state.theta), goal_state.position.y + 50*sin(goal_state.theta));

    setfillcolor(WHITE);//设置画笔颜色、填充颜色
    setlinecolor(WHITE);
    setlinestyle(PS_DASH, 3);
    if(path_type == "LSL" || path_type == "LSR" || path_type == "LRL")
        circle(init_circle_counterclockwise.center_pos.x, init_circle_counterclockwise.center_pos.y, init_circle_counterclockwise.radius);
    else    
        circle(init_circle_clockwise.center_pos.x, init_circle_clockwise.center_pos.y, init_circle_clockwise.radius);
    if(path_type == "LSL" || path_type == "RSL" || path_type == "LRL")
        circle(goal_circle_counterclockwise.center_pos.x, goal_circle_counterclockwise.center_pos.y, goal_circle_counterclockwise.radius);
    else    
        circle(goal_circle_clockwise.center_pos.x, goal_circle_clockwise.center_pos.y, goal_circle_clockwise.radius);    

    setfillcolor(YELLOW);//设置画笔颜色、填充颜色
    setlinecolor(YELLOW);
    setlinestyle(PS_SOLID, 3);
    int i = 0;
    for(auto point : path_sequence){
        fillcircle(point.x,point.y,1);
    }

    setfillcolor(BROWN);//设置画笔颜色、填充颜色
    setlinecolor(BROWN);
    fillcircle(start1.x, start1.y, 3);
    setfillcolor(GREEN);//设置画笔颜色、填充颜色
    setlinecolor(GREEN);
    fillcircle(start2.x, start2.y, 9);
    setfillcolor(BLUE);//设置画笔颜色、填充颜色
    setlinecolor(BLUE);
    fillcircle(start3.x, start3.y, 6);
    _getch();        // Press any key to continue
    closegraph();      // Close the graphics window
}

void Dubins::InitialDubinsCurve(){
    GenerateInitCircle();
    GenerateGoalCircle();

    double Dis_LR = sqrt(pow(init_circle_counterclockwise.center_pos.x - goal_circle_clockwise.center_pos.x, 2) + (init_circle_counterclockwise.center_pos.y - goal_circle_clockwise.center_pos.y, 2));
    double Dis_LL = sqrt(pow(init_circle_counterclockwise.center_pos.x - goal_circle_counterclockwise.center_pos.x, 2) + (init_circle_counterclockwise.center_pos.y - goal_circle_counterclockwise.center_pos.y, 2));
    double Dis_RR = sqrt(pow(init_circle_clockwise.center_pos.x - goal_circle_clockwise.center_pos.x, 2) + (init_circle_clockwise.center_pos.y - goal_circle_clockwise.center_pos.y, 2));
    double Dis_RL = sqrt(pow(init_circle_clockwise.center_pos.x - goal_circle_counterclockwise.center_pos.x, 2) + (init_circle_clockwise.center_pos.y - goal_circle_counterclockwise.center_pos.y, 2));
    double deta_R = fabs(initial_state.R - goal_state.R);

    path path_CSC, path_CCC;
    double a = getCCC(path_CCC);
    double b = getCSC(path_CSC);
    cost = (a < b)?(a):(b);

    if(Dis_LL <= (initial_state.R + goal_state.R + redundancy) || Dis_RR <= (initial_state.R + goal_state.R + redundancy))
        if(a < b){
            path_sequence = path_CCC.path_sequence;
            path_type = path_CCC.path_type;
            start1 = path_CCC.start1;
            start2 = path_CCC.start2;
            start3 = path_CCC.start3;
        }
        else{
            path_sequence = path_CSC.path_sequence;
            path_type = path_CSC.path_type;
            start1 = path_CSC.start1;
            start2 = path_CSC.start2;
            start3 = path_CSC.start3;
        }
    else{
        getCSC(path_CSC);
        path_sequence = path_CSC.path_sequence;
        path_type = path_CSC.path_type;
        start1 = path_CSC.start1;
        start2 = path_CSC.start2;
        start3 = path_CSC.start3;
    }
    cout << "最短路径类型为：" << path_type << " ,路径长度为：" << cost  << "。" << endl;
    // getCCC(path_sequence_CCC, path_type_CCC);
    // path_sequence = path_sequence_CCC;
    // path_type = path_type_CCC;
}