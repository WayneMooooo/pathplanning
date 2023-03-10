#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <cmath>
using namespace std;

#define PI 3.141592653589793
#define MAX_STEER PI/4
#define MAX_SPEED 55.0 / 3.6
#define MIN_SPEED -20.0 / 3.6
#define L 0.5

class Vehicle_State
{
public:
    Vehicle_State(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0, double dt = 0.1):
    x(x), y(y), yaw(yaw), v(v), dt(dt){}
    void _update_state(double a, double delta){
        delta = max(min(delta, MAX_STEER), -MAX_STEER);
        
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
        yaw += v / L * tan(delta) * dt;
        v += a * dt;

        if(v > MAX_SPEED)
            v = MAX_SPEED;
        else if(v < MIN_SPEED)
            v = MIN_SPEED;
    }
    double x;
    double y;
    double yaw;
    double v;
    double dt;
};

inline double mod2pi(double x)
{
    double v = fmod(x, 2 * PI);
    if (v < - PI)
        v += 2 * PI;
    else
        if (v > PI)
            v -= 2 * PI;
    return v;
}

#endif