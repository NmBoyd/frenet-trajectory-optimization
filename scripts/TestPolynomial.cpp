#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

#include "PolynomialTraj.h"
#include "json.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"
 
using namespace Eigen;
namespace plt = matplotlibcpp;

double deg2rad(double val)
{
    return val*3.14159/180.0;
}

double rad2deg(double val)
{
    return 180.0*val/3.14159;
}

int main()
{
    double sx = 10.0;  // start x position [m]
    double sy = 10.0;  // start y position [m]
    double syaw = deg2rad(10.0);  // start yaw angle [rad]
    double sv = 1.0;  // start speed [m/s]
    double sa = 0.1;  // start accel [m/ss]
    double gx = 30.0;  // goal x position [m]
    double gy = -10.0;  // goal y position [m]
    double gyaw = deg2rad(20.0);  // goal yaw angle [rad]
    double gv = 1.0;  // goal speed [m/s]
    double ga = 0.1;  // goal accel [m/ss]
    double max_accel = 1.0;  // max accel [m/ss]
    double max_jerk = 0.5;  // max jerk [m/sss]
    double dt = 0.1;  // time tick [s]

    Trajectory2d traj;
    bool ok = QuinticPolynomialPlanFrenet(traj, sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt);
    std::cout << "Finished" << std::endl;
    plt::plot(traj.x_list, traj.y_list, "-r");
    plt::show();
    
    for (int i=0;i<traj.yaw_list.size();i++){
        traj.yaw_list[i] = rad2deg(traj.yaw_list[i]);
    }

    plt::plot(traj.time_list, traj.yaw_list, "-r");
    plt::xlabel("Time[s]");
    plt::ylabel("Yaw[deg]");
    plt::grid(true);
    plt::show();

    plt::plot(traj.time_list, traj.vel_list, "-r");
    plt::xlabel("Time[s]");
    plt::ylabel("Speed[m/s]");
    plt::grid(true);
    plt::show();

    plt::plot(traj.time_list, traj.accel_list, "-r");
    plt::xlabel("Time[s]");
    plt::ylabel("accel[m/ss]");
    plt::grid(true);
    plt::show();

    plt::plot(traj.time_list, traj.jerk_list, "-r");
    plt::xlabel("Time[s]");
    plt::ylabel("jerk[m/sss]");
    plt::grid(true);

    plt::show();
    return 0;
}