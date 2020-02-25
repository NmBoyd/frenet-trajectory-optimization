#pragma once

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Utilities.h"
#include "PolynomialTraj.h"
#include "SafetyChecking.h"
#include "CubicSpline.h"
 
using namespace Eigen;


struct FrenetState
{
    // "longitudinal"
    double curr_speed; //current longitudinal speed
    double curr_s; // current position of the vehicle

    // "lateral" - 
    double curr_d; // current lateral position of the vehicle
    double curr_d_d; // current lateral speed
    double curr_d_dd; //current lateral acceleration
};

struct Vehicle
{
    int id;
    FrenetState f_state;
};

struct PathSE2
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> k; //curvature
};

struct FrenetTrajectory
{
    std::vector<double> t;  // time
    std::vector<double> d;  // lateral distance
    std::vector<double> d_d;    // lateral speed
    std::vector<double> d_dd;   // lateral accel
    std::vector<double> d_ddd;  // lateral jerk
    
    std::vector<double> s;  //longitudinal position (distance)
    std::vector<double> s_d; //longitudinal speed
    std::vector<double> s_dd; //longitudinal accel
    std::vector<double> s_ddd; //longitudinal jerk


    double cost_lat;    // lateral cost f the frenet path
    double cost_lon;    // longitudinal cost of the frenet path
    double cost_total;  // total cost of the trajectory

    PathSE2 global_path;
    // std::vector<double> x; // cartesian pose for the trajectory
    // std::vector<double> y;
    // std::vector<double> yaw;  
    // std::vector<double> k;  // curvature

    std::vector<double> delta_s; // change in longitudinal pose (distance)
    
};

class FrenetOptimization {
public:
    FrenetOptimization();
    ~FrenetOptimization();

    FrenetTrajectory calcOptimalMotionPlan(Spline2D c_spline,
                                        FrenetState f_state,
                                        std::vector<Vector2d> obstacles);

    FrenetTrajectory calcFrenetTrajectory(FrenetState curr_state);

    FrenetTrajectory calcFrenetTrajectoryGlobal(FrenetTrajectory f_traj, 
                                            Spline2D c_spline);

    FrenetTrajectory checkTrajectory(FrenetTrajectory f_traj, 
                                std::vector<Vector2d> obstacles);

private:

};