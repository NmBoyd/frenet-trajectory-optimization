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
    double speed; // longitudinal speed
    double s; // position of the vehicle

    // "lateral" - 
    double d; // lateral position of the vehicle
    double d_d; // lateral speed
    double d_dd; //lateral acceleration
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

    std::vector<double> delta_s; // change in longitudinal pose (distance)
    
};

class FrenetOptimalPlanner {
public:
    struct Params
    {
        double max_speed = 14;          // m/s
        double max_accel = 2.0;         // m/ss
        double max_curvature = 1.0;     // 1/m
        double max_road_width = 7.0;    // m
        double max_predicition_t = 5.0; // s
        double min_prediction_t = 4.0;  // s
        
        double d_road_w = 1.0;          // road width sampling length [m]
        double d_t_s = 1.4;             // target speed sampling length [m/s]
        double n_s_sample = 1;          // sampling number of target speed

        double dt = 0.2;                // time tick [s]
        
        double hull_radius;             // radius [m]

        // cost weights
        double kj = 0.1;                // jerk weight (lat and lon)
        double kt = 0.1;                // sampling time weight
        double kd = 1.0;                // sqr of diff from trgt speed weight
        double klat = 1.0;              // lateral bias
        double klon = 1.0;              // longitudinal bias
    };

    FrenetOptimalPlanner();
    FrenetOptimalPlanner(Params params);
    ~FrenetOptimalPlanner();

    FrenetTrajectory calcOptimalMotionPlan(Spline2D c_spline,
                                        FrenetState f_state,
                                        std::vector<Vector2d> obstacles);

    std::vector<FrenetTrajectory> calcFrenetTrajectory(FrenetState curr_state);

    std::vector<FrenetTrajectory> calcFrenetTrajectoryGlobal(std::vector<FrenetTrajectory> f_traj, 
                                            Spline2D c_spline);

    std::vector<FrenetTrajectory> checkTrajectoryCollisions(std::vector<FrenetTrajectory> f_traj, 
                                std::vector<Vector2d> obstacles);

private:
    Params params_;
};