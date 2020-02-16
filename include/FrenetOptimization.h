#pragma once

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "VehicleModel.h"
#include "Spline.h"
#include "Utilities.h"
#include "PolynomialTraj.h"
#include "SafetyChecking.h"
 
using namespace Eigen;

class FrenetOptimization {
    public:
        FrenetOptimization();
        ~FrenetOptimization();

    public:
        double target_d;
        std::vector<VehicleModel> obstacles;
        VehicleModel target_to_follow;
        int following_target_id;
        double dist_to_target;

        MatrixXd s_trajectories;
        VectorXd s_costs;
        MatrixXd d_trajectories;
        VectorXd d_costs;

        bool obstacle_following; // if false: velocity keeping
        bool feasible_traj_exist;
        int optimal_s_id;
        int optimal_d_id;
        double minimal_cost;
        int iters;
};