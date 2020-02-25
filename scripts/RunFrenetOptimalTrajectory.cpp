#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

#include "FrenetOptimalPlanner.h"
#include "json.hpp"
#include "matplotlib-cpp/matplotlibcpp.h"
 
using namespace Eigen;
namespace plt = matplotlibcpp;

int main()
{
    // waypoints
    std::vector<double> wp_x, wp_y;
    wp_x.push_back(0.0); wp_y.push_back(0.0);
    wp_x.push_back(10.0); wp_y.push_back(-6.0);
    wp_x.push_back(20.5); wp_y.push_back(5.0);
    wp_x.push_back(35.0); wp_y.push_back(6.5);
    wp_x.push_back(70.5); wp_y.push_back(0.0);

    // obstacles
    std::vector<double> ob_x, ob_y;
    ob_x.push_back(20.0); ob_y.push_back(10.0);
    ob_x.push_back(30.0); ob_y.push_back(6.0);
    ob_x.push_back(30.0); ob_y.push_back(8.0);
    ob_x.push_back(35.0); ob_y.push_back(8.0);
    ob_x.push_back(50.0); ob_y.push_back(3.0);

    std::vector<Vector2d> obstacle_poses;
    for (int i=0;i<ob_x.size();i++)
    {
        obstacle_poses.push_back(Vector2d(ob_x[i], ob_y[i]));
    }

    Spline2D route_c_spline(wp_x, wp_y);
    PathSE2 path;
    const double seg_len = 0.1; // segment length
    for (double i=0;i < route_c_spline.s.back(); i+=seg_len)
    {
        std::array<double, 2> point_ = route_c_spline.calc_postion(i);
        path.x.push_back(point_[0]);
        path.y.push_back(point_[1]);
        path.yaw.push_back(route_c_spline.calc_yaw(i));
        path.k.push_back(route_c_spline.calc_curvature(i));
    }

    int n = 100;
    // TODO: Write the frenet optimal planner
    // TODO: Tune where needed and add lane-based FSM
    FrenetOptimalPlanner::Params params;
    std::shared_ptr<FrenetOptimalPlanner> planner = std::make_shared<FrenetOptimalPlanner>();

    FrenetState curr_state;
    curr_state.speed = 10.0/3.6;    // current speed [m/s]
    curr_state.d = 2.0;             // current lateral position [m]
    curr_state.d_d = 0.0;           // current lateral speed [m/s]
    curr_state.d_dd = 0.0;          // current lateral accel [m/s2]
    curr_state.s = 0.0;             // current course position

	for(int i=0; i<n; i++) {

        // Calculate the optimal motion plan
        FrenetTrajectory traj;
        traj = planner->calcOptimalMotionPlan(route_c_spline, curr_state, obstacle_poses);
        
        // Update vehicle state (current, previous, and end state)
        curr_state.s = traj.s[1];
        curr_state.speed = traj.s_d[1];
        curr_state.d = traj.d[1];
        curr_state.d_d = traj.d_d[1];
        curr_state.d_dd = traj.d_dd[1];

        // Update nearby vehicle state

        // Update the route spline interpolation.

        // Frenet Optimal Planning (route spline, curr_course_pose, curr_speed, curr_lat_pose, curr_lat_speed, curr_lat_accel)
            // Calculate the list of frenet paths and convert them to global space
            // Check to make sure the paths are valid
            // find the minimum cost option
            // get the best path
        
        // Update the plot shown
        plt::clf();
        plt::scatter(wp_x, wp_y, 100.0);
        plt::plot(ob_x, ob_y, "xk");
        plt::plot(path.x, path.y);
        plt::xlim(0, 100);
        plt::title("World map");
        plt::legend();
        plt::grid(true);
        plt::pause(0.0001);
        std::cout << "looped" <<std::endl;
	}
    return 0;
}