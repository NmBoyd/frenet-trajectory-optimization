#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

#include "FrenetOptimization.h"
#include "PolynomialTraj.h"
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

    int n = 100;
    // TODO: Generate cubic spline of route
    // TODO: Check the polynomial interpolation for quartic and quintic
    // TODO: Write the frenet optimal planner
    // TODO: Tune where needed and add lane-based FSM

	for(int i=0; i<n; i++) {

        // Update vehicle state (current, previous, and end state)

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
        plt::xlim(0, 100);
        plt::title("World map");
        plt::legend();
        plt::grid(true);
        plt::pause(0.0001);
        std::cout << "looped" <<std::endl;
	}
    return 0;
}