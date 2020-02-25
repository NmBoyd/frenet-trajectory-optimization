// This cubic spline planner is done with no context of velocity/acceleration
#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <Eigen/Eigen>
#include <stdexcept>

class Spline {
public:

    std::vector<double> x;
    std::vector<double> y;
    int nx;
    std::vector<double> h;
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> c;
    std::vector<double> d;

    Spline();
    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline(std::vector<double> x_, std::vector<double> y_);

    double calc(double t);

    double calc_d(double t);

    double calc_dd(double t);

private:

    Eigen::MatrixXd calc_A();
    Eigen::VectorXd calc_B();
    int bisect(double t, int start, int end);

};

class Spline2D {
public:

    Spline sx;
    Spline sy;
    std::vector<double> s;

    Spline2D(std::vector<double> x, std::vector<double> y);

    std::array<double, 2> calc_postion(double s_t);

    double calc_curvature(double s_t);

    double calc_yaw(double s_t);

    std::vector<double> calc_s(std::vector<double> x, std::vector<double> y);

private:

    
};
