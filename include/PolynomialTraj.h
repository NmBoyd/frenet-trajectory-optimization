#pragma once
#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

typedef struct {
  std::vector<double> time_list; 
  std::vector<double> x_list;
  std::vector<double> y_list;
  std::vector<double> yaw_list;
  std::vector<double> vel_list;
  std::vector<double> accel_list;
  std::vector<double> jerk_list;
} Trajectory2d;

class QuinticPolynomial {
public:
  /**
   * @param min_time the minimum time to the goal
   * @param max_time the max time to the goal
   */
  QuinticPolynomial(double x_init, 
    double v_init, 
    double a_init,
    double x_end, 
    double v_end, 
    double a_end, 
    double time_end);
  ~QuinticPolynomial();

  double GetPosition(double time);
  double GetVelocity(double time);
  double GetAcceleration(double time);
  double GetJerk(double time);

private:
  double a_[6] = {0,0,0,0,0,0};
};

/**
 * quintic polynomial planner
 * 
 * input
 *     @param traj the trajectory being generated
 *     @param sx start x position [m]
 *     @param sy start y position [m]
 *     @param syaw start yaw angle [rad]
 *     @param sa start accel [m/ss]
 *     @param gx goal x position [m]
 *     @param gy goal y position [m]
 *     @param gyaw goal yaw angle [rad]
 *     @param ga goal accel [m/ss]
 *     @param max_accel maximum accel [m/ss]
 *     @param max_jerk maximum jerk [m/sss]
 *     @param dt time tick [s]
 *
 */
bool QuinticPolynomialPlanFrenet(Trajectory2d& traj, double sx, double sy, double syaw,
  double sv, double sa, double gx, double gy, double gyaw, double gv, double ga, double max_accel,
  double max_jerk, double dt, double min_time = 5.0, double max_time = 100.0);

class QuarticPolynomial {
public:
  /**
   * @param min_time the minimum time to the goal
   * @param max_time the max time to the goal
   */
  QuarticPolynomial(double x_init, 
    double v_init, 
    double a_init,
    double v_end, 
    double a_end, 
    double time_end);
  ~QuarticPolynomial();

  double GetPosition(double time);
  double GetVelocity(double time);
  double GetAcceleration(double time);
  double GetJerk(double time);

private:
  double a_[5] = {0,0,0,0,0};
};