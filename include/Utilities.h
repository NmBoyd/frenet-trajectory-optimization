#pragma once
#include <math.h>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

vector<VectorXd> InterpolateWayPoints(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
