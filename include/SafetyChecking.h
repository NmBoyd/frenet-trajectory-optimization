#pragma once

#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
int contains(double n, vector<double> range);

int overlap(vector<double> a, vector<double> b);

int checkCollision(double s0, double d0, double theta0, double s1, double d1, double theta1);
