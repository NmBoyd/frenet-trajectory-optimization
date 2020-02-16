#pragma once

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
 
using namespace Eigen;

class VehicleModel {
public:
    VehicleModel();
    
public:
    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;
    double speed;
};