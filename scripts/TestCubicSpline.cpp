#include "matplotlib-cpp/matplotlibcpp.h"
#include "CubicSpline.h"
#include <vector>
#include <chrono>

using namespace Eigen;
int main(int argc, char** argv)
{
    std::vector<double> x{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    std::vector<double> y{0.7, -6,   5,   6.5, 0.0, 5.0, -2.0};
    std::vector<double> r_x;
    std::vector<double> r_y;
    std::vector<double> ryaw;
    std::vector<double> rcurvature;
    std::vector<double> rs;
    std::vector<double> t;
    Spline2D csp_obj(x, y);

    // Interpolate every 1/10 distance
    for(double i=0; i<csp_obj.s.back(); i+=0.1){
        std::array<double, 2> point_ = csp_obj.calc_postion(i);
        t.push_back(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }

    matplotlibcpp::figure(1);
    matplotlibcpp::title("X-Y Interpolation");
    matplotlibcpp::ylabel("Y-Position [m]");
    matplotlibcpp::xlabel("X-Position [m]");
    matplotlibcpp::plot(r_x, r_y, "x-");
    matplotlibcpp::figure(2);
    matplotlibcpp::title("Yaw");
    matplotlibcpp::ylabel("Yaw [theta/s]");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::plot(t, ryaw, "r-");   
    matplotlibcpp::figure(3);
    matplotlibcpp::title("Spline Distance");
    matplotlibcpp::ylabel("Distance [m]");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::plot(t, rs, "r-");  
    matplotlibcpp::figure(4);
    matplotlibcpp::title("Spline Curvature");
    matplotlibcpp::ylabel("Curvature [1/m]");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::plot(t, rcurvature, "r-"); 
    matplotlibcpp::show();

}
