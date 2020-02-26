#include "CubicSpline.h"

std::vector<double> vec_diff(std::vector<double> input){
  std::vector<double> output;
  for(unsigned int i=1; i<input.size(); i++){
    output.push_back(input[i] - input[i-1]);
  }
  return output;
};

std::vector<double> cum_sum(std::vector<double> input){
  std::vector<double> output;
  double temp = 0;
  for(unsigned int i=0; i<input.size(); i++){
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};


// --------------------------------------------------
// Spline 
// --------------------------------------------------

Spline::Spline()
{

}

Spline::Spline(std::vector<double> x_, std::vector<double> y_)
    :x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_)
{
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    double * c_pointer = c_eigen.data();
    c.assign(c_pointer, c_pointer+c_eigen.rows());

    for(int i=0; i<nx-1; i++){
        d.push_back((c[i+1]-c[i])/(3.0*h[i]));
        b.push_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
    }
}

std::shared_ptr<double> Spline::calc(double t)
{
    if(t<x.front() || t>x.back()){
        return nullptr;
    //   throw std::invalid_argument( "Spline: received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    double val = a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx; 
    std::shared_ptr<double> p = std::make_shared<double>(val);
    return p;
}

std::shared_ptr<double> Spline::calc_d(double t)
{
    if(t<x.front() || t>x.back()){
        return nullptr;
    //   throw std::invalid_argument( "Spline: received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx-1);
    double dx = t - x[seg_id];
    double val = b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
    std::shared_ptr<double> p_d = std::make_shared<double>(val);
    return p_d;
}

std::shared_ptr<double> Spline::calc_dd(double t)
{
    if(t<x.front() || t>x.back()){
        return nullptr;
    //   throw std::invalid_argument( "Spline: received value out of the pre-defined range" );
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    double val = 2 * c[seg_id] + 6 * d[seg_id] * dx;
    std::shared_ptr<double> p_dd = std::make_shared<double>(val);
    return p_dd;
}

Eigen::MatrixXd Spline::calc_A()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1;
    for(int i=0; i<nx-1; i++){
        if (i != nx-2){
        A(i+1, i+1) = 2 * (h[i] + h[i+1]);
        }
        A(i+1, i) = h[i];
        A(i, i+1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx-1, nx-2) = 0.0;
    A(nx-1, nx-1) = 1.0;
    return A;
}

Eigen::VectorXd Spline::calc_B()
{
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for(int i=0; i<nx-2; i++){
        B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
    }
    return B;
}

int Spline::bisect(double t, int start, int end)
{
    int mid = (start+end)/2;
    if (t==x[mid] || end-start<=1){
        return mid;
    }else if (t>x[mid]){
        return bisect(t, mid, end);
    }else{
        return bisect(t, start, mid);
    }
}

// --------------------------------------------------
// Spline 2D
// --------------------------------------------------

Spline2D::Spline2D(std::vector<double> x, std::vector<double> y)
{
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
}

std::array<std::shared_ptr<double>, 2> Spline2D::calc_postion(double s_t)
{
    std::shared_ptr<double> x_ptr, y_ptr;
    x_ptr = sx.calc(s_t);
    y_ptr = sy.calc(s_t);
    if (x_ptr == nullptr || y_ptr == nullptr) {
        return {{nullptr, nullptr}};
    } 
    
    return {{x_ptr, y_ptr}};
};

std::shared_ptr<double> Spline2D::calc_curvature(double s_t)
{
    std::shared_ptr<double> dx_ptr, ddx_ptr, dy_ptr, ddy_ptr;
    dx_ptr = sx.calc_d(s_t);
    ddx_ptr = sx.calc_dd(s_t);
    dy_ptr = sy.calc_d(s_t);
    ddy_ptr = sy.calc_dd(s_t);

    if (dx_ptr == nullptr || ddx_ptr == nullptr || dy_ptr == nullptr || ddy_ptr == nullptr) {
        return nullptr;
    } 
    double dx = *dx_ptr; 
    double ddx = *ddx_ptr;
    double dy = *dy_ptr;
    double ddy = *ddy_ptr;
    double val = (ddy * dx - ddx * dy)/(dx * dx + dy * dy);
    std::shared_ptr<double> k = std::make_shared<double>(val);
    return k;
}

std::shared_ptr<double> Spline2D::calc_yaw(double s_t)
{
    double dx = *sx.calc_d(s_t);
    double dy = *sy.calc_d(s_t);
    double val = std::atan2(dy, dx);
    std::shared_ptr<double> yaw = std::make_shared<double>(val);
    return yaw;
}

std::vector<double> Spline2D::calc_s(std::vector<double> x, std::vector<double> y)
{
    std::vector<double> ds;
    std::vector<double> out_s{0};
    std::vector<double> dx = vec_diff(x);
    std::vector<double> dy = vec_diff(y);

    for(unsigned int i=0; i<dx.size(); i++){
        ds.push_back(std::sqrt(dx[i]*dx[i] + dy[i]*dy[i]));
    }

    std::vector<double> cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
};