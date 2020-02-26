#include "PolynomialTraj.h"


QuinticPolynomial::QuinticPolynomial(double x_init, 
  double v_init, 
  double a_init,
  double x_end, 
  double v_end, 
  double a_end, 
  double time_end)
{
  a_[0] = x_init;
  a_[1] = v_init;
  a_[2] = a_init / 2;

  double t = time_end;
  Eigen::Matrix3d A;
  A << pow(t, 3.), pow(t, 4.), pow(t, 5.),
      3*pow(t, 2.), 4*pow(t, 3.), 5*pow(t,4.),
      6*t, 12*pow(t, 2.), 20*pow(t, 3.);

  Eigen::Vector3d b;
  b << x_end - (a_[0] + a_[1] * t + a_[2] * pow(t, 2.)),
      v_end - (a_[1] + 2 * a_[2] * t),
      a_end - 2 * a_[2];

  Eigen::Vector3d x;
  x = A.inverse() * b; // TODO: use a function to do this faster
  a_[3] = x[0];
  a_[4] = x[1];
  a_[5] = x[2];

  // std::cout << "Quintic Polynomial Coefficients: " << x << std::endl;
}

QuinticPolynomial::~QuinticPolynomial(){
  // std::cout << "Destructing polynomial" << std::endl;
}

double QuinticPolynomial::GetPosition(double t)
{
  double xt = a_[0] + a_[1] * t + a_[2] * pow(t,2) + 
        a_[3] * pow(t,3) + a_[4] * pow(t,4) + a_[5] * pow(t,5);
  return xt;
}

double QuinticPolynomial::GetVelocity(double t)
{
  double vt = a_[1] + 2 * a_[2] * t + 3 * a_[3] * pow(t,2) +
                4 * a_[4] * pow(t,3) + 5 * a_[5] * pow(t,4);
  return vt;
}

double QuinticPolynomial::GetAcceleration(double t)
{
  double at = 2 * a_[2] + 6 * a_[3] * t + 12 * a_[4] * pow(t,2)
               + 20 * a_[5] * pow(t,3);
  return at;
}

double QuinticPolynomial::GetJerk(double t)
{
  double jt = 6 * a_[3] + 24 * a_[4] * t + 60 * a_[5] * pow(t,2);
  return jt;
}

bool QuinticPolynomialPlanFrenet(Trajectory2d& traj, double sx, double sy, double syaw,
  double sv, double sa, double gx, double gy, double gyaw, double gv, double ga, double max_accel,
  double max_jerk, double dt, double min_time, double max_time)
{
  double vxs = sv * cos(syaw);
  double vys = sv * sin(syaw);
  double vxg = gv * cos(gyaw);
  double vyg = gv * sin(gyaw);

  double axs = sa * cos(syaw);
  double ays = sa * sin(syaw);
  double axg = ga * cos(gyaw);
  double ayg = ga * sin(gyaw);

  for (double T = min_time; T < max_time; T += min_time)
  {
    traj = Trajectory2d(); 
    QuinticPolynomial xqp(sx, vxs, axs, gx, vxg, axg, T);
    QuinticPolynomial yqp(sy, vys, ays, gy, vyg, ayg, T);

    for (double t = 0.0; t < T+dt; t+=dt)
    {
        traj.time_list.push_back(t);
        traj.x_list.push_back(xqp.GetPosition(t));
        traj.y_list.push_back(yqp.GetPosition(t));

        double vx = xqp.GetVelocity(t);
        double vy = yqp.GetVelocity(t);
        double v = sqrt(pow(vx,2) + pow(vy,2));
        double yaw = atan2(vy, vx);
        traj.vel_list.push_back(v);
        traj.yaw_list.push_back(yaw);

        double ay = yqp.GetAcceleration(t);
        double ax = xqp.GetAcceleration(t);
        double a = sqrt(pow(ax,2) + pow(ay,2));
        if (traj.vel_list.size() >= 2 && 
          traj.vel_list.back() - traj.vel_list.at(traj.vel_list.size()-2) < 0.0)
        {
          a *= -1;
        }
        traj.accel_list.push_back(a);

        double jx = xqp.GetJerk(t);
        double jy = yqp.GetJerk(t);
        double j = sqrt(pow(jx,2) + pow(jy,2));
        if (traj.accel_list.size() >= 2 && 
          traj.accel_list.back() - traj.accel_list.at(traj.accel_list.size()-2) < 0.0)
        {
            j *= -1;
        }
        traj.jerk_list.push_back(j);
    }

    double max_accel_check = std::abs(traj.accel_list[0]);
    double max_jerk_check = std::abs(traj.jerk_list[0]);
    for (int i = 0; i < traj.accel_list.size(); i++ ) {
      if(std::abs(traj.accel_list[i]) > max_accel_check){
        max_accel_check = std::abs(traj.accel_list[i]);
      }
      if(std::abs(traj.jerk_list[i]) > max_jerk_check){
        max_jerk_check = std::abs(traj.jerk_list[i]);
      }
    }
    if (max_accel_check <= max_accel && max_jerk_check <= max_jerk){
      std::cout << ("find path!!") << std::endl;
      break;
    }
        
  }
  return true;
}

// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------
// QUARTIC POLYNOMIAL
// -----------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------

QuarticPolynomial::QuarticPolynomial(double x_init, 
  double v_init, 
  double a_init,
  double v_end, 
  double a_end, 
  double time_end)
{
  a_[0] = x_init;
  a_[1] = v_init;
  a_[2] = a_init / 2;

  double t = time_end;
  Eigen::Matrix2d A;
  A << 3*pow(t, 2.), 4*pow(t, 3.), 
      6*t, 12*pow(t, 2.);

  Eigen::Vector2d b;
  b << v_end - (a_[1] + 2 * a_[2] * t),
      a_end - 2 * a_[2];

  Eigen::Vector2d x;
  x = A.inverse() * b; // TODO: use a function to do this faster
  a_[3] = x[0];
  a_[4] = x[1];
}

QuarticPolynomial::~QuarticPolynomial()
{
}

double QuarticPolynomial::GetPosition(double t)
{
  double xt = a_[0] + a_[1] * t + a_[2] * pow(t,2) + 
        a_[3] * pow(t,3) + a_[4] * pow(t,4);
  return xt;
}

double QuarticPolynomial::GetVelocity(double t)
{
  double vt = a_[1] + 2 * a_[2] * t + 3 * a_[3] * pow(t,2) +
                4 * a_[4] * pow(t,3);
  return vt;
}

double QuarticPolynomial::GetAcceleration(double t)
{
  double at = 2 * a_[2] + 6 * a_[3] * t + 12 * a_[4] * pow(t,2);
  return at;
}

double QuarticPolynomial::GetJerk(double t)
{
  double jt = 6 * a_[3] + 24 * a_[4] * t;
  return jt;
}