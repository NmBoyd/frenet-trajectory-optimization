#include "FrenetOptimalPlanner.h"

FrenetOptimalPlanner::FrenetOptimalPlanner()
{
}

FrenetOptimalPlanner::FrenetOptimalPlanner(FrenetOptimalPlanner::Params params)
{
    params_ = params;
}

FrenetOptimalPlanner::~FrenetOptimalPlanner()
{

}

FrenetTrajectory FrenetOptimalPlanner::calcOptimalMotionPlan(
    Spline2D c_spline, FrenetState f_state, std::vector<Vector2d> obstacles)
{
    std::vector<FrenetTrajectory> f_traj_list = calcFrenetTrajectory(f_state);
    f_traj_list = calcFrenetTrajectoryGlobal(f_traj_list, c_spline);
    f_traj_list = checkTrajectoryCollisions(f_traj_list, obstacles);

    double min_cost = numeric_limits<double>::max();
    FrenetTrajectory best_traj;
    for (const auto& traj : f_traj_list)
    {
        if (min_cost >= traj.cost_total) {
            min_cost = traj.cost_total;
            best_traj = traj;
        }
    }    
    return best_traj;
}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::calcFrenetTrajectory(
    FrenetState curr_state)
{

    double kj = params_.kj;
    double kt = params_.kt;
    double kd = params_.kd;
    double klat = params_.klat;
    double klon = params_.klon;
    std::vector<FrenetTrajectory> traj_list;

    // make sure that the outer loop can handle lane merge scenarios properly
    for (double lane=-params_.max_road_width; lane<params_.max_road_width; lane+=params_.d_road_w)
    {
        // Lateral motion planning (steering)
        for (double Ti=params_.min_prediction_t; Ti < params_.max_predicition_t; Ti+=params_.dt)
        {
            FrenetTrajectory f_traj;
            QuinticPolynomial qp_lat(curr_state.d, curr_state.d_d, curr_state.d_dd, lane, 0.0, 0.0, Ti);
            
            // lateral spline calc
            for (double t=0.0; t<Ti; t+=params_.dt) {
                f_traj.t.push_back(t);
                f_traj.d.push_back(qp_lat.GetPosition(t));
                f_traj.d_d.push_back(qp_lat.GetVelocity(t));
                f_traj.d_dd.push_back(qp_lat.GetAcceleration(t));
                f_traj.d_ddd.push_back(qp_lat.GetJerk(t));
            } 

            // Longitudinal motion planning (velocity keeping)
            double min_variance_speed = params_.target_speed-params_.d_t_s*params_.n_s_sample;
            double max_variance_speed = params_.target_speed+params_.d_t_s*params_.n_s_sample;
            for (double tv=min_variance_speed; tv<max_variance_speed; tv+=params_.d_t_s)
            {
                FrenetTrajectory t_f_traj = f_traj;
                QuarticPolynomial qp_lon(curr_state.s, curr_state.speed, 0.0, tv, 0.0, Ti);

                for (double t : f_traj.t)
                {
                    t_f_traj.s.push_back(qp_lon.GetPosition(t));
                    t_f_traj.s_d.push_back(qp_lon.GetVelocity(t));
                    t_f_traj.s_dd.push_back(qp_lon.GetAcceleration(t));
                    t_f_traj.s_ddd.push_back(qp_lon.GetJerk(t));
                }

                // lateral and longitudinal square of jerk
                double Jp = std::inner_product( t_f_traj.d_ddd.begin(),
                                            t_f_traj.d_ddd.end(), 
                                            t_f_traj.d_ddd.begin(),
                                            0 );

                double Js = std::inner_product( t_f_traj.s_ddd.begin(),
                                            t_f_traj.s_ddd.end(), 
                                            t_f_traj.s_ddd.begin(),
                                            0 );

                double ds = pow((params_.target_speed - t_f_traj.s_d.back()),2);


                t_f_traj.cost_lat = kj*Jp + kt*Ti + kd*pow(t_f_traj.d.back(), 2);
                t_f_traj.cost_lon = kj*Js + kt*Ti + kd*ds;
                t_f_traj.cost_total = (klat*t_f_traj.cost_lat) + (klon*t_f_traj.cost_lon);

                traj_list.push_back(t_f_traj);
            }
        }
    }
    return traj_list;
}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::calcFrenetTrajectoryGlobal(
    std::vector<FrenetTrajectory> f_traj, Spline2D c_spline)
{

}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::checkTrajectoryCollisions(
    std::vector<FrenetTrajectory> f_traj, std::vector<Vector2d> obstacles)
{

}