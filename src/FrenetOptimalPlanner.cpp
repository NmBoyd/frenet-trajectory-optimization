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
    f_traj_list = calcFrenetTrajectoriesGlobal(f_traj_list, c_spline);
    f_traj_list = checkTrajectories(f_traj_list, obstacles);

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
    // std::cout << "Generated " << traj_list.size() << " trajectories" << std::endl;
    return traj_list;
}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::calcFrenetTrajectoriesGlobal(
    std::vector<FrenetTrajectory> f_traj_list, Spline2D c_spline)
{
    for (auto& f_traj : f_traj_list)
    {
        for (int i=0; i < f_traj.s.size(); i++)
        {
            // std::cout << "calculating global xy pos at "<< f_traj.s[i] << std::endl;
            std::array<std::shared_ptr<double>, 2> pos_ptr = c_spline.calc_postion(f_traj.s[i]); 
            if (pos_ptr[0] == nullptr || pos_ptr[1] == nullptr) {
                // std::cout << "nullptr break on " << i <<std::endl;
                break;
            } 
            std::array<double,2> pos;
            pos[0] = *pos_ptr[0];
            pos[1] = *pos_ptr[1];
            double yaw = *c_spline.calc_yaw(f_traj.s[i]);
            double d = f_traj.d[i];
            double x_global = pos[0] + d * cos(yaw + M_PI_2);
            double y_global = pos[1] + d * sin(yaw + M_PI_2);
            f_traj.global_path.x.push_back(x_global);
            f_traj.global_path.y.push_back(y_global);
        }

        for (int i=0; i < f_traj.global_path.x.size()-1; i++)
        {
            double dx = f_traj.global_path.x[i+1] - f_traj.global_path.x[i];
            double dy = f_traj.global_path.y[i+1] - f_traj.global_path.y[i];
            f_traj.global_path.yaw.push_back(atan2(dy,dx));
            f_traj.delta_s.push_back(hypot(dx,dy));
        }
        f_traj.global_path.yaw.push_back(f_traj.global_path.yaw.back());
        f_traj.delta_s.push_back(f_traj.delta_s.back());

        for (int i=0;i<f_traj.global_path.yaw.size()-1; i++)
        {
            f_traj.global_path.k.push_back((f_traj.global_path.yaw[i+1]-f_traj.global_path.yaw[i])/f_traj.delta_s[i]);
        }
    }
    return f_traj_list;
}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::checkTrajectories(
    std::vector<FrenetTrajectory> f_traj_list, std::vector<Vector2d> obstacles)
{
    std::vector<FrenetTrajectory> ok_traj_list;
    for (auto& traj : f_traj_list)
    {
        bool flagged = false;

        for (int i=0; i < traj.s_d.size(); i++) {
            if (traj.s_d[i] > params_.max_speed) {
                // std::cout << "Speed Violation: removing trjectory: " << traj.s_d[i] << ">" << params_.max_speed << std::endl;
                flagged = true;
            }
            else if (abs(traj.s_dd[i]) > params_.max_accel) {
                // std::cout << "Accel Violation: removing trjectory: " << traj.s_dd[i] << ">" << params_.max_accel << std::endl;
                flagged = true;
            }
            else if (abs(traj.global_path.k[i]) > params_.max_curvature) {
                // std::cout << "Curvature Violation: removing trjectory: " << traj.global_path.k[i] << ">" << params_.max_curvature << std::endl;
                flagged = true;
            }
            else if (collision(traj, obstacles)) {
                // std::cout << "Collision Violation: removing trjectory: " << traj.global_path.x[1] << ", " << traj.global_path.y[1] << std::endl;
                flagged = true;
            }
        }
        if (!flagged) {
            ok_traj_list.push_back(traj);
        }
    }
    return ok_traj_list;
}

bool FrenetOptimalPlanner::collision(FrenetTrajectory traj, std::vector<Vector2d> obstacles)
{
    for (auto& obstacle : obstacles) 
    {
        for (int i=0; i < traj.global_path.x.size(); i++)
        {
            double dist = pow((traj.global_path.x[i]-obstacle[0]),2) +
                                pow((traj.global_path.y[i]-obstacle[1]),2);
            bool collision = dist <= pow(params_.hull_radius, 2) ? true : false;
            if (collision) {
                // std::cout << "Collision: Distance value " << dist << std::endl;
                return true;
            }
        }
    }
    return false;
}