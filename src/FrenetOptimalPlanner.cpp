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
    // make sure that the outer loop can handle lane merge scenarios properly
    for (double lane=-params_.max_road_width; lane<params_.max_road_width; lane+=params_.d_road_w)
    {
        // Lateral motion planning
        // for (double )
    }
}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::calcFrenetTrajectoryGlobal(
    std::vector<FrenetTrajectory> f_traj, Spline2D c_spline)
{

}

std::vector<FrenetTrajectory> FrenetOptimalPlanner::checkTrajectoryCollisions(
    std::vector<FrenetTrajectory> f_traj, std::vector<Vector2d> obstacles)
{

}