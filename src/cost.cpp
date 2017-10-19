#include "cost.h"

double cost_function(vector<vector<double>> trajectory, int target_lane, double target_vel, double target_time)
{
  double cost = 0; // lower cost preferred

  double cost_feasibility = 0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal

  double weight_feasibility = 100000; // vs collisions, vs vehicle capabilities
  double weight_safety      = 1000; // vs buffer distance, vs visibility or curvature
  double weight_legality    = 100; // vs speed limits
  double weight_comfort     = 10; // vs jerk
  double weight_efficiency  = 1; // vs target lane, target speed and time to goal

  // 1) FEASIBILITY cost
  //if (check_collision(predictions, pred_lane, 10))
  //{
  //  cost_feasibility = 10;
  //}
  cost = cost + weight_feasibility * cost_feasibility;

  // 2) SAFETY cost
  cost = cost + weight_safety * cost_safety;

  // 3) LEGALITY cost
  cost = cost + weight_legality * cost_legality;

  // 4) COMFORT cost
  cost = cost + weight_comfort * cost_comfort;

  // 5) EFFICIENCY cost
  cost = cost + weight_efficiency * cost_efficiency;

  return cost;

}
