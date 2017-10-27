#include "cost.h"
#include "utility.h"
#include "params.h"

#include <cassert>
#include <cmath>

using namespace std;


// check max speed, acceleration, jerk
bool check_max_capabilities(vector<vector<double>> &traj)
{
  double vx, ax, jx;
  double vy, ay, jy;
  double vel, acc, jerk;
  double max_vel = 0;
  double max_acc = 0;
  double total_jerk = 0;
  double x, x_1, x_2, x_3;
  double y, y_1, y_2, y_3;
  double jerk_per_second;

  assert(traj[0].size() == traj[1].size()); // as much x than y ...

  for (int t = 3; t < traj[0].size(); t++)
  {
    x   = traj[0][t];
    x_1 = traj[0][t-1];
    x_2 = traj[0][t-2];
    x_3 = traj[0][t-3];

    y   = traj[1][t];
    y_1 = traj[1][t-1];
    y_2 = traj[1][t-2];
    y_3 = traj[1][t-3];

    vx = (x - x_1) / param_dt;
    vy = (y - y_1) / param_dt;

    ax = (x - 2*x_1 + x_2) / (param_dt * param_dt);
    ay = (y - 2*y_1 + y_2) / (param_dt * param_dt);

    // rounding to 2 decimals (cm precision) to ensure numerical stability
    jx = (x - 3*x_1 + 3*x_2 - x_3);
    jx = roundf(jx * 100) / 100;
    jx = jx / (param_dt * param_dt * param_dt);
    jy = (y - 3*y_1 + 3*y_2 - y_3);
    jy = roundf(jy * 100) / 100;
    jy = jy / (param_dt * param_dt * param_dt);

    vel = sqrt(vx*vx + vy*vy);
    acc = sqrt(ax*ax + ay*ay);
    jerk = sqrt(jx*jx + jy*jy);

    total_jerk += jerk * param_dt;

    //cout << "jx=" << jx << " jy=" << jy << endl;
    //cout << "vel=" << vel << " acc=" << acc << " jerk=" << jerk << endl;

    if (vel > max_vel)
      max_vel = vel;
    if (acc > max_acc)
      max_acc = acc;
  }

  jerk_per_second = total_jerk / (param_nb_points * param_dt);

  if (roundf(max_vel) > param_max_speed || roundf(max_acc) > param_max_accel || jerk_per_second > param_max_jerk)
  {
    cout << "max_vel=" << max_vel << " max_acc=" << max_acc << " jerk_per_second=" << jerk_per_second  << endl;
    //assert(1 == 0);
    return true;
  }
  else
  {
    return false;
  }
}


double get_predicted_dmin(vector<vector<double>> &trajectory, std::map<int, vector<vector<double>>> &predictions)
{
  double dmin = 1e10;

  std::map<int, vector<vector<double> > >::iterator it = predictions.begin();
  while(it != predictions.end())
  {
    int fusion_index = it->first;
    //cout << "fusion_index=" << fusion_index << endl;
    vector<vector<double>> prediction = it->second;

    assert(prediction.size() == trajectory[0].size());
    assert(prediction.size() == trajectory[1].size());

    for (int i = 0; i < prediction.size(); i++) // up to 50 (x,y) coordinates
    {
      double obj_x = prediction[i][0];
      double obj_y = prediction[i][1];
      double ego_x = trajectory[0][i];
      double ego_y = trajectory[1][i];

      double dist = distance(ego_x, ego_y, obj_x, obj_y);
      if (dist < dmin)
      {
        dmin = dist;
      }
      //cout << "dist[" << i << "]=" << dist << endl; 
      if (dist <= param_dist_collision)
      {
        cout << "=====> DMIN = " << dmin << endl;
        cout << "predicted collision in " << i << " steps with fusion_index " << fusion_index << " (dist=" << dist << ")" << endl;
        //assert(1 == 0); // TODO temp just for checking purposes
      }
    }
    it++;
  }

  cout << "=====> dmin = " << dmin << endl;
  return dmin;
}


double cost_function(vector<vector<double>> &trajectory, int target_lane, double target_vel, std::map<int, vector<vector<double>>> &predictions)
{
  double cost = 0; // lower cost preferred

  double cost_feasibility = 0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal

  double weight_feasibility = 10000; // vs collisions, vs vehicle capabilities
  double weight_safety      = 1000; // vs buffer distance, vs visibility or curvature
  double weight_legality    = 100; // vs speed limits
  double weight_comfort     = 10; // vs jerk
  double weight_efficiency  = 1; // vs target lane, target speed and time to goal

  // 1) FEASIBILITY cost
  // TODO: handled via safety so far
  //if (check_collision(trajectory, predictions))
  //{
  //  cost_feasibility += 10;
  //}
  //if (check_max_capabilities(trajectory))
  //{
  //  cost_feasibility += 1;
  //}
  cost = cost + weight_feasibility * cost_feasibility;

  // 2) SAFETY cost
  double dmin = get_predicted_dmin(trajectory, predictions);
  assert(dmin >= 0);
  if (dmin < param_dist_safety)
  {
    cost_safety = param_dist_safety - dmin;
  }
  else
  {
    cost_safety = 0;
  }
  cost = cost + weight_safety * cost_safety;

  // 3) LEGALITY cost
  cost = cost + weight_legality * cost_legality;

  // 4) COMFORT cost
  cost = cost + weight_comfort * cost_comfort;

  // 5) EFFICIENCY cost
  cost_efficiency = param_max_speed_mph - target_vel;
  cost = cost + weight_efficiency * cost_efficiency;

  return cost;

}
