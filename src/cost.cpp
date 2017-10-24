#include "cost.h"
#include "utility.h"
#include "params.h"

#include <cassert>

using namespace std;

bool check_collision(vector<vector<double>> &trajectory, std::map<int, vector<vector<double>>> &predictions)
{
  std::map<int, vector<vector<double> > >::iterator it = predictions.begin();
  while(it != predictions.end())
  {
    int obj_id = it->first;
    cout << "obj_id=" << obj_id << endl;
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
      cout << "dist[" << i << "]=" << dist << endl; 
      if (dist <= param_dist_collision)
      {
        cout << "collision in " << i << " steps with fusion obj_id " << obj_id << " (dist=" << dist << ")" << endl;
        assert(1 == 0);
        return true;
      }
    }
    it++;
  }
  return false;
}


double cost_function(vector<vector<double>> &trajectory, int target_lane, double target_vel, std::map<int, vector<vector<double>>> &predictions)
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
  if (check_collision(trajectory, predictions))
  {
    cost_feasibility = 10;
  }
  cost = cost + weight_feasibility * cost_feasibility;

  // 2) SAFETY cost
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
