#include "behavior.h"
#include "params.h"
#include "utility.h"

using namespace std;

vector<vector<double>> behavior_planner_find_targets(vector<vector<double>> &sensor_fusion, int prev_size, int lane, double car_s, double car_d, double ref_vel)
{
  vector<vector<double>> possible_next_states;

  bool too_close = false;
  
  // find ref_v to use
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d > get_dleft(lane) && d < get_dright(lane))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];
  
      // if using previous points can project s value outwards in time
      check_car_s+=((double)prev_size * param_dt * check_speed);
  
      if ((check_car_s > car_s) && ((check_car_s - car_s) < param_dist_slow_down))
      {
        // do some logic here: lower reference velocity so we dont crash into the car infront of us, could
        // also flag to try to change lane
        //ref_vel = 29.5; //mph
        too_close = true;
        
        //if (lane < 2)
        //{
        //  lane++;
        //}
  
        if (lane > 0)
        {
          lane--;
        }
        else
        {
          lane++;
        }
      }  
    }
  }
  
  if (too_close)
  {
    //ref_vel -= 2 * .224; // 5 m.s-2 under the 10 requirement
    ref_vel -= param_max_speed_inc_mph;
  }
  else if (ref_vel < param_max_speed_mph)
  {
    //ref_vel += 2 * .224;
    ref_vel += param_max_speed_inc_mph;
    ref_vel = min(ref_vel, param_max_speed_mph);
  }

  possible_next_states.push_back({(double)lane, ref_vel});
  return possible_next_states;
}

