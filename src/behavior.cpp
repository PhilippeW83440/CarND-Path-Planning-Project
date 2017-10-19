#include "behavior.h"

vector<vector<double>> behavior_planner_find_targets(vector<vector<double>> sensor_fusion, int prev_size, int lane, double car_s, double car_d, double ref_vel)
{
  vector<vector<double>> possible_next_states;

  bool too_close = false;
  
  // find ref_v to use
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d > (2+4*lane-2) && d <(2+4*lane+2))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];
  
      // if using previous points can project s value outwards in time
      check_car_s+=((double)prev_size*0.02*check_speed);
  
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
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
    ref_vel -= 2 * .224; // 5 m.s-2 under the 10 requirement
  }
  else if (ref_vel < 49.5)
  {
    ref_vel += 2 * .224;
  }

  possible_next_states.push_back({(double)lane, ref_vel});
  return possible_next_states;
}

