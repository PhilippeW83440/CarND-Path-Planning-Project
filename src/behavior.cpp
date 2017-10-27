#include "behavior.h"
#include "params.h"
#include "utility.h"
#include <cassert>

using namespace std;

vector<vector<double>> behavior_planner_find_targets(vector<vector<double>> &sensor_fusion, int prev_size, int lane, double car_s, double car_d, double ref_vel)
{
  vector<vector<double>> possible_targets;
  bool too_close = false;
  int ref_vel_inc = 0; // -1 for max deceleration, 0 for constant speed, +1 for max acceleration
  
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
    ref_vel = max(ref_vel, 0.0); // no backwards driving ... just in case
    ref_vel_inc = -1;
  }
  else if (ref_vel < param_max_speed_mph)
  {
    //ref_vel += 2 * .224;
    ref_vel += param_max_speed_inc_mph;
    ref_vel = min(ref_vel, param_max_speed_mph);
    ref_vel_inc = +1;
  }

  // our nominal target
  possible_targets.push_back({(double)lane, ref_vel});

  ///////////////////////////////////////////////////////////////////
  // Backup targets
  ///////////////////////////////////////////////////////////////////

  vector<int> backup_lanes;
  switch (lane)
  {
    case 2:
      backup_lanes.push_back(1);
      break;
    case 1:
      backup_lanes.push_back(0);
      backup_lanes.push_back(2);
      break;
    case 0:
      backup_lanes.push_back(1);
      break;
    default:
      assert(1 == 0); // something went wrong
      break;
  }

  vector<double> backup_vel; // only lower speeds so far ...
  switch (ref_vel_inc)
  {
    case 1:
      backup_vel.push_back(ref_vel - param_max_speed_inc_mph);
      backup_vel.push_back(ref_vel - 2 * param_max_speed_inc_mph);
      break;
    case 0: // already max speed
      backup_vel.push_back(ref_vel - param_max_speed_inc_mph);
      break;
    case -1:
      // emergency breaking
      backup_vel.push_back(ref_vel - param_max_speed_inc_mph);

      // emergency acceleration (dangerous here)
      //backup_vel.push_back(ref_vel + param_max_speed_inc_mph);
      break;
    default:
      assert(1 == 0); // something went wrong
      break;
  }

  if (param_trajectory_jmt == false) // TODO temporary
  {
    // 1) target velocity on backup lanes
    for (int i = 0; i < backup_lanes.size(); i++)
    {
      possible_targets.push_back({(double)backup_lanes[i], ref_vel});
    }

    // 2) backup velocities on target lane
    for (int i = 0; i < backup_vel.size(); i++)
    {
      possible_targets.push_back({(double)lane, backup_vel[i]});
    }

    // 2) backup velocities on backup lanes
    for (int i = 0; i < backup_vel.size(); i++)
    {
      for (int j = 0; j < backup_lanes.size(); j++)
      {
        possible_targets.push_back({(double)backup_lanes[j], backup_vel[i]});
      }
    }
  }

  ///////////////////////////////////////////////////////////////////

  return possible_targets;
}
