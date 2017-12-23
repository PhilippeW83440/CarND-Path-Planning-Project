#include "behavior.h"
#include "params.h"
#include "utility.h"
#include <cassert>

using namespace std;

vector<vector<double>> behavior_planner_find_targets(vector<vector<double>> &sensor_fusion, int lane, double car_s, double car_d, double ref_vel)
{
  vector<vector<double>> possible_targets;
  bool too_close = false;
  int ref_vel_inc = 0; // -1 for max deceleration, 0 for constant speed, +1 for max acceleration
  double dist_safety = PARAM_DIST_SLOW_DOWN;

  double ref_vel_ms = mph_to_ms(ref_vel);
  double closest_speed_ms = INF;
  double closest_dist = INF;
  
  // find ref_v to use based on car in front of us
  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d > get_dleft(lane) && d < get_dright(lane)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];

      cout << "obj_idx=" << i << " REF_VEL_MS=" << ref_vel_ms << " CHECK_SPEED=" << check_speed << endl;
      dist_safety = PARAM_DIST_SLOW_DOWN;
      if (fabs(ref_vel_ms - check_speed) <= 2)
        dist_safety = 20; // XXX TODO remove harcoded value
  
      if ((check_car_s > car_s) && ((check_car_s - car_s) < dist_safety)) {
        // do some logic here: lower reference velocity so we dont crash into the car infront of us
        //ref_vel = 29.5; //mph
        too_close = true;
        double dist_to_check_car_s = check_car_s - car_s;
        if (dist_to_check_car_s < closest_dist) {
          closest_dist = dist_to_check_car_s;
          closest_speed_ms = check_speed;
        }
      }  
    }
  }
  
  if (too_close) {
    //ref_vel -= 2 * .224; // 5 m.s-2 under the 10 requirement
    if (ref_vel_ms > closest_speed_ms) { // in m.s-1 !
      ref_vel -= PARAM_MAX_SPEED_INC_MPH; // in mph !
      if (closest_dist <= 10 && ref_vel > closest_speed_ms) {
        ref_vel -= 5 * PARAM_MAX_SPEED_INC_MPH;
      }
    }

    ref_vel = max(ref_vel, 0.0); // no backwards driving ... just in case
    ref_vel_inc = -1;
  } else if (ref_vel < PARAM_MAX_SPEED_MPH) {
    //ref_vel += 2 * .224;
    ref_vel += PARAM_MAX_SPEED_INC_MPH;
    ref_vel = min(ref_vel, PARAM_MAX_SPEED_MPH);
    ref_vel_inc = +1;
  }

  // our nominal target .. same lane
  possible_targets.push_back({(double)lane, ref_vel});

  // Backup targets (lane and speed)

  vector<int> backup_lanes;
  switch (lane)
  {
    case 2:
      backup_lanes.push_back(1);
      break;
    case 1:
      backup_lanes.push_back(2);
      backup_lanes.push_back(0);
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
      backup_vel.push_back(ref_vel - PARAM_MAX_SPEED_INC_MPH);
      backup_vel.push_back(ref_vel - 2 * PARAM_MAX_SPEED_INC_MPH);
      break;
    case 0: // already max speed
      backup_vel.push_back(ref_vel - PARAM_MAX_SPEED_INC_MPH);
      break;
    case -1:
      // emergency breaking
      backup_vel.push_back(ref_vel - PARAM_MAX_SPEED_INC_MPH);

      // emergency acceleration (dangerous here)
      //backup_vel.push_back(ref_vel + PARAM_MAX_SPEED_INC_MPH);
      break;
    default:
      assert(1 == 0); // something went wrong
      break;
  }

  // 1) backup velocities on target lane
  for (size_t i = 0; i < backup_vel.size(); i++) {
    possible_targets.push_back({(double)lane, backup_vel[i]});
  }

  // 2) target velocity on backup lanes
  for (size_t i = 0; i < backup_lanes.size(); i++) {
    possible_targets.push_back({(double)backup_lanes[i], ref_vel});
  }

  // 2) backup velocities on backup lanes
  for (size_t i = 0; i < backup_vel.size(); i++) {
    for (size_t j = 0; j < backup_lanes.size(); j++) {
      possible_targets.push_back({(double)backup_lanes[j], backup_vel[i]});
    }
  }


  return possible_targets;
}
