#include "behavior.h"

using namespace std;

Behavior::Behavior(vector<vector<double>> const &sensor_fusion, CarData car) {
  Target target;
  target.time = 2.0;

  bool too_close = false;
  int ref_vel_inc = 0; // -1 for max deceleration, 0 for constant speed, +1 for max acceleration
  double dist_safety = PARAM_DIST_SLOW_DOWN;

  double ref_vel_ms = mph_to_ms(car.speed_target);
  double closest_speed_ms = INF;
  double closest_dist = INF;
  
  // find ref_v to use based on car in front of us
  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d > get_dleft(car.lane) && d < get_dright(car.lane)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];

      cout << "obj_idx=" << i << " REF_VEL_MS=" << ref_vel_ms << " CHECK_SPEED=" << check_speed << endl;
      dist_safety = PARAM_DIST_SLOW_DOWN;
      if (fabs(ref_vel_ms - check_speed) <= 2)
        dist_safety = 20; // XXX TODO remove harcoded value
  
      if ((check_car_s > car.s) && ((check_car_s - car.s) < dist_safety)) {
        // do some logic here: lower reference velocity so we dont crash into the car infront of us
        //ref_vel = 29.5; //mph
        too_close = true;
        double dist_to_check_car_s = check_car_s - car.s;
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
      car.speed_target -= PARAM_MAX_SPEED_INC_MPH; // in mph !
      if (closest_dist <= 10 && car.speed_target > closest_speed_ms) {
        car.speed_target -= 5 * PARAM_MAX_SPEED_INC_MPH;
      }
    }

    car.speed_target = max(car.speed_target, 0.0); // no backwards driving ... just in case
    ref_vel_inc = -1;
  } else if (car.speed_target < PARAM_MAX_SPEED_MPH) {
    //ref_vel += 2 * .224;
    car.speed_target += PARAM_MAX_SPEED_INC_MPH;
    car.speed_target = min(car.speed_target, PARAM_MAX_SPEED_MPH);
    ref_vel_inc = +1;
  }

  // our nominal target .. same lane
  target.lane = car.lane;
  target.velocity = car.speed_target;

#if 0
  // XXX TEMP just for testing
  target.velocity = closest_speed_ms;
  target.accel = 0.9 * PARAM_MAX_ACCEL;
  double car_speed_ms = mph_to_ms(car.speed);
  if (closest_speed_ms < car_speed_ms)
    target.accel *= -1.0;
  cout << "!!!!! target: velocity=" << target.velocity << " accel=" << target.accel << '\n';
#endif

  targets_.push_back(target);

#if 0
  return; // XXX TEMP just for testing
#endif

  // Backup targets (lane and speed)

  vector<int> backup_lanes;
  switch (car.lane)
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
      backup_vel.push_back(car.speed_target - PARAM_MAX_SPEED_INC_MPH);
      backup_vel.push_back(car.speed_target - 2 * PARAM_MAX_SPEED_INC_MPH);
      break;
    case 0: // already max speed
      backup_vel.push_back(car.speed_target - PARAM_MAX_SPEED_INC_MPH);
      break;
    case -1:
      // emergency breaking
      backup_vel.push_back(car.speed_target - PARAM_MAX_SPEED_INC_MPH);

      // emergency acceleration (dangerous here)
      //backup_vel.push_back(car.speed_target + PARAM_MAX_SPEED_INC_MPH);
      break;
    default:
      assert(1 == 0); // something went wrong
      break;
  }

  // 1) backup velocities on target lane
  target.lane = car.lane;
  for (size_t i = 0; i < backup_vel.size(); i++) {
    target.velocity = backup_vel[i];
    targets_.push_back(target);
  }

  // 2) target velocity on backup lanes
  target.velocity = car.speed_target;
  for (size_t i = 0; i < backup_lanes.size(); i++) {
    target.lane = backup_lanes[i];
    targets_.push_back(target);
  }

  // 2) backup velocities on backup lanes
  for (size_t i = 0; i < backup_vel.size(); i++) {
    target.velocity = backup_vel[i];
    for (size_t j = 0; j < backup_lanes.size(); j++) {
      target.lane = backup_lanes[j];
      targets_.push_back(target);
    }
  }
}

Behavior::~Behavior() {}

vector<Target> Behavior::get_targets() {
  return targets_;
}
