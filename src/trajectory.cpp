#include "trajectory.h"
#include "spline.h"
#include "coord.h"
#include "params.h"

vector<vector<double>> generate_trajectory(int lane, double ref_vel, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, double car_x, double car_y, double car_yaw, double car_s, double car_d, vector<double> previous_path_x, vector<double> previous_path_y)
{
  vector<double> ptsx;
  vector<double> ptsy;
  
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  int prev_size = previous_path_x.size();
  
  if (prev_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
  
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
  
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
  
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
  
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
  
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  vector<double> next_wp0 = getXY(car_s+30, get_dcenter(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, get_dcenter(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, get_dcenter(lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  
  for (int i = 0; i < ptsx.size(); i++)
  {
    // shift car reference angle to 0 degrees
    // transformation to local car's coordinates (cf MPC)
    // last point of previous path at origin and its angle at zero degree
  
    // shift and rotation
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
  
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
  
  
  tk::spline spl;
  spl.set_points(ptsx, ptsy);
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  for (int i = 0; i < prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  
  double x_add_on = 0;
  
  // fill up the rest of our path planner after filing it with previous points
  // here we will always output 50 points
  for (int i = 1; i <= param_nb_points - prev_size; i++)
  {
    double N = (target_dist / (param_dt * mph_to_ms(ref_vel))); // divide by 2.24: mph -> m/s
    double x_point = x_add_on + target_x/N;
    double y_point = spl(x_point);
  
    x_add_on = x_point;
  
    double x_ref = x_point; // x_ref IS NOT ref_x !!!
    double y_ref = y_point;
  
    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
  
    x_point += ref_x;
    y_point += ref_y;
  
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return { next_x_vals, next_y_vals };
}
