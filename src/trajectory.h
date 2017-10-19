#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

// INPUTS:
//    target       : lane, ref_vel
//    map          : map_waypoints_s, map_waypoints_x, map_waypoints_y
//    car          : car_x, car_y, car_yaw, car_s, car_d
//    previous_path: previous_path_x, previous_path_y


// OUTPUTS:
//    trajectory: next_x_vals, next_y_vals
vector<vector<double>> generate_trajectory(int lane, double ref_vel, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, double car_x, double car_y, double car_yaw, double car_s, double car_d, vector<double> pervious_path_x, vector<double> previous_path_y);

#endif // TRAJECTORY_H
