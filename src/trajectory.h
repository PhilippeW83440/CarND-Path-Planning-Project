#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

#include "map.h"


// INPUTS:
//    target       : lane, ref_vel
//    map          : map_waypoints_s, map_waypoints_x, map_waypoints_y
//    car          : car_x, car_y, car_yaw, car_s, car_d
//    previous_path: previous_path_x, previous_path_y


// OUTPUTS:
//    trajectory: next_x_vals, next_y_vals
std::vector<std::vector<double>> generate_trajectory(int target_lane, double target_vel, Map &map, double car_x, double car_y, double car_yaw, double car_s, double car_d, std::vector<double> pervious_path_x, std::vector<double> previous_path_y);

#endif // TRAJECTORY_H
