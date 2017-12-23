#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

#include "map.h"
#include "behavior.h"


struct trajectory_jmt {
  std::vector<std::vector<double>> trajectory;
  std::vector<std::vector<double>> path_s;
  std::vector<std::vector<double>> path_d;
};

struct trajectory_jmt JMT_init(double car_s, double car_d);

// INPUTS:
//    target       : lane, ref_vel
//    map          : map_waypoints_s, map_waypoints_x, map_waypoints_y
//    car          : car_x, car_y, car_yaw, car_s, car_d
//    previous_path: previous_path_x, previous_path_y


// OUTPUTS:
//    trajectory: next_x_vals, next_y_vals
std::vector<std::vector<double>> generate_trajectory(Target target, Map &map, double car_x, double car_y, double car_yaw, double car_s, double car_d, std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, int prev_size);

struct trajectory_jmt generate_trajectory_jmt(Target target, Map &map, std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, int prev_size, std::vector<std::vector<double>> &ref_path_s, std::vector<std::vector<double>> &ref_path_d);

#endif // TRAJECTORY_H
