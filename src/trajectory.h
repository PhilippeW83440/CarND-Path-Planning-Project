#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

#include "map.h"
#include "behavior.h"


// Point of a C2 class function
struct PointC2 {
  double f;
  double f_dot;
  double f_ddot;
  PointC2 (double y=0, double y_dot=0, double y_ddot=0) : f(y), f_dot(y_dot), f_ddot(y_ddot) {}
};

struct TrajectoryXY {
  std::vector<double> x_vals;
  std::vector<double> y_vals;
  TrajectoryXY (std::vector<double> X={}, std::vector<double> Y={}) : x_vals(X), y_vals(Y) {}
};

struct TrajectorySD {
  std::vector<PointC2> path_s;
  std::vector<PointC2> path_d;
  TrajectorySD (std::vector<PointC2> S={}, std::vector<PointC2> D={}) : path_s(S), path_d(D) {}
};

struct TrajectoryJMT {
  TrajectoryXY trajectory;
  TrajectorySD path_sd;
};

TrajectoryJMT JMT_init(double car_s, double car_d);

// INPUTS:
//    target       : lane, ref_vel
//    map          : map_waypoints_s, map_waypoints_x, map_waypoints_y
//    car          : car_x, car_y, car_yaw, car_s, car_d
//    previous_path: previous_path_x, previous_path_y


// OUTPUTS:
//    trajectory: next_x_vals, next_y_vals
TrajectoryXY generate_trajectory(Target target, Map &map, CarData car, std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, int prev_size);

TrajectoryJMT generate_trajectory_jmt(Target target, Map &map, std::vector<double> &previous_path_x, std::vector<double> &previous_path_y, int prev_size, TrajectorySD &prev_path_sd);

#endif // TRAJECTORY_H
