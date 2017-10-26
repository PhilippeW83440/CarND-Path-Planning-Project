#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include "utility.h"

extern std::string map_file_; // cf params.cpp
// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;
// center point of the track
const double param_center_x = 1000;
const double param_center_y = 2000;

const int param_nb_points = 50; // in the trajectory sent to simulator
const double param_dt = 0.02; // 1 point every 0.02 s

const double param_lane_width = 4.0; // meters

const double param_max_speed_mph = 49.5;

const double param_max_speed = 22; // m.s-1
const double param_max_accel = 10; // m.s-2
const double param_max_jerk  = 10; // m.s-3 average jerk over 1 second

const double param_fov = 100.0; // Field Of View 100 meters

const double param_max_speed_inc = param_max_accel * param_dt; // m.s-1 per 0.02 sec
const double param_max_speed_inc_mph = ms_to_mph(param_max_speed_inc);

const double param_dist_slow_down = 30; // when a car is 30 m ahead of us => adjust speed if needed

const double param_dist_collision = 3; // meters


#endif // PARAMS_H
