#ifndef PARAMS_H
#define PARAMS_H

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

const int param_nb_points_simu = 50; // in the trajectory sent to simulator
const double param_dt = 0.02; // 1 point every 0.02 s

const double param_lane_width = 4.0; // meters

const double param_max_speed_mph = 49.5;

const double param_max_speed = 22; // m.s-1
const double param_max_accel = 10; // m.s-2
const double param_max_jerk  = 10; // m.s-3

const double param_fov = 100.0; // Field Of View 100 meters


#endif // PARAMS_H
