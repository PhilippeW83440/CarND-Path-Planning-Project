#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include "utility.h"

const bool PARAM_MAP_BOSCH = true;

extern std::string map_file_; // cf params.cpp
extern std::string map_bosch_file_; // cf params.cpp

// The max s value before wrapping around the track back to 0
const double MAXIMUM_S = 6945.554;
extern double MAX_S;

// center point of the track
const double PARAM_CENTER_X = 1000;
const double PARAM_CENTER_Y = 2000;


const int PARAM_NB_POINTS = 50; // in the trajectory sent to simulator
const double PARAM_DT = 0.02; // 1 point every 0.02 s

const double PARAM_LANE_WIDTH = 4.0; // meters

const double PARAM_MAX_SPEED_MPH = 49;

const double PARAM_MAX_SPEED = 22; // m.s-1
const double PARAM_MAX_ACCEL = 10; // m.s-2
const double PARAM_MAX_JERK  = 10; // m.s-3 average jerk over 1 second

const double PARAM_FOV = 70.0; // Field Of View

const double PARAM_MAX_SPEED_INC = PARAM_MAX_ACCEL * PARAM_DT; // m.s-1 per 0.02 sec
const double PARAM_MAX_SPEED_INC_MPH = ms_to_mph(PARAM_MAX_SPEED_INC);

const double PARAM_DIST_SAFETY = 3.5; // meters

// reduce latency reaction, but account for simulator latency ...
// assume 100 ms max simulator latency
const int PARAM_PREV_PATH_XY_REUSED = 5;

const bool PARAM_TRAJECTORY_JMT = true;

// some extra margin for safety: safety box around the car
const double PARAM_CAR_SAFETY_W = 3; // meters
const double PARAM_CAR_SAFETY_L = 6; // meters
const int PARAM_MAX_COLLISION_STEP = 25;

// cf cost.cpp: weighted cost function from most critical to less critical one
const int PARAM_COST_FEASIBILITY = 10000; // vs collisions, vs vehicle capabilities
const int PARAM_COST_SAFETY      = 1000; // vs buffer distance, vs visibility or curvature
const int PARAM_COST_LEGALITY    = 100; // vs speed limits
const int PARAM_COST_COMFORT     = 10; // vs jerk
const int PARAM_COST_EFFICIENCY  = 1; // vs target lane, target speed and time to goal

const int PARAM_NB_LANES = 3;

// default Safety Distance for Lane Change: will be re-evaluated dynamically
const double PARAM_SD_LC = 10.0;
// default Safety Distance: will be re-evaluated dynamically
const double PARAM_SD = 30.0;

#endif // PARAMS_H
