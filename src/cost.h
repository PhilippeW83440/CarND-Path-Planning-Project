#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>

#include "utility.h"
#include "params.h"
#include "predictions.h"
#include "Eigen-3.3/Eigen/Dense"

#include <cassert>
#include <cmath>


// TARGET: lane, velocity, time for maneuver
double cost_function(std::vector<std::vector<double>> &trajectory, int target_lane, double target_vel, std::map<int, std::vector<Coord> > &predictions, std::vector<std::vector<double>> &sensor_fusion, int car_lane);

#endif // COST_H
