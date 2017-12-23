#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>

#include "utility.h"
#include "params.h"
#include "predictions.h"
#include "Eigen-3.3/Eigen/Dense"

#include "behavior.h"


// TARGET: lane, velocity, time for maneuver
double cost_function(std::vector<std::vector<double>> &trajectory, Target target, Predictions &predictions, std::vector<std::vector<double>> &sensor_fusion, int car_lane);

#endif // COST_H
