#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>


// TARGET: lane, velocity, time for maneuver
double cost_function(std::vector<std::vector<double>> &trajectory, int target_lane, double target_vel, std::map<int, std::vector<std::vector<double>>> &predictions, std::vector<std::vector<double>> &sensor_fusion, int car_lane);

#endif // COST_H
