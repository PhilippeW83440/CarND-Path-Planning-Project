#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>


// TARGET: lane, velocity, time for maneuver
double cost_function(std::vector<std::vector<double>> trajectory, int target_lane, double target_vel, double target_time);

#endif // COST_H
