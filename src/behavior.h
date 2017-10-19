#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

vector<vector<double>> behavior_planner_find_targets(vector<vector<double>> sensor_fusion, int prev_size, int lane, double car_s, double car_d, double ref_vel);

#endif // BEHAVIOR_H
