#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

vector<vector<vector<double>>> generate_predictions(vector<vector<double>> sensor_fusion, double car_s, double car_d, int horizon);

#endif // PREDICTIONS_H
