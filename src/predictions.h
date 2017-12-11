#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>

extern double predictions_lane_speed[3];

std::map<int, std::vector<std::vector<double> > > generate_predictions(std::vector<std::vector<double>> &sensor_fusion, double car_s, double car_d, int horizon);

#endif // PREDICTIONS_H
