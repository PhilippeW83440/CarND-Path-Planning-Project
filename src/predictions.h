#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>

#include "utility.h"
#include "params.h"

extern double predictions_lane_speed[3];
extern double predictions_free_space[3];

std::map<int, std::vector<Coord> > generate_predictions(std::vector<std::vector<double>> &sensor_fusion, double car_s, double car_d, int horizon);


#endif // PREDICTIONS_H
