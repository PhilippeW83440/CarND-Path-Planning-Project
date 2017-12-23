#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>

#include "utility.h"
#include "params.h"

// TODO remove hardcoded values

class Predictions {
public:
  Predictions(std::vector<std::vector<double> > &sensor_fusion, double car_s, double car_d, int horizon);
  virtual ~Predictions();

  std::map<int, std::vector<Coord> > get_predictions();
  double get_lane_speed(int lane);
  double get_lane_free_space(int lane);

private:
  std::vector<int> find_closest_objects(std::vector<std::vector<double>> &sensor_fusion, double car_s, double car_d);

  // map of at most 6 predicitons of "n_horizon" (x,y) coordinates
  std::map< int, std::vector<Coord> > predictions_;
  double lane_speed_[3];
  double lane_free_space_[3];
};

#endif // PREDICTIONS_H
