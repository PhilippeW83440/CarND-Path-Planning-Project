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



class Cost {
public:
  Cost(std::vector<std::vector<double>> &trajectory, Target target, Predictions &predictions, int car_lane);
  virtual ~Cost();

  double get_cost();

private:
  bool check_collision(double x0, double y0, double theta0, double x1, double y1, double theta1);
  int  check_collision_on_trajectory(std::vector<std::vector<double>> &trajectory, std::map<int, std::vector<Coord> > &predictions);
  bool check_max_capabilities(std::vector<std::vector<double>> &traj);
  double get_predicted_dmin(std::vector<std::vector<double>> &trajectory, std::map<int, std::vector<Coord> > &predictions);

  double cost_;
};

#endif // COST_H
