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
#include "trajectory.h"



class Cost {
public:
  Cost(struct TrajectoryXY const &trajectory, Target target, Predictions &predictions, int car_lane);
  virtual ~Cost();

  double get_cost();

private:
  bool check_collision(double x0, double y0, double theta0, double x1, double y1, double theta1);
  int  check_collision_on_trajectory(struct TrajectoryXY const &trajectory, std::map<int, std::vector<Coord> > &predictions);
  bool check_max_capabilities(struct TrajectoryXY const &traj);
  double get_predicted_dmin(struct TrajectoryXY const &trajectory, std::map<int, std::vector<Coord> > &predictions);

  double cost_;
};

#endif // COST_H
