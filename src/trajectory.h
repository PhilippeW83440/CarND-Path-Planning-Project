#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

#include "map.h"
#include "behavior.h"
#include "spline.h"
#include "utility.h"
#include "map.h"
#include "cost.h"
#include "params.h"
#include "predictions.h"

#include "Eigen-3.3/Eigen/Dense"


// Point of a C2 class function
struct PointC2 {
  double f;
  double f_dot;
  double f_ddot;
  PointC2 (double y=0, double y_dot=0, double y_ddot=0) : f(y), f_dot(y_dot), f_ddot(y_ddot) {}
};

struct TrajectorySD {
  std::vector<PointC2> path_s;
  std::vector<PointC2> path_d;
  TrajectorySD (std::vector<PointC2> S={}, std::vector<PointC2> D={}) : path_s(S), path_d(D) {}
};

struct TrajectoryXY {
  std::vector<double> x_vals;
  std::vector<double> y_vals;
  TrajectoryXY (std::vector<double> X={}, std::vector<double> Y={}) : x_vals(X), y_vals(Y) {}
};

struct TrajectoryJMT {
  TrajectoryXY trajectory;
  TrajectorySD path_sd;
};


struct PreviousPath {
  TrajectoryXY xy;   // < PARAM_NB_POINTS (some already used by simulator)
  TrajectorySD sd;   // exactly PARAM_NB_POINTS (not sent to simulator)
  int num_xy_reused;  // reused from xy
  PreviousPath (TrajectoryXY XY={}, TrajectorySD SD={}, int N=0) : xy(XY), sd(SD), num_xy_reused(N) {}
};

TrajectoryJMT JMT_init(double car_s, double car_d);


class Trajectory {
public:
  Trajectory(std::vector<Target> targets, Map &map, CarData &car, PreviousPath &previous_path, Predictions &predictions);
  ~Trajectory() {};

  double getMinCost() { return min_cost_; };
  double getMinCostIndex() { return min_cost_index_; } ;
  TrajectoryXY getMinCostTrajectoryXY() { return trajectories_[min_cost_index_]; };
  TrajectorySD getMinCostTrajectorySD() { return trajectories_sd_[min_cost_index_]; };

private:
  std::vector<class Cost> costs_;
  std::vector<TrajectoryXY> trajectories_;
  std::vector<TrajectorySD> trajectories_sd_;
  double min_cost_;
  int min_cost_index_;

  std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);
  double polyeval(std::vector<double> c, double t);
  double polyeval_dot(std::vector<double> c, double t);
  double polyeval_ddot(std::vector<double> c, double t);

  TrajectoryXY generate_trajectory     (Target target, Map &map, CarData const &car, PreviousPath const &previous_path);
  TrajectoryJMT generate_trajectory_jmt(Target target, Map &map, PreviousPath const &previous_path);
};

#endif // TRAJECTORY_H
