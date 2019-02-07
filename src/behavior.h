#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <iostream>
#include <vector>
#include <cassert>
#include <algorithm>

#include "params.h"
#include "utility.h"
#include "predictions.h"

struct Target {
  int lane;
  double velocity; // for JMT trajectories
  double time;  // for manoeuver
  double accel; // XXX for emergency trajectories
  Target(int l=0, double v=0, double t=0, double a=0) : lane(l), velocity(v), time(t), accel(a) {}
};


class Behavior {
public:
  Behavior(std::vector<std::vector<double>> const &sensor_fusion, CarData car, Predictions const &predictions);
  virtual ~Behavior();
  std::vector<Target> get_targets();

private:
  std::vector<Target> targets_;
};


#endif // BEHAVIOR_H
