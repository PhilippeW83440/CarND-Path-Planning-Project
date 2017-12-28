#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <iostream>
#include <vector>
#include <cassert>

#include "params.h"
#include "utility.h"

struct Target {
  double lane;
  double velocity; // for JMT trajectories
  double time;  // for manoeuver
  double accel; // XXX for emergency trajectories
};


class Behavior {
public:
  Behavior(std::vector<std::vector<double>> const &sensor_fusion, CarData car);
  virtual ~Behavior();
  std::vector<Target> get_targets();

private:
  std::vector<Target> targets_;
};


#endif // BEHAVIOR_H
