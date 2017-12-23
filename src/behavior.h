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
  double velocity;
  double time;  // for manoeuver
};


class Behavior {
public:
  Behavior(std::vector<std::vector<double>> &sensor_fusion, int lane, double car_s, double car_d, double ref_vel);
  virtual ~Behavior();
  std::vector<Target> get_targets();

private:
  std::vector<Target> targets_;
};


#endif // BEHAVIOR_H
