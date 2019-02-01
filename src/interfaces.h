#ifndef INTERFACES_H
#define INTERFACES_H

#include <vector>

#include "utility.h"
#include "map.h"
#include "trajectory.h"

struct ItfFusionPlanning {
  CarData car; // ego 
  PreviousPath previous_path;
  std::vector<std::vector<double>> sensor_fusion; // other objects

  Map map;
};

struct ItfPlanningCtrl {
  TrajectoryJMT trajectory;
  double dt;

  std::vector<double> poly_s;
  std::vector<double> poly_d;
  double tmax;
};


#endif // INTERFACES_H
