#ifndef INTERFACES_H
#define INTERFACES_H

#include <vector>

#include "utility.h"
#include "map.h"
#include "trajectory.h"

struct ItfFusionPlanning { // FUSION -> PLANNING
  CarData car; // ego: x,y,yaw,norm_v(,s,d)
  PreviousPath previous_path; // SCANeR => 49 pts, Unity => 8 pts in mean
  std::vector<std::vector<double>> sensor_fusion; // other objects: car_id,x,y,vx,vy(,s,d)
};

struct ItfNavigationPlanning { // NAV -> PLANNING
  Map map; // x,y,s,dx,dy (SCANeR 1 pt every 1.11m, Unity 1 pt every 30m->1m)
};

struct ItfPlanningCtrl {  // PLANNING -> CTRL
  TrajectoryJMT trajectory;   // TrajectoryXY, TrajectorySD
  std::vector<double> poly_s; // TJP
  std::vector<double> poly_d; // TJP
  double tmax;                // TJP
};

struct ItfCtrlPlanning { // CTRL -> PLANNING
  double dt;
};


#endif // INTERFACES_H
