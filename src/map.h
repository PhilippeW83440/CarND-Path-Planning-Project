#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "spline.h"

class Map {

public:
  /**
  * Constructor
  */
  Map() {};
  
  /**
  * Destructor
  */
  virtual ~Map();

  std::vector<double> getFrenet(double x, double y, double theta);
  std::vector<double> getXY(double s, double d);
  std::vector<double> getXYspline(double s, double d); // with splines
  double getSpeedToFrenet(double Vxy, double s);

  void plot(void);
  double testError(double x, double y, double yaw); // (x,y) -> (s,d) -> (x,y) conversions and dump accuracy
  void read(std::string map_file);


private:
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
  std::vector<double> map_waypoints_normx;
  std::vector<double> map_waypoints_normy;

  std::vector<double> map_s; // pre-computed for faster access

  // better granularity: 1 point per meter
  std::vector<double> new_map_waypoints_x;
  std::vector<double> new_map_waypoints_y;
  std::vector<double> new_map_waypoints_dx;
  std::vector<double> new_map_waypoints_dy;

  std::vector<double> new_map_s; // pre-computed for faster access

  double max_error = 0.0;
  double sum_error = 0.0;
  double avg_error = 0.0;
  unsigned int num_error = 0;

  int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
};

#endif // MAP_H
