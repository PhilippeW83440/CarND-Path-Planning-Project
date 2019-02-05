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
  Predictions(std::vector<std::vector<double> > const &sensor_fusion, CarData const &car, int horizon);
  virtual ~Predictions();

  std::map<int, std::vector<Coord> > get_predictions() const { return predictions_; };
  double get_safety_distance() const { return safety_distance_; };
  double get_paranoid_safety_distance() const { return paranoid_safety_distance_; };
  double get_lane_speed(int lane) const;
  double get_lane_free_space(int lane) const;


private:
  void set_safety_distances(std::vector<std::vector<double>> const &sensor_fusion, CarData const &car);
  void set_lane_info(std::vector<std::vector<double>> const &sensor_fusion, CarData const &car);
  std::vector<int> find_closest_objects(std::vector<std::vector<double>> const &sensor_fusion, CarData const &car);
  double get_safety_distance(double vel_back, double vel_front, double time_latency);

  // TODO use vector init depending on PARAM_NB_LANES
  std::vector<int> front_; // idx of closest object per lane
  std::vector<int> back_; // idx of closest object per lane

  // TODO use FOV instead of INF
  std::vector<double> front_dmin_; // dist min per lane
  std::vector<double> back_dmin_;  // dist min per lane

  std::vector<double> front_velocity_;
  std::vector<double> front_safety_distance_;

  std::vector<double> back_velocity_;
  std::vector<double> back_safety_distance_;

  // map of at most 6 predicitons of "n_horizon" (x,y) coordinates
  std::map< int, std::vector<Coord> > predictions_;
  double lane_speed_[PARAM_NB_LANES];
  double lane_free_space_[PARAM_NB_LANES];

  // safety distance computation related
  double vel_ego_;
  double decel_;

  double dist_front_;
  double vel_front_;
  double time_to_collision_;  // vs front vehicle
  double time_to_stop_;  // time from vel_ego_ to 0
  double time_to_decelerate_;  // time from vel_ego_ to vel_front_

  // will be used by behavior planner
  double safety_distance_ = PARAM_SD;
  double paranoid_safety_distance_ = PARAM_SD;

};

#endif // PREDICTIONS_H
