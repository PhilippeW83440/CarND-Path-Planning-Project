#include "predictions.h"


using namespace std;


// map of at most 6 predictions: with 50 points x 2 coord (x,y): 6 objects predicted over 1 second horizon
// predictions map: a dictionnary { fusion_index : horizon * (x,y) }
Predictions::Predictions(vector<vector<double>> const &sensor_fusion, CarData const &car, int horizon)
{
  std::map<int, vector<Coord> > predictions; // map of at most 6 predicitons of "n_horizon" (x,y) coordinates

  front_.push_back(-1); front_.push_back(-1); front_.push_back(-1);
  back_.push_back(-1);  back_.push_back(-1);  back_.push_back(-1);

  front_dmin_.push_back(INF); front_dmin_.push_back(INF); front_dmin_.push_back(INF);
  back_dmin_.push_back(INF);  back_dmin_.push_back(INF);  back_dmin_.push_back(INF);

  front_velocity_.push_back(PARAM_MAX_SPEED); front_velocity_.push_back(PARAM_MAX_SPEED); front_velocity_.push_back(PARAM_MAX_SPEED);
  front_safety_distance_.push_back(PARAM_SD_LC); front_safety_distance_.push_back(PARAM_SD_LC); front_safety_distance_.push_back(PARAM_SD_LC);

  back_velocity_.push_back(PARAM_MAX_SPEED); back_velocity_.push_back(PARAM_MAX_SPEED); back_velocity_.push_back(PARAM_MAX_SPEED);
  back_safety_distance_.push_back(PARAM_SD_LC); back_safety_distance_.push_back(PARAM_SD_LC); back_safety_distance_.push_back(PARAM_SD_LC);

  // vector of indexes in sensor_fusion
  vector<int> closest_objects = find_closest_objects(sensor_fusion, car); 

  for (size_t i = 0; i < closest_objects.size(); i++) {
    int fusion_index = closest_objects[i];
    if (fusion_index >= 0) {
      //double fusion_id = sensor_fusion[fusion_index][0];
      double x = sensor_fusion[fusion_index][1];
      double y = sensor_fusion[fusion_index][2];
      double vx = sensor_fusion[fusion_index][3];
      double vy = sensor_fusion[fusion_index][4];
      vector<Coord> prediction; // vector of at most 6 predicitons of "n_horizon" (x,y) coordinates
      for (int j = 0; j < horizon; j++) {
        Coord coord;
        coord.x = x + vx * j*PARAM_DT;
        coord.y = y + vy * j*PARAM_DT;
        prediction.push_back(coord);
      }
      predictions_[fusion_index] = prediction;
    }
  }

  set_safety_distances(sensor_fusion, car);
  set_lane_info(sensor_fusion, car);
}

Predictions::~Predictions() {}


//double get_sdistance(double s1, double s2)
//{
//  // account for s wraparound at MAX_S
//  double sdistance = min( fabs(s1 - s2), min(fabs((s1+MAX_S) - s2), fabs(s1 - (s2+MAX_S))) );
//  return sdistance;
//}


double get_sensor_fusion_vel(vector<vector<double>> const &sensor_fusion, int idx, double default_vel)
{
  double vx, vy, vel;
  if (idx >= 0 && idx < (int)sensor_fusion.size()) {
    vx = sensor_fusion[idx][3];
    vy = sensor_fusion[idx][4];
    vel = sqrt(vx*vx+vy*vy);
  } else {
    vel = default_vel;
  }
  return vel;
}

double Predictions::get_safety_distance(double vel_back, double vel_front, double time_latency)
{
  //cout << "get_safety_distance: " << vel_back << ", " << vel_front << ", " << time_latency << '\n';

  double safety_distance = PARAM_SD_LC;
  if (vel_back > vel_front) {
      double time_to_decelerate = (vel_back - vel_front) / decel_ + time_latency;
      safety_distance = vel_back * time_to_decelerate + 1.5 * PARAM_CAR_SAFETY_L;
  }
  safety_distance = fmax(safety_distance, PARAM_SD_LC);  // conservative
  return safety_distance;
}

void Predictions::set_safety_distances(vector<vector<double>> const &sensor_fusion, CarData const &car)
{
  vel_ego_ = mph_to_ms(car.speed);  // velocity of ego vehicle
  // slightly conservative as it will relate to safety distance
  decel_ = 0.8 * PARAM_MAX_ACCEL;
  time_to_stop_ = vel_ego_ / decel_;

  vel_front_ = get_sensor_fusion_vel(sensor_fusion, front_[car.lane], PARAM_MAX_SPEED);
  dist_front_ = front_dmin_[car.lane];

  if (vel_ego_ > vel_front_) {
    time_to_collision_ = dist_front_ / (vel_ego_ - vel_front_);
    time_to_decelerate_ = (vel_ego_ - vel_front_) / decel_;
    safety_distance_ = vel_ego_ * time_to_decelerate_ + 1.75 * PARAM_CAR_SAFETY_L;
  } else {
    time_to_collision_ = INF;
    time_to_decelerate_ = 0;
    safety_distance_ = 1.75 * PARAM_CAR_SAFETY_L; 
  }

  paranoid_safety_distance_ = vel_ego_ * time_to_stop_ + 2 * PARAM_CAR_SAFETY_L;

  cout << "SAFETY: D=" << dist_front_ << " dV=" << vel_ego_ - vel_front_ << " TTC=" << time_to_collision_ 
       << " TTD=" << time_to_decelerate_ << " SD=" << safety_distance_ << " PSD=" << paranoid_safety_distance_ << '\n';

  for (int i = 0; i < PARAM_NB_LANES; i++) {
    front_velocity_[i] = get_sensor_fusion_vel(sensor_fusion, front_[i], PARAM_MAX_SPEED);
    front_safety_distance_[i] = get_safety_distance(vel_ego_, front_velocity_[i], 0.0);

    back_velocity_[i] = get_sensor_fusion_vel(sensor_fusion, back_[i], 0);
    back_safety_distance_[i] = get_safety_distance(back_velocity_[i], vel_ego_, 2.0);

    cout << "SAFETY_DISTANCE for LC[" << i << "]: front_sd=" << front_safety_distance_[i] << " back_sd=" << back_safety_distance_[i] << '\n';
  }

}

void Predictions::set_lane_info(vector<vector<double>> const &sensor_fusion, CarData const &car)
{
  int car_lane = get_lane(car.d);
  for (size_t i = 0; i < front_.size(); i++) {
    cout << "lane " << i << ": ";
    cout << "front " << front_[i] << " at " << front_dmin_[i] << " s_meters ; ";
    cout << "back " << back_[i] << " at " << back_dmin_[i] << " s_meters" << endl;

    int lane = i;
    // !!! This should be part of the behavior planner behavior.cpp
    if (front_[i] >= 0) { // a car in front of us
      //if (lane != car_lane && (back_dmin_[i] <= 10 || front_dmin_[i] <= 10)) {
      if (lane != car_lane && (back_dmin_[i] <= back_safety_distance_[i] || front_dmin_[i] <= front_safety_distance_[i])) {
        lane_speed_[i] = 0;
        lane_free_space_[i] = 0; // too dangerous
      } else {
        double vx = sensor_fusion[front_[i]][3];
        double vy = sensor_fusion[front_[i]][4];
        lane_speed_[i] = sqrt(vx*vx+vy*vy);
        lane_free_space_[i] = front_dmin_[i];
      }
    } else {  // if nobody in front of us
      //if (lane != car_lane && back_dmin_[i] <= 10) {
      if (lane != car_lane && back_dmin_[i] <= back_safety_distance_[i]) {
        lane_speed_[i] = 0;
        lane_free_space_[i] = 0; // too dangerous
      } else {
        lane_speed_[i] = PARAM_MAX_SPEED_MPH;
        lane_free_space_[i] = PARAM_FOV;
      }
    }
    cout << "Predictions::lane_speed_[" << i << "]=" << lane_speed_[i] << endl;
  }
}

// we generate predictions for closet car per lane in front of us
// we generate predictions for closet car per lane behind us
// => at most 6 predictions (for now on) as we have 3 lanes

// sort of simple scene detection
vector<int> Predictions::find_closest_objects(vector<vector<double>> const &sensor_fusion, CarData const &car) {
  // Handle FOV and s wraparound
  double sfov_min = car.s - PARAM_FOV;
  double sfov_max = car.s + PARAM_FOV;
  double sfov_shit = 0;
  if (sfov_min < 0) { // Handle s wrapping
    sfov_shit = -sfov_min;
  } else if (sfov_max > MAX_S) {
    sfov_shit = MAX_S - sfov_max;
  }
  sfov_min += sfov_shit;
  sfov_max += sfov_shit;
  assert(sfov_min >= 0 && sfov_min <= MAX_S);
  assert(sfov_max >= 0 && sfov_max <= MAX_S);

  double car_s = car.s;

  car_s += sfov_shit;

  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    double s = sensor_fusion[i][5] + sfov_shit;
    if (s >= sfov_min && s <= sfov_max) { // object in FOV
      double d = sensor_fusion[i][6];
      int lane = get_lane(d);
      if (lane < 0 || lane > 2)
          continue; // some garbage values in sensor_fusion from time to time

      // s wraparound already handled via FOV shift
      //double dist = get_sdistance(s, car_s);
      double dist = fabs(s - car_s);

      if (s >= car_s) {  // front
        if (dist < front_dmin_[lane]) {
          front_[lane] = i;
          front_dmin_[lane] = dist;
        }
      } else {  // back
        if (dist < back_dmin_[lane]) {
          back_[lane] = i;
          back_dmin_[lane] = dist;
        }
      }
    }
  }


  return { front_[0], back_[0], front_[1], back_[1], front_[2], back_[2] };
}

double Predictions::get_lane_speed(int lane) const {
  if (lane >= 0 && lane <= 3) {
    return lane_speed_[lane];
  } else {
    return 0;
  }
}

double Predictions::get_lane_free_space(int lane) const {
  if (lane >= 0 && lane <= 3) {
    return lane_free_space_[lane];
  } else {
    return 0;
  }
}
