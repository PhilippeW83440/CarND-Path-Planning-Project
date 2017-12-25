#include "cost.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector2d;


// ------------------------------------------------------------
// checkCollision as per SAT (Separating Axis Theorem)
// to identify collision between 2 convex rectangular objects
// cf http://www.dyn4j.org/2010/01/sat/
// ------------------------------------------------------------
bool Cost::check_collision(double x0, double y0, double theta0, double x1, double y1, double theta1) 
{
  Vector2d trans0; 
  trans0 << x0, y0;
  Vector2d trans1; 
  trans1 << x1, y1;
  Matrix2d rot0, rot1;
  rot0 << cos(theta0), -sin(theta0),
          sin(theta0),  cos(theta0);
  rot1 << cos(theta1), -sin(theta1),
          sin(theta1),  cos(theta1);
  
  double W = PARAM_CAR_SAFETY_W; //2;
  double L = PARAM_CAR_SAFETY_L; //5;

  MatrixXd car(2,4);
  car << -L/2, L/2,  L/2, -L/2,
          W/2, W/2, -W/2, -W/2; 

  MatrixXd car0(2,4);
  MatrixXd car1(2,4);

  for (int i = 0; i < car.cols(); i++) {
    car0.col(i) = rot0 * car.col(i) + trans0;
    car1.col(i) = rot1 * car.col(i) + trans1;
  }

  // principal axis list
  MatrixXd axis(2,4);
  axis << cos(theta0), -sin(theta0), cos(theta1), -sin(theta1), 
          sin(theta0),  cos(theta0), sin(theta1),  cos(theta1);

  for (int i = 0; i < axis.cols(); i++) {
    Vector2d principal_axis = axis.col(i);

    // projection of car0
    double min0 = principal_axis.dot(car0.col(0));
    double max0 = min0;
    for (int j = 1; j < car0.cols(); j++) {
      double proj0 = principal_axis.dot(car0.col(j));
      if (proj0 > max0) max0 = proj0;
      if (proj0 < min0) min0 = proj0;
    }

    // projection of car1
    double min1 = principal_axis.dot(car1.col(0));
    double max1 = min1;
    for (int j = 1; j < car1.cols(); j++) {
      double proj1 = principal_axis.dot(car1.col(j));
      if (proj1 > max1) max1 = proj1;
      if (proj1 < min1) min1 = proj1;
    }

    //cout << "min0=" << min0 << " max0=" << max0 << endl;
    //cout << "min1=" << min1 << " max1=" << max1 << endl;

    bool overlap = false;
    if (min1 >= min0 && min1 < max0) overlap = true;
    if (max1 >= min0 && max1 < max0) overlap = true;
    if (min0 >= min1 && min0 < max1) overlap = true;
    if (max0 >= min1 && max0 < max1) overlap = true;
    if (!overlap) return false;
  }

  return true;
}

int Cost::check_collision_on_trajectory(vector<vector<double>> &trajectory, std::map<int, vector<Coord> > &predictions)
{
  std::map<int, vector<Coord> >::iterator it = predictions.begin();
  while(it != predictions.end()) {
    int fusion_index = it->first;
    vector<Coord> prediction = it->second;

    assert(prediction.size() == trajectory[0].size());
    assert(prediction.size() == trajectory[1].size());

    for (int i = 0; i < PARAM_MAX_COLLISION_STEP; i++) { // up to 50 (x,y) coordinates
      double obj_x = prediction[i].x;
      double obj_y = prediction[i].y;
      double obj_x_next = prediction[i+1].x;
      double obj_y_next = prediction[i+1].y;
      double obj_heading = atan2(obj_y_next - obj_y, obj_x_next - obj_x);

      double ego_x = trajectory[0][i];
      double ego_y = trajectory[1][i];
      double ego_x_next = trajectory[0][i+1];
      double ego_y_next = trajectory[1][i+1];
      double ego_heading = atan2(ego_y_next - ego_y, ego_x_next - ego_x);

      if (check_collision(obj_x, obj_y, obj_heading, ego_x, ego_y, ego_heading)) {
        cout << "!!! ... COLLISION predicted on candidate trajectory at step " << i << "  ... !!!" << endl;
        return (i+1);
      }
    }
    it++;
  }

  return 0;
}


// check max speed, acceleration, jerk
bool Cost::check_max_capabilities(vector<vector<double>> &traj)
{
  double vx, ax, jx;
  double vy, ay, jy;
  double vel, acc, jerk;
  double max_vel = 0;
  double max_acc = 0;
  double total_jerk = 0;
  double x, x_1, x_2, x_3;
  double y, y_1, y_2, y_3;
  double jerk_per_second;

  assert(traj[0].size() == traj[1].size()); // as much x than y ...

  for (size_t t = 3; t < traj[0].size(); t++) {
    x   = traj[0][t];
    x_1 = traj[0][t-1];
    x_2 = traj[0][t-2];
    x_3 = traj[0][t-3];

    y   = traj[1][t];
    y_1 = traj[1][t-1];
    y_2 = traj[1][t-2];
    y_3 = traj[1][t-3];

    vx = (x - x_1) / PARAM_DT;
    vy = (y - y_1) / PARAM_DT;

    ax = (x - 2*x_1 + x_2) / (PARAM_DT * PARAM_DT);
    ay = (y - 2*y_1 + y_2) / (PARAM_DT * PARAM_DT);

    // rounding to 2 decimals (cm precision) to ensure numerical stability
    jx = (x - 3*x_1 + 3*x_2 - x_3);
    jx = roundf(jx * 100) / 100;
    jx = jx / (PARAM_DT * PARAM_DT * PARAM_DT);
    jy = (y - 3*y_1 + 3*y_2 - y_3);
    jy = roundf(jy * 100) / 100;
    jy = jy / (PARAM_DT * PARAM_DT * PARAM_DT);

    vel = sqrt(vx*vx + vy*vy);
    acc = sqrt(ax*ax + ay*ay);
    jerk = sqrt(jx*jx + jy*jy);

    total_jerk += jerk * PARAM_DT;

    //cout << "jx=" << jx << " jy=" << jy << endl;
    //cout << "vel=" << vel << " acc=" << acc << " jerk=" << jerk << endl;

    if (vel > max_vel)
      max_vel = vel;
    if (acc > max_acc)
      max_acc = acc;
  }

  jerk_per_second = total_jerk / (PARAM_NB_POINTS * PARAM_DT);

  if (roundf(max_vel) > PARAM_MAX_SPEED || roundf(max_acc) > PARAM_MAX_ACCEL || jerk_per_second > PARAM_MAX_JERK) {
    cout << "max_vel=" << max_vel << " max_acc=" << max_acc << " jerk_per_second=" << jerk_per_second  << endl;
    //assert(1 == 0);
    return true;
  } else {
    return false;
  }
}


double Cost::get_predicted_dmin(vector<vector<double>> &trajectory, std::map<int, vector<Coord> > &predictions)
{
  double dmin = INF;

  std::map<int, vector<Coord> >::iterator it = predictions.begin();
  while(it != predictions.end())
  {
    int fusion_index = it->first;
    //cout << "fusion_index=" << fusion_index << endl;
    vector<Coord> prediction = it->second;

    assert(prediction.size() == trajectory[0].size());
    assert(prediction.size() == trajectory[1].size());

    for (size_t i = 0; i < prediction.size(); i++) { // up to 50 (x,y) coordinates
      double obj_x = prediction[i].x;
      double obj_y = prediction[i].y;
      double ego_x = trajectory[0][i];
      double ego_y = trajectory[1][i];

      double dist = distance(ego_x, ego_y, obj_x, obj_y);
      if (dist < dmin) {
        dmin = dist;
      }
    }
    it++;
  }

  cout << "=====> dmin = " << dmin << endl;
  return dmin;
}


Cost::Cost(vector<vector<double>> &trajectory, Target target, Predictions &predict, int car_lane)
{
  cost_ = 0; // lower cost preferred

  double cost_feasibility = 0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal

  double weight_feasibility = 10000; // vs collisions, vs vehicle capabilities
  double weight_safety      = 1000; // vs buffer distance, vs visibility or curvature
  double weight_legality    = 100; // vs speed limits
  double weight_comfort     = 10; // vs jerk
  double weight_efficiency  = 1; // vs target lane, target speed and time to goal

  std::map<int, vector<Coord> > predictions = predict.get_predictions();

  // 1) FEASIBILITY cost
  cost_feasibility += check_collision_on_trajectory(trajectory, predictions);
  //if (check_max_capabilities(trajectory))
  //{
  //  cost_feasibility += 1;
  //}
  cost_ = cost_ + weight_feasibility * cost_feasibility;

  // 2) SAFETY cost
  // double dmin = get_predicted_dmin(trajectory, predictions);
  // assert(dmin >= 0);
  // if (dmin < PARAM_DIST_SAFETY) {
  //   cost_safety = PARAM_DIST_SAFETY - dmin;
  // } else {
  //   cost_safety = 0;
  // }
  cost_ = cost_ + weight_safety * cost_safety;

  // 3) LEGALITY cost
  cost_ = cost_ + weight_legality * cost_legality;

  // 4) COMFORT cost
  cost_ = cost_ + weight_comfort * cost_comfort;

  // 5) EFFICIENCY cost
  //cost_efficiency = PARAM_MAX_SPEED_MPH - target.velocity;
  //cost_efficiency = PARAM_MAX_SPEED_MPH - predictions_lane_speed[target.lane];

  // sensor_fusion speed in m/s !!!
  //cost_efficiency = PARAM_MAX_SPEED - predictions_lane_speed[target.lane];
  cost_efficiency = PARAM_FOV - predict.get_lane_free_space(target.lane);
  cost_ = cost_ + weight_efficiency * cost_efficiency;

  cout << "car_lane=" << car_lane << " target.lane=" << target.lane << " target_lvel=" << predict.get_lane_speed(target.lane) << " cost=" << cost_ << endl;
}

Cost::~Cost() {}

double Cost::get_cost() 
{
  return cost_;
}
