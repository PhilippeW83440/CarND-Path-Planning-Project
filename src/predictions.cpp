#include "predictions.h"


using namespace std;

double predictions_lane_speed[3];
double predictions_free_space[3];

//double get_sdistance(double s1, double s2)
//{
//  // account for s wraparound at MAX_S
//  double sdistance = min( fabs(s1 - s2), min(fabs((s1+MAX_S) - s2), fabs(s1 - (s2+MAX_S))) );
//  return sdistance;
//}

// we generate predictions for closet car per lane in front of us
// we generate predictions for closet car per lane behind us
// => at most 6 predictions (for now on) as we have 3 lanes

// sort of simple scene detection
vector<int> find_closest_objects(vector<vector<double>> &sensor_fusion, double car_s, double car_d) {
  vector<int> front = {-1, -1, -1}; // idx of closest object per lane
  vector<int> back = {-1, -1, -1}; // idx of closest object per lane

  vector<double> front_dmin = {INF, INF, INF}; // per lane
  vector<double> back_dmin = {INF, INF, INF}; // per lane

  // Handle FOV and s wraparound
  double sfov_min = car_s - PARAM_FOV;
  double sfov_max = car_s + PARAM_FOV;
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

      if (s >= car_s) { /* front */
        if (dist < front_dmin[lane]) {
          front[lane] = i;
          front_dmin[lane] = dist;
        }
      } else /* back */ {
        if (dist < back_dmin[lane]) {
          back[lane] = i;
          back_dmin[lane] = dist;
        }
      }
    }
  }

  int car_lane = get_lane(car_d);
  for (size_t i = 0; i < front.size(); i++) {
    cout << "lane " << i << ": ";
    cout << "front " << front[i] << " at " << front_dmin[i] << " s_meters ; ";
    cout << "back " << back[i] << " at " << back_dmin[i] << " s_meters" << endl;

    int lane = i;
    // !!! This should be part of the behavior planner behavior.cpp
    if (front[i] >= 0) { // a car in front of us
      if (lane != car_lane && (back_dmin[i] <= 10 || front_dmin[i] <= 10)) {
        predictions_lane_speed[i] = 0;
        predictions_free_space[i] = 0; // too dangerous
      } else {
        double vx = sensor_fusion[front[i]][3];
        double vy = sensor_fusion[front[i]][4];
        predictions_lane_speed[i] = sqrt(vx*vx+vy*vy);
        predictions_free_space[i] = front_dmin[i];
      }
    } else { // if nobody in front of us
      if (lane != car_lane && back_dmin[i] <= 10) {
        predictions_lane_speed[i] = 0;
        predictions_free_space[i] = 0; // too dangerous
      } else {
        predictions_lane_speed[i] = PARAM_MAX_SPEED_MPH;
        predictions_free_space[i] = PARAM_FOV;
      }
    }

    cout << "predictions_lane_speed[" << i << "]=" << predictions_lane_speed[i] << endl;
  }

  return { front[0], back[0], front[1], back[1], front[2], back[2] };
}


// map of at most 6 predictions: with 50 points x 2 coord (x,y): 6 objects predicted over 1 second horizon
// predictions map: a dictionnary { fusion_index : horizon * (x,y) }
std::map< int, vector<Coord>> generate_predictions(vector<vector<double>> &sensor_fusion, double car_s, double car_d, int horizon)
{
  std::map<int, vector<Coord> > predictions; // map of at most 6 predicitons of "n_horizon" (x,y) coordinates

  // vector of indexes in sensor_fusion
  vector<int> closest_objects = find_closest_objects(sensor_fusion, car_s, car_d); 

  for (int i = 0; i < closest_objects.size(); i++) {
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
      predictions[fusion_index] = prediction;
    }
  }

  return predictions;
}
