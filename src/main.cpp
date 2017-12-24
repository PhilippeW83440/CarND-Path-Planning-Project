#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

//#include "utility.h"
#include "map.h"
#include "behavior.h"
#include "trajectory.h"
#include "cost.h"
#include "predictions.h"

#include "params.h"

#include <map>

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  //////////////////////////////////////////////////////////////////////
  Map map;

  if (PARAM_MAP_BOSCH == true) {
    map.read(map_bosch_file_);
  } else {
    map.read(map_file_);
  }

  map.plot();

  bool start = true;
  double ref_vel = 0.0; // mph

  // keep track of previous s and d paths: to initialize for continuity the new trajectory
  vector<PointC2> prev_path_s;
  vector<PointC2> prev_path_d;
  //////////////////////////////////////////////////////////////////////


  h.onMessage([&map, &ref_vel, &start, &prev_path_s, &prev_path_d](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;


    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];

          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            // --- just for debugging purposes
            double dist_min = INF;
            for (size_t i = 0; i < sensor_fusion.size(); i++) {
              // sensor_fusion: pre object [ID, x, y, vx, vy, s, d]
              double obj_x = sensor_fusion[i][1];
              double obj_y = sensor_fusion[i][2];
              double dist = distance(car_x, car_y, obj_x, obj_y);
              if (dist < dist_min)
                dist_min = dist;
            }
            cout << "************** closest object at " << dist_min << " meters *************" << endl;
            assert(dist_min >= 1);

            //////////////////////////////////////////////////////////////////////


            map.testError(car_x, car_y, car_yaw);

            int prev_size = previous_path_x.size();
            cout << "prev_size=" << prev_size << " car_x=" << car_x << " car_y=" << car_y << " car_s=" << 
                    car_s << " car_d=" << car_d << " car_speed=" << car_speed << " ref_vel=" << ref_vel << endl;

            vector<double> frenet_car = map.getFrenet(car_x, car_y, deg2rad(car_yaw));
            car_s = frenet_car[0];
            car_d = frenet_car[1];
            cout << "car_frenet_s=" << car_s << " car_frenet_d=" << car_d << endl;

            if (start) {
              struct trajectory_jmt traj_jmt = JMT_init(car_s, car_d);
              prev_path_s = traj_jmt.path_s;
              prev_path_d = traj_jmt.path_d;
              start = false;
            }

            // --- 6 car predictions x 50 points x 2 coord (x,y): 6 objects predicted over 1 second horizon ---
            Predictions predictions = Predictions(sensor_fusion, car_s, car_d, PARAM_NB_POINTS /* 50 */);

            int car_lane = get_lane(car_d);
            Behavior behavior = Behavior(sensor_fusion, car_lane, car_s, car_d, ref_vel /* car_vel */);
            vector<Target> targets = behavior.get_targets();

            // -- short time horizon (close to 100 msec when possible; not lower bcz of simulator latency) for trajectory (re)generation ---
            prev_size = min(prev_size, PARAM_TRUNCATED_PREV_SIZE);
            vector<double> frenet_close;
            if (prev_size > 0) { // prev_size typically close to 100 msec
              frenet_close = map.getFrenet(previous_path_x[prev_size-1], previous_path_y[prev_size-1], deg2rad(car_yaw));
              car_s = frenet_close[0];
              car_d = frenet_close[1];
            }
            car_lane = get_lane(car_d);

            vector<double> costs;
            vector<vector<vector<double>>> trajectories;
            vector<vector<PointC2>> prev_paths_s;
            vector<vector<PointC2>> prev_paths_d;

            for (size_t i = 0; i < targets.size(); i++) {
              // vector of (traj_x, traj_y)
              vector<vector<double>> trajectory;
              if (PARAM_TRAJECTORY_JMT) {
                struct trajectory_jmt traj_jmt;

                // generate JMT trajectory in s and d: converted then to (x,y) for trajectory output
                traj_jmt = generate_trajectory_jmt(targets[i], map, previous_path_x, previous_path_y, prev_size, prev_path_s, prev_path_d);
                trajectory = traj_jmt.trajectory;
                prev_paths_s.push_back(traj_jmt.path_s);
                prev_paths_d.push_back(traj_jmt.path_d);
              } else {
                // generate SPLINE trajectory in x and y
                trajectory = generate_trajectory(targets[i], map, car_x, car_y, car_yaw, car_s, car_d, 
                                                 previous_path_x, previous_path_y, prev_size);
              }

              double cost = cost_function(trajectory, targets[i], predictions, sensor_fusion, car_lane);
              costs.push_back(cost);
              trajectories.push_back(trajectory);
            }

            // --- retrieve the lowest cost trajectory ---
            double min_cost = INF;
            int min_cost_index = 0;
            for (size_t i = 0; i < costs.size(); i++) {
              if (costs[i] < min_cost) {
                min_cost = costs[i];
                min_cost_index = i;
              }
            }
            int target_lane = targets[min_cost_index].lane;
            ref_vel = targets[min_cost_index].velocity;
            if (PARAM_TRAJECTORY_JMT) {
              prev_path_s = prev_paths_s[min_cost_index];
              prev_path_d = prev_paths_d[min_cost_index];
            }

            if (target_lane != car_lane) {
              cout << "====================> CHANGE LANE: lowest cost for target " << min_cost_index << " = (target_lane=" << target_lane
                   << " target_vel=" << ref_vel << " car_lane=" << car_lane << " cost="<< min_cost << ")" << endl;
            }


            //////////////////////////////////////////////////////////////////////


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = trajectories[min_cost_index][0]; //next_x_vals;
          	msgJson["next_y"] = trajectories[min_cost_index][1]; //next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
