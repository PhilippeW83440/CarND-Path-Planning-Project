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

#include "spline.h"
#include <time.h>
#include "matplotlibcpp.h"

#include "coord.h"
#include "behavior.h"
#include "trajectory.h"
#include "cost.h"

namespace plt = matplotlibcpp;

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

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  vector<double> map_waypoints_normx;
  vector<double> map_waypoints_normy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;

  bool not_started = true;
  double x0;
  double y0;
  double dx0;
  double dy0;

  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
    if (not_started)
    {
      x0 = x; y0 = y; dx0 = d_x; dy0 = d_y;
      not_started = false;
    }
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);

  	map_waypoints_normx.push_back(x+10*d_x);
  	map_waypoints_normy.push_back(y+10*d_y);
  }

  // to get a good spline approximation on last segment wrapping around
  map_waypoints_x.push_back(x0);
  map_waypoints_y.push_back(y0);
  map_waypoints_s.push_back(max_s);
  map_waypoints_dx.push_back(dx0);
  map_waypoints_dy.push_back(dy0);
  map_waypoints_normx.push_back(x0+10*dx0);
  map_waypoints_normy.push_back(y0+10*dy0);

  plt::title("Map");
  plt::plot(map_waypoints_x, map_waypoints_y, "r*");
  plt::plot(map_waypoints_normx, map_waypoints_normy, "g*");

  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;

  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  // remove last point so we do not have duplicates (x,y): it was just for spline continuity at wraparound
  map_waypoints_x.pop_back();
  map_waypoints_y.pop_back();
  map_waypoints_s.pop_back();
  map_waypoints_dx.pop_back();
  map_waypoints_dy.pop_back();
  map_waypoints_normx.pop_back();
  map_waypoints_normy.pop_back();


  cout << "x(990)=" << spline_x(990) << " y(990)=" << spline_y(990) << endl;

  double len_ref = 0;
  double prev_x = spline_x(0);
  double prev_y = spline_y(0);
  for (double s = 1; s <= 6945; s++)
  {
    double x = spline_x(s);
    double y = spline_y(s);
    len_ref += distance(x, y, prev_x, prev_y);
    prev_x = x;
    prev_y = y;
  }
  cout << "len_ref=" << len_ref << endl;

  double len_lane;

  for (int i = 0; i < 3; i++)
  {
    double laned = 2 + 4 * i;
    len_lane = 0;
    prev_x = spline_x(0) + laned * spline_dx(0);
    prev_y = spline_y(0) + laned * spline_dy(0);
    for (double s = 1; s <= 6945; s++)
    {
      double x = spline_x(s) + laned * spline_dx(s);
      double y = spline_y(s) + laned * spline_dy(s);
      len_lane += distance(x, y, prev_x, prev_y);
      prev_x = x;
      prev_y = y;
    }
    cout << "len_lane" << i << "=" << len_lane << endl;
  }

  // map with higher precision: 1 point every 1 meter (instead of every 30 meters)
  vector<double> new_map_waypoints_x;
  vector<double> new_map_waypoints_y;
  vector<double> new_map_waypoints_dx;
  vector<double> new_map_waypoints_dy;
  for (double s = 0; s <= 6945; s++)
  {
    double x = spline_x(s);
    double y = spline_y(s);
    double dx = spline_dx(s);
    double dy = spline_dy(s);

    new_map_waypoints_x.push_back(x);
    new_map_waypoints_y.push_back(y);
    new_map_waypoints_dx.push_back(dx);
    new_map_waypoints_dy.push_back(dy);
  }

  plt::plot(new_map_waypoints_x, new_map_waypoints_y, "b-");
  vector<double> car_x = { 1, 770.0906};
  vector<double> car_y = { 1, 1129.872};
  plt::plot(car_x, car_y, "gx");
  plt::show();

	double frenet_s = 0;
  vector<double> map_s;
  map_s.push_back(0.0);
	for(int i = 1; i < map_waypoints_x.size(); i++)
	{
		frenet_s += distance(map_waypoints_x[i], map_waypoints_y[i], map_waypoints_x[i-1], map_waypoints_y[i-1]);
    map_s.push_back(frenet_s);
	}

	frenet_s = 0;
  vector<double> new_map_s;
  new_map_s.push_back(0.0);
	for(int i = 1; i < new_map_waypoints_x.size(); i++)
	{
		frenet_s += distance(new_map_waypoints_x[i], new_map_waypoints_y[i], new_map_waypoints_x[i-1], new_map_waypoints_y[i-1]);
    //new_map_s.push_back(frenet_s); // TODO test both alternatives
    new_map_s.push_back(i); // better
    //cout << "frenet_s=" << frenet_s << " " << i << endl;
	}


  //////////////////////////////////////////////////////////////////////
  int lane = 1;
  double ref_vel = 0.0; // mph
  double max_error = 0.0;
  double sum_error = 0.0;
  double avg_error = 0.0;
  unsigned int num_error = 0;
  //////////////////////////////////////////////////////////////////////


  h.onMessage([&sum_error, &avg_error, &num_error, &map_s, &new_map_s, &new_map_waypoints_x, &new_map_waypoints_y, &new_map_waypoints_dx, &new_map_waypoints_dy, &spline_x, &spline_y, &spline_dx, &spline_dy, &max_error,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
                //vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


            // TEST XXX TODO TEMP
            //car_x = 770.0906;
            //car_y = 1129.872;
            //car_s = 6931.203;
            //car_d = 6.087758;
            //car_yaw = 375.72;
            //car_speed = 39.366;

            //car_x = 783.6693;
            //car_y = 1129.419;
            //car_s = 6944.789;
            //car_d = 6.174933;
            //car_yaw = 358.449;
            //car_speed = 41.1659;

            // check transformations accuracy
            clock_t start = clock();
            vector<double> frenet = getFrenet(car_x, car_y, deg2rad(car_yaw), new_map_waypoints_x, new_map_waypoints_y, new_map_s);
            //vector<double> frenet = getFrenet(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y, map_s);
            int frenet_s = frenet[0];
            int frenet_d = frenet[1];

            double my_x = spline_x(frenet_s) + frenet_d * spline_dx(frenet_s);
            double my_y = spline_y(frenet_s) + frenet_d * spline_dy(frenet_s);
            //auto car_xy = getXY(frenet_s, frenet_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //auto car_xy = getXY(frenet_s, frenet_d, new_map_s, new_map_waypoints_x, new_map_waypoints_y);
            //my_x = car_xy[0];
            //my_y = car_xy[1];

            clock_t stop = clock();
            double elapsed = (double)(stop - start) * 1000000.0 / CLOCKS_PER_SEC;

            double error = distance(my_x, my_y, car_x, car_y);
            sum_error += error;
            num_error++;
            avg_error = sum_error / num_error;
            assert(error < 4);
            if (error > max_error)
            {
              max_error = error;
            }

          	json msgJson;


            int prev_size = previous_path_x.size();

            cout << "error=" << error << " trt_time=" << elapsed << " us (max_error=" << max_error <<  " avg_error=" << avg_error << ")" 
                 << " prev_size=" << prev_size << endl;

            //////////////////////////////////////////////////////////////////////

            if (prev_size > 0)
            {
              car_s = end_path_s;
              car_d = end_path_d;
            }

            // TODO add predictions

            vector<vector<double>> targets = behavior_planner_find_targets(sensor_fusion, prev_size, lane /* car_lane */, 
                                                                           car_s, car_d, ref_vel /* car_vel */);

            vector<double> costs;
            vector<vector<vector<double>>> trajectories;
            for (int i = 0; i < targets.size(); i++)
            {
              lane = targets[i][0];
              ref_vel = targets[i][1];

              // vector of (traj_x, traj_y)
              vector<vector<double>> trajectory = generate_trajectory(lane, ref_vel, map_waypoints_s, map_waypoints_x, map_waypoints_y, 
                                                                      car_x, car_y, car_yaw, car_s, car_d, previous_path_x, previous_path_y);

              // achtung TODO ... la trajectoire est potentiellement tronquee en fait: OK pu pas ? */
              double cost = cost_function(trajectory, lane, ref_vel, 2.0 /* sec */); // TODO add predictions
              costs.push_back(cost);
              trajectories.push_back(trajectory);
            }

            double min_cost = 1e10;
            int min_cost_index = 0;
            for (int i = 0; i < costs.size(); i++)
            {
              if (costs[i] < min_cost)
              {
                min_cost = costs[i];
                min_cost_index = i;
              }
            }
            cout << "lowest cost for target " << min_cost_index << " = (lane=" << targets[min_cost_index][0] 
                 << ", vel=" << targets[min_cost_index][1] << ", cost="<< min_cost << ")" << endl;


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
