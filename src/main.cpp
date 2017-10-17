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

namespace plt = matplotlibcpp;

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

  int size = maps_x.size();

  if (size <= 200)
  {
	  for(int i = 0; i < size; i++)
	  {
	  	double map_x = maps_x[i];
	  	double map_y = maps_y[i];
	  	double dist = distance(x,y,map_x,map_y);
	  	if(dist < closestLen)
	  	{
	  		closestLen = dist;
	  		closestWaypoint = i;
	  	}
	  }
  }
  else // Faster search with big maps: 2 hierarchical steps of search
  {
    // 1) Search a point relatively close to the nearest
    int jump_points = size / 181; // so that we have 1 jump_points with a 181 points map (default)
    int point = 0;
	  while(point < size)
    {
	  	double map_x = maps_x[point];
	  	double map_y = maps_y[point];
	  	double dist = distance(x,y,map_x,map_y);
	  	if(dist < closestLen)
	  	{
	  		closestLen = dist;
	  		closestWaypoint = point;
	  	}
      point += jump_points;
    }

    // 2) Search a point which is the nearest in a refined area
	  //for(int i = closestWaypoint - 181; i < closestWaypoint + 181; i++)
	  for(int i = closestWaypoint - 91; i < closestWaypoint + 91; i++)
    {
      int idx = i;
      if (i < 0)
      {
        idx += size;
      }
      else if (i >= size)
      {
        idx -= size;
      }

	  	double map_x = maps_x[idx];
	  	double map_y = maps_y[idx];
	  	double dist = distance(x,y,map_x,map_y);
	  	if(dist < closestLen)
	  	{
	  		closestLen = dist;
	  		closestWaypoint = idx;
	  	}
    }
  }

  // TODO XX TEMP
  cout << "closestWaypoint=" << closestWaypoint << endl;
	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);
	angle = min(2*pi() - angle, angle); // XXX bug fix

	if(angle > pi()/4)
	{
		closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0; // XXX bug fix
    }
	}

  // XXX debug
  cout << "corrected closestWaypoint=" << closestWaypoint << endl;
	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	// XXX double frenet_s = 0;
	// XXX for(int i = 0; i < prev_wp; i++)
	// XXX {
	// XXX 	frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	// XXX }

  double frenet_s = maps_s[prev_wp];
	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

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
    cout << "frenet_s=" << frenet_s << " " << i << endl;
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
            //car_xy = getXY(frenet[0], frenet[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
            }

            bool too_close = false;

            // find ref_v to use
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              // car is in my lane
              float d = sensor_fusion[i][6];
              if (d > (2+4*lane-2) && d <(2+4*lane+2))
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // if using previous points can project s value outwards in time
                check_car_s+=((double)prev_size*0.02*check_speed);

                if ((check_car_s > car_s) && ((check_car_s - car_s) < 30))
                {
                  // do some logic here: lower reference velocity so we dont crash into the car infront of us, could
                  // also flag to try to change lane
                  //ref_vel = 29.5; //mph
                  too_close = true;
                  
                  //if (lane < 2)
                  //{
                  //  lane++;
                  //}

                  if (lane > 0)
                  {
                    lane--;
                  }
                  else
                  {
                    lane++;
                  }
                }  
              }
            }

            if (too_close)
            {
              ref_vel -= 2 * .224; // 5 m.s-2 under the 10 requirement
            }
            else if (ref_vel < 49.5)
            {
              ref_vel += 2 * .224;
            }
            

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if (prev_size < 2)
            {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            else
            {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);


            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


            for (int i = 0; i < ptsx.size(); i++)
            {
              // shift car reference angle to 0 degrees
              // transformation to local car's coordinates (cf MPC)
              // last point of previous path at origin and its angle at zero degree

              // shift and rotation
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
            }


            tk::spline spl;
            spl.set_points(ptsx, ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            for (int i = 0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = spl(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            // fill up the rest of our path planner after filing it with previous points
            // here we will always output 50 points
            for (int i = 1; i <= 50-previous_path_x.size(); i++)
            {
              double N = (target_dist / (0.02*ref_vel/2.24)); // divide by 2.24: mph -> m/s
              double x_point = x_add_on + target_x/N;
              double y_point = spl(x_point);

              x_add_on = x_point;

              double x_ref = x_point; // x_ref IS NOT ref_x !!!
              double y_ref = y_point;

              // rotate back to normal after rotating it earlier
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            //////////////////////////////////////////////////////////////////////


          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

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
