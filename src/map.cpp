#include "utility.h"
#include "params.h"

#include <vector>
#include <iostream>
#include <sstream>
#include <math.h>

#include "map.h"
#ifndef WIN32
#include "matplotlibcpp.h"
#endif

#include <time.h>


#ifndef WIN32
namespace plt = matplotlibcpp;
#endif
using namespace std;

double MAX_S;

/**
 * Initializes Vehicle
 */
void Map::read(string map_file) {
  ifstream in_map_(map_file.c_str(), ifstream::in);
  string line;
  bool not_started = true;
  double x0, y0, dx0, dy0;

  double last_s = 0;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  while (getline(in_map_, line)) {
    istringstream iss(line);
  	double x, y;
  	float s, d_x, d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
    if (not_started) {
      x0 = x; y0 = y; dx0 = d_x; dy0 = d_y;
      not_started = false;
    }

    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    last_s = s;
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);

  	map_waypoints_normx.push_back(x+10*d_x);
  	map_waypoints_normy.push_back(y+10*d_y);
  }
  assert(map_waypoints_x.size() && "map not loaded, probably path include is missing");

  if (PARAM_MAP_BOSCH == true)
    MAX_S = last_s;
  else
    MAX_S = MAXIMUM_S;

  // to get a good spline approximation on last segment wrapping around
  if (PARAM_MAP_BOSCH == false) {
    map_waypoints_x.push_back(x0);
    map_waypoints_y.push_back(y0);
    map_waypoints_s.push_back(MAX_S);
    map_waypoints_dx.push_back(dx0);
    map_waypoints_dy.push_back(dy0);
    map_waypoints_normx.push_back(x0+10*dx0);
    map_waypoints_normy.push_back(y0+10*dy0);
  }

  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  // remove last point so we do not have duplicates (x,y): it was just for spline continuity at wraparound
  if (PARAM_MAP_BOSCH == false) {
    map_waypoints_x.pop_back();
    map_waypoints_y.pop_back();
    map_waypoints_s.pop_back();
    map_waypoints_dx.pop_back();
    map_waypoints_dy.pop_back();
    map_waypoints_normx.pop_back();
    map_waypoints_normy.pop_back();
  }

  double len_ref = 0;
  double prev_x = spline_x(0);
  double prev_y = spline_y(0);
  for (double s = 1; s <= floor(MAX_S); s++) {
    double x = spline_x(s);
    double y = spline_y(s);
    len_ref += distance(x, y, prev_x, prev_y);
    prev_x = x;
    prev_y = y;
  }
  cout << "len_ref=" << len_ref << endl;

  // map with higher precision: 1 point every 1 meter (instead of every 30 meters)
  new_map_waypoints_x;
  new_map_waypoints_y;
  new_map_waypoints_dx;
  new_map_waypoints_dy;
  for (double s = 0; s <= floor(MAX_S); s++) {
    double x = spline_x(s);
    double y = spline_y(s);
    double dx = spline_dx(s);
    double dy = spline_dy(s);

    new_map_waypoints_x.push_back(x);
    new_map_waypoints_y.push_back(y);
    new_map_waypoints_dx.push_back(dx);
    new_map_waypoints_dy.push_back(dy);
  }

	double frenet_s = 0.0;
  map_s.push_back(0.0);
	for (size_t i = 1; i < map_waypoints_x.size(); i++) {
		frenet_s += distance(map_waypoints_x[i], map_waypoints_y[i], map_waypoints_x[i-1], map_waypoints_y[i-1]);
    map_s.push_back(frenet_s);
	}

	frenet_s = 0.0;
  new_map_s.push_back(0.0);
  // new map: 1 point every meter
	for (size_t i = 1; i < new_map_waypoints_x.size(); i++) {
		frenet_s += distance(new_map_waypoints_x[i], new_map_waypoints_y[i], new_map_waypoints_x[i-1], new_map_waypoints_y[i-1]);
    //new_map_s.push_back(frenet_s); // TODO test both alternatives
    new_map_s.push_back(i); // better
    cout << "frenet_s=" << frenet_s << " " << i << endl;
	}

  max_error = 0.0;
  sum_error = 0.0;
  avg_error = 0.0;
  num_error = 0;
}

Map::~Map() {}


#ifndef WIN32
void Map::plot(void) {
  plt::title("Map");
  plt::plot(map_waypoints_x, map_waypoints_y, "r*");
  plt::plot(map_waypoints_normx, map_waypoints_normy, "g*");

  //cout << new_map_waypoints_x.size() << " " << new_map_waypoints_y << endl;
  plt::plot(new_map_waypoints_x, new_map_waypoints_y, "b-");
  vector<double> car_x = { 1, 770.0906};
  vector<double> car_y = { 1, 1129.872};
  plt::plot(car_x, car_y, "gx");
  plt::show();
}
#endif

int Map::ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

  int size = maps_x.size();

  if (size <= 200) {
	  for (int i = 0; i < size; i++) {
	  	double map_x = maps_x[i];
	  	double map_y = maps_y[i];
	  	double dist = distance(x,y,map_x,map_y);
	  	if(dist < closestLen) {
	  		closestLen = dist;
	  		closestWaypoint = i;
	  	}
	  }
  } else  { // Faster search with big maps: 2 hierarchical steps of search
    // 1) Search a point relatively close to the nearest
    int jump_points = size / 181; // so that we have 1 jump_points with a 181 points map (default)
    int point = 0;
	  while(point < size)
    {
	  	double map_x = maps_x[point];
	  	double map_y = maps_y[point];
	  	double dist = distance(x,y,map_x,map_y);
	  	if(dist < closestLen) {
	  		closestLen = dist;
	  		closestWaypoint = point;
	  	}
      point += jump_points;
    }

    // 2) Search a point which is the nearest in a refined area
	  //for (int i = closestWaypoint - 181; i < closestWaypoint + 181; i++)
	  for (int i = closestWaypoint - 91; i < closestWaypoint + 91; i++) {
      int idx = i;
      if (i < 0) {
        idx += size;
      } else if (i >= size) {
        idx -= size;
      }

	  	double map_x = maps_x[idx];
	  	double map_y = maps_y[idx];
	  	double dist = distance(x,y,map_x,map_y);
	  	if(dist < closestLen) {
	  		closestLen = dist;
	  		closestWaypoint = idx;
	  	}
    }
  }

  // TODO XX TEMP
  //cout << "closestWaypoint=" << closestWaypoint << endl;
	return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);
	angle = min(2*M_PI - angle, angle); // XXX bug fix

	if(angle > M_PI/4)
	{
		closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0; // XXX bug fix
    }
	}

  // XXX debug
  //cout << "corrected closestWaypoint=" << closestWaypoint << endl;
	return closestWaypoint;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta) {
  vector<double> &maps_s = this->new_map_s; ; 
  vector<double> &maps_x = this->new_map_waypoints_x;
  vector<double> &maps_y = this->new_map_waypoints_y;

	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0) {
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

  if (PARAM_MAP_BOSCH == false) {
	  double center_x = PARAM_CENTER_X - maps_x[prev_wp];
	  double center_y = PARAM_CENTER_Y - maps_y[prev_wp];
	  double centerToPos = distance(center_x,center_y,x_x,x_y);
	  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	  if (centerToPos <= centerToRef) {
	  	frenet_d *= -1;
	  }
  }

  double frenet_s = maps_s[prev_wp]; // XXX faster
	frenet_s += distance(0,0,proj_x,proj_y);

  assert(frenet_d >= 0);

	return {frenet_s, frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d) {
  vector<double> &maps_s = map_waypoints_s; ; 
  vector<double> &maps_x = map_waypoints_x;
  vector<double> &maps_y = map_waypoints_y;

	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};
}

vector<double> Map::getXYspline(double s, double d) {
  s = fmod(s, MAX_S); // bug fix for JMT wraparound
	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);

	return {x,y};
}

double Map::getSpeedToFrenet(double Vxy, double s) {
  s = fmod(s, MAX_S);
  double dx_over_ds = spline_x.deriv(1, s);
  double dy_over_ds = spline_y.deriv(1, s);
  double Vs = (Vxy / sqrt(dx_over_ds*dx_over_ds + dy_over_ds*dy_over_ds));
  return Vs;
}

double Map::testError(double car_x, double car_y, double car_yaw) {
  double error = 0;

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
  vector<double> frenet = getFrenet(car_x, car_y, deg2rad(car_yaw));

  double frenet_s = frenet[0];
  double frenet_d = frenet[1];

  vector<double> car_xy = getXYspline(frenet_s, frenet_d);

  clock_t stop = clock();
  double elapsed = (double)(stop - start) * 1000000.0 / CLOCKS_PER_SEC;

  error = distance(car_xy[0], car_xy[1], car_x, car_y);
  sum_error += error;
  num_error++;
  avg_error = sum_error / num_error;
  //assert(error < 4);
  if (error > max_error) {
    max_error = error;
  }

  cout << "error=" << error << " trt_time=" << elapsed << " us (max_error=" << max_error <<  " avg_error=" << avg_error << ")" << endl;

  return error;
}
