#include "utility.h"
#include "params.h"
#include <math.h>

double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }
double mph_to_ms(double mph) { return mph / 2.24; } // m.s-1
double ms_to_mph(double ms) { return ms * 2.24; } // mph

// d coord for left lane
double get_dleft(int lane) {
  double dleft = lane * PARAM_LANE_WIDTH;
  return dleft;
}

// d coord for right lane
double get_dright(int lane) {
  double dright = (lane + 1) * PARAM_LANE_WIDTH;
  return dright;
}

// d coord for center lane
double get_dcenter(int lane) {
  double dcenter = (lane + 0.5) * PARAM_LANE_WIDTH;
#ifndef _WIN32
  if (dcenter >= 10) {
    // this a workaround for a simulator issue I think (reported by others as well on udacity forums)
    // with d set to 10 from time to time a lane violation is reported by simulator
    // while everything looks fine
    dcenter = 9.8; // hack !!!
  }
#endif
  return dcenter;
}

int get_lane(double d) {
  return (int)(d / PARAM_LANE_WIDTH);
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
