#ifndef UTILITY_H
#define UTILITY_H

#include <vector>

// Computationnal defines
#define INF 1e10
enum {
  ID = 0, // 0
  X  = 1, // 1
  Y  = 2, // 2
  VX = 3, // 3
  VY = 4, // 4
  S  = 5, // 5
  D  = 6, // 6
  //SIZE    // 7
};

struct Coord {
  double x;
  double y;
};

struct Frenet {
  double s;
  double d;
};


struct CarData {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  double speed_target;
  int    lane;
  bool   emergency;
  CarData (double X=0, double Y=0, double S=0, double D=0, double YAW=0, 
           double V=0, double VF=0, int L=0, bool E=false) : x(X), y(Y), s(S), yaw(YAW), 
           speed(V), speed_target(VF), lane(L), emergency(E) {}
};

typedef std::vector<double > t_coord;
typedef std::vector<t_coord> t_traj;
typedef std::vector<t_traj > t_trajSet;


// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);
double mph_to_ms(double mph); // m.s-1
double ms_to_mph(double ms);

// d coord for "left lane" of a lane
double get_dleft(int lane);
// d coord for "right lane" of a lane
double get_dright(int lane);
// d coord for "center lane" of a lane
double get_dcenter(int lane);

int get_lane(double d);

double distance(double x1, double y1, double x2, double y2);

#ifndef _WIN32
#else
// SCANeR-specific
#define M_PI 3.14159
#endif

#endif // UTILITY_H
