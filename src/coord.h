#ifndef COORD_H
#define COORD_H

#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
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

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y, const vector<double> &maps_s);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


#endif // COORD_H
