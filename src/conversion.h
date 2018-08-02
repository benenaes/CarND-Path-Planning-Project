#ifndef CONVERSION_HPP
#define CONVERSION_HPP

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h> 

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

std::vector<double> getFrenet(
	double x,
	double y,
	double theta,
	const std::vector<double>& maps_x,
	const std::vector<double>& maps_y);

std::vector<double> getXY(
	double s,
	double d,
	const std::vector<double>& maps_s,
	const std::vector<double>& maps_x,
	const std::vector<double>& maps_y);

float convert_from_middle_of_lane_to_d(unsigned int lane);
unsigned int convert_from_d_to_lane(float d);

float mph_to_mps(float mph);
float mps_to_mph(float mps);

#endif