#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <vector>

struct CalculatedTrajectory
{
	std::vector<double> m_path_x;
	std::vector<double> m_path_y;
};

/*
Calculate a JMT
*/
std::vector<double> JMT(const std::vector<double>& start, const std::vector<double>& end, double T);

/*
Calculate the y position for a given x position on the JMT (given its coefficients) 
*/
double poly_eval(const std::vector<double>& coeffs, double x);

/*
Calculate a spline using the last two points in (pts_x, pts_y) and waypoints in the lane
*/
CalculatedTrajectory calculate_spline_trajectory_in_lane(
	std::vector<double>& pts_x,
	std::vector<double>& pts_y,
	unsigned int target_lane,
	const std::vector<double>& deltas,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s);

#endif