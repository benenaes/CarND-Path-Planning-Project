#include "Eigen-3.3/Eigen/Dense"

#include "conversion.h"
#include "spline.h"

#include "trajectory_generation.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(const vector<double>& start, const vector<double>& end, double T)
{
	/*
	Calculate the Jerk Minimizing Trajectory that connects the initial state
	to the final state in time T.

	INPUTS

	start - the vehicles start location given as a length three array
	corresponding to initial values of [s, s_dot, s_double_dot]

	end   - the desired end state for vehicle. Like "start" this is a
	length three array.

	T     - The duration, in seconds, over which this maneuver should occur.

	OUTPUT
	an array of length 6, each value corresponding to a coefficent in the polynomial
	s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

	EXAMPLE

	> JMT( [0, 10, 0], [10, 10, 0], 1)
	[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	*/
	MatrixXd A(3, 3);
	VectorXd b(3);

	double t2 = pow(T, 2);
	double t3 = pow(T, 3);
	double t4 = pow(T, 4);
	double t5 = pow(T, 5);

	A << 
		t3, t4, t5, 
		3 * t2, 4 * t3, 5 * t4, 
		6 * T, 12 * t2, 20 * t3;
	b << 
		(end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2)),
		(end[1] - (start[1] + start[2] * T)),
		(end[2] - start[2]);

	VectorXd x = A.colPivHouseholderQr().solve(b);

	return { start[0],start[1],start[2] * 0.5, x[0], x[1], x[2] };
}

double poly_eval(const vector<double>& coeffs, double x) 
{
	double temp = 0.0;
	for (size_t i = 0; i < coeffs.size(); i++) 
	{
		temp += coeffs[i] * pow(x, i);
	}
	return temp;
}

CalculatedTrajectory calculate_spline_trajectory_in_lane(
	vector<double>& pts_x,
	vector<double>& pts_y,
	unsigned int target_lane,
	const vector<double>& deltas,
	const vector<double>& map_waypoints_x,
	const vector<double>& map_waypoints_y,
	const vector<double>& map_waypoints_s)
{
	const double x_i = pts_x[pts_x.size() - 1];
	const double y_i = pts_y[pts_x.size() - 1];
	const double ref_yaw = atan2(y_i - pts_y[pts_x.size() - 2], x_i - pts_x[pts_x.size() - 2]);
	const vector<double> last_point = getFrenet(x_i, y_i, ref_yaw, map_waypoints_x, map_waypoints_y);
	const double last_point_s = last_point[0];
	const double last_point_d = convert_from_middle_of_lane_to_d(target_lane);

	vector<double> next_wp0 = getXY(last_point_s + 30, last_point_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(last_point_s + 60, last_point_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(last_point_s + 90, last_point_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	pts_x.push_back(next_wp0[0]);
	pts_x.push_back(next_wp1[0]);
	pts_x.push_back(next_wp2[0]);

	pts_y.push_back(next_wp0[1]);
	pts_y.push_back(next_wp1[1]);
	pts_y.push_back(next_wp2[1]);


	// Convert spline points to coordinate system with the last point of the precalculated trajectory as the origin 
	// and the axes of the car as x,y axes
	for (unsigned int i = 0; i < pts_x.size(); ++i)
	{
		const double shift_x = pts_x[i] - x_i;
		const double shift_y = pts_y[i] - y_i;

		pts_x[i] = (shift_x * cos(ref_yaw) + shift_y * sin(ref_yaw));
		pts_y[i] = (-shift_x * sin(ref_yaw) + shift_y * cos(ref_yaw));
	}

	tk::spline spline;

	spline.set_points(pts_x, pts_y);

	double x_add_on = 0;

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	for (unsigned int i = 0; i < deltas.size(); ++i)
	{
		double x_point = x_add_on + deltas[i];
		double y_point = spline(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		// Back to original coordinate system
		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point += x_i;
		y_point += y_i;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	return { next_x_vals, next_y_vals };
}