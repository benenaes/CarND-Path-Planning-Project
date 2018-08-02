#include "trajectory_generation.h"

#include "Eigen-3.3/Eigen/Dense"

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