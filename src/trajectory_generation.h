#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include <vector>

std::vector<double> JMT(const std::vector<double>& start, const std::vector<double>& end, double T);
double poly_eval(const std::vector<double>& coeffs, double x);

#endif