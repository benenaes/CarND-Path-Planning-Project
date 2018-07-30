#ifndef COST_FUNCTIONS_HPP
#define COST_FUNCTIONS_HPP

#include <vector>

#include "car_state.h"
#include "predict.h"

float goal_distance_cost(
	int goal_lane, int intended_lane, int final_lane, float distance_to_goal);

float speed_cost(float speed_limit, float current_velocity);

float safety_car_distance(
	const CarState& car_state, 
	const FrenetCoordinate& goal,
	std::vector<CarPrediction>& car_predictions, 
	float min_distance);


#endif