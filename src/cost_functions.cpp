#include <math.h>

#include "conversion.h"
#include "spline.h"

#include "cost_functions.h"

using namespace std;

float goal_distance_cost(
	int goal_lane, int intended_lane, int final_lane, float distance_to_goal)
{
	/*
	The cost increases with both the distance of intended lane from the goal
	and the distance of the final lane from the goal. The cost of being out of the
	goal lane also becomes larger as vehicle approaches the goal.
	*/

	const double delta_d = abs(intended_lane + final_lane - goal_lane);
	const float cost = 1 - 1 / exp(delta_d / distance_to_goal);

	return cost;
}

float speed_cost(float speed_limit, float current_velocity)
{
	const float buffer_velocity = 0.5;
	const float stop_cost = 0.8;

	float cost;
	const float target_speed = speed_limit - buffer_velocity;

	if (current_velocity < target_speed)
	{
		cost = stop_cost * (target_speed - current_velocity) / target_speed;
	}
	else if (current_velocity < speed_limit)
	{
		cost = (current_velocity - target_speed) / buffer_velocity;
	}
	else
	{
		cost = 1;
	}
}

float feasibility_acceleration_cost(float acceleration, float max_acceleration)
{
	if (abs(acceleration) < max_acceleration)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

float safety_off_the_road_cost(float d, float left_lane_d, float right_lane_d)
{
	if (d >= left_lane_d && d <= right_lane_d)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

float safety_car_distance(
	const CarState& car_state, 
	const FrenetCoordinate& goal, 
	const vector<CarPrediction>& car_predictions, 
	float min_distance)
{
	float cost = 0;

	for (unsigned int i = 0; i < car_predictions.size(); ++i)
	{
		float current_car_cost = 0;

		const CarPrediction& current_car_prediction = car_predictions[i];

		vector<double> pts_s;
		vector<double> pts_d;

		for (unsigned int j = 0; j < current_car_prediction.m_PositionPredictions.size(); ++j)
		{
			pts_s.push_back(current_car_prediction.m_PositionPredictions[j].m_s);
			pts_d.push_back(current_car_prediction.m_PositionPredictions[j].m_d);
		}

		tk::spline s;

		s.set_points(pts_s, pts_d);

		for (unsigned int j = 0; j < current_car_prediction.m_PositionPredictions.size(); ++j)
		{
			const double target_x = 30.0;
			const double target_y = s(target_x);
			const double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
		}
	}

	return cost;
}

float off_center_lane_cost(float d)
{
	const float lane_center = convert_from_middle_of_lane_to_d(convert_from_d_to_lane(d));

	const float cost = 1 - 1 / exp(d - lane_center);
}