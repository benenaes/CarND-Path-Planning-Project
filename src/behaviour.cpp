#include "behaviour.h"
#include "cost_functions.h"
#include "conversion.h"

#include <vector>
#include <stdexcept>
#include <optional>
#include <limits>
#include <string>

using namespace std;

struct EgoCarTarget
{
	float m_TargetSpeed;
	unsigned int m_OriginalLane;
	unsigned int m_TargetLane;

	EgoCarTarget(float target_speed, unsigned int original_lane, unsigned int target_lane) :
		m_TargetSpeed(target_speed), m_OriginalLane(original_lane), m_TargetLane(target_lane) {}
};

Behaviour::Behaviour( 
	const unsigned int total_lanes, 
	const float normal_acceleration,
	const float target_speed):
	m_CurrentManoeuver(Manoeuver::FOLLOWING_LANE),
	m_NormalAcceleration(normal_acceleration),
	m_TargetSpeed(target_speed),
	m_TotalLanes(total_lanes)
{
}

pair<optional<CarPrediction>, optional<CarPrediction>> vehicle_up_front_and_back(
	const CarState& ego_car,
	const vector<CarPrediction>& cars_of_particular_lane,
	unsigned int point_in_time)
{
	optional<CarPrediction> up_front_car = nullopt;
	optional<CarPrediction> back_car = nullopt;

	if (cars_of_particular_lane.size() == 0)
	{
		return make_pair(up_front_car, back_car);
	}

	if (point_in_time >= cars_of_particular_lane[0].m_PositionPredictions.size())
	{
		throw invalid_argument("Wrong point_in_time argument to vehicle_up_front_and_back: " 
			+ to_string(point_in_time) + " of " + to_string(cars_of_particular_lane[0].m_PositionPredictions.size()));
	}

	float up_front_s = numeric_limits<float>::max();
	int front_index = -1;
	float back_s = numeric_limits<float>::min();
	int back_index = -1;

	for (unsigned int i=0; i < cars_of_particular_lane.size(); ++i)
	{
		const CarPrediction& car_prediction = cars_of_particular_lane[i];
		const float other_car_s = car_prediction.m_PositionPredictions[point_in_time].m_s;
		if (other_car_s < ego_car.m_s)
		{
			if (back_s < other_car_s)
			{
				back_s = other_car_s;
				back_index = i;
			}
		}
		else
		{
			if (up_front_s > other_car_s)
			{
				up_front_s = other_car_s;
				front_index = i;
			}
		}
	}

	if (front_index != -1)
	{
		up_front_car = cars_of_particular_lane[front_index];
	}
	if (back_index != -1)
	{
		back_car = cars_of_particular_lane[back_index];
	}

	return make_pair(up_front_car, back_car);
}

vector<vector<CarPrediction>> bin_vehicles_by_lane(
	const vector<CarPrediction>& close_by_car_predictions,
	unsigned int point_in_time,
	unsigned int total_lanes)
{
	vector<std::vector<CarPrediction>> car_predictions_per_lane;
	car_predictions_per_lane.resize(total_lanes);

	if (close_by_car_predictions.size() == 0)
	{
		return car_predictions_per_lane;
	}

	if (point_in_time >= close_by_car_predictions[0].m_PositionPredictions.size())
	{
		throw invalid_argument("Wrong point_in_time argument to sort_vehicles_by_lane: " 
			+ to_string(point_in_time) + " of " + to_string(close_by_car_predictions[0].m_PositionPredictions.size()));
	}

	for (const CarPrediction& car_prediction : close_by_car_predictions)
	{
		const unsigned int lane = convert_from_d_to_lane(car_prediction.m_PositionPredictions[point_in_time].m_d);

		if (lane >= total_lanes)
		{
			throw invalid_argument("Wrong d argument to bin_vehicles_by_lane: " + to_string(lane));
		}

		car_predictions_per_lane[lane].push_back(car_prediction);
	}

	return car_predictions_per_lane;
}

void Behaviour::calculate_new_behaviour(
	const CarState& ego_car,
	const PredictionCalculator& prediction_calculator,
	const PrecalculatedTrajectory& precalculated_trajectory)
{
	// If FOLLOWING_LANE:
	//	- If ego_car v = TARGET_SPEED:
	//        - If other cars up front in same lane have delta_s_upfront >= SAFETY_DISTANCE
	//				* Add possible state: "Keep velocity; add target up front"
	//        - Else:
	//				- Add possible state: "Slow down; target up front"
	//				- For each adjacent lane:
	//					- if delta_s_upfront >= SAFETY_DISTANCE_FRONT and delta_s_back <= SAFETY_DISTANCE_BACK:
	//						* Add possible state: "Change lane; velocity of car up front if any, otherwise keep velocity"
	//  - Else: (can be combined with if counterpart)
	//        - If other cars up front in same lane have delta_s_upfront >= SAFETY_DISTANCE
	//				* Add possible state: "Accelerate; add target up front"
	//		  - Else:
	//				- Add possible state: "Slow down; target up front"
	//				- For each adjacent lane:
	//					- if delta_s_upfront >= SAFETY_DISTANCE_FRONT and delta_s_back <= SAFETY_DISTANCE_BACK:
	//						* Add possible state: "Change lane; velocity of car up front if any, otherwise accelerate"
	// If CHANGE_LANE:
	// - If other cars up front in target lane has delta_s_upfront >= SAFETY_DISTANCE
	//	  * Add possible state: Keep changing lane with target lane, target s and target velocity
	// - Else:
	//    * Add possible state: change line to other direction with target velocity of up-front velocity

	const float safety_distance_front = 30; // meter
	const float safety_distance_back = 10; // meter

	const int previous_size = precalculated_trajectory.m_previous_path_x.size();

	float car_s = ego_car.m_s;

	if (previous_size > 0)
	{
		car_s = precalculated_trajectory.m_end_path_s;
	}

	vector<vector<CarPrediction>> vehicle_prediction_bins_per_lane = bin_vehicles_by_lane(
		prediction_calculator.get_car_predictions(),
		previous_size,
		m_TotalLanes);
	const unsigned int current_ego_lane = convert_from_d_to_lane(ego_car.m_d);

	using CarsInLane = pair<optional<CarPrediction>, optional<CarPrediction>>;
	vector<CarsInLane> cars_per_lane;
	vector<EgoCarTarget> targets;

	for (unsigned int i = 0; i < m_TotalLanes; ++i)
	{
		cars_per_lane.emplace_back(vehicle_up_front_and_back(
			ego_car,
			vehicle_prediction_bins_per_lane[i],
			previous_size));

		const CarsInLane& up_front_and_back = cars_per_lane[i];

		if (up_front_and_back.first)
		{
			float target_velocity = m_TargetSpeed;
			if (up_front_and_back.first->m_PositionPredictions[previous_size].m_s - ego_car.m_s < safety_distance_front)
			{
				target_velocity = up_front_and_back.first->m_SpeedPrediction;
			}
			targets.emplace_back(target_velocity, current_ego_lane, i);
		}
		else
		{
			targets.emplace_back(m_TargetSpeed, current_ego_lane, i);
		}
	}

	for (const EgoCarTarget& target : targets)
	{
		vector<double> pts_x;
		vector<double> pts_y;

		// Still going for the same target; reuse precalculated path
		if (target.m_TargetLane == m_TargetLane)
		{

		}
	}

	

	switch (m_CurrentManoeuver)
	{
	case Manoeuver::FOLLOWING_LANE:
	{
		const CarsInLane& up_front_and_back = cars_per_lane[current_ego_lane];

		if (up_front_and_back.first)
		{
			float target_velocity = m_TargetSpeed;
			if (up_front_and_back.first->m_PositionPredictions[previous_size].m_s - ego_car.m_s < safety_distance_front)
			{
				target_velocity = up_front_and_back.first->m_SpeedPrediction;
			}
			targets.emplace_back(target_velocity, current_ego_lane, current_ego_lane);
		}
		else
		{
			targets.emplace_back(m_TargetSpeed, current_ego_lane, current_ego_lane);
		}

		break;
	}
	}
}

