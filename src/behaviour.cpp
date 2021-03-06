#include "behaviour.h"

#include "cost_functions.h"
#include "conversion.h"

#include <vector>
#include <stdexcept>
#include <optional>
#include <limits>
#include <string>
#include <iostream>
#include <algorithm>
#include <math.h>

using namespace std;

const float safety_distance_front = 10; // meter
const float safety_distance_back = 5; // meter
const float min_speed_diff_to_change_lane = 2; // m/s
const float max_distance_to_change_lane = 50; // m: max distance for a car up front to consider a lane change

const double time_interval = 0.02; // 20 ms
const double max_sim_latency = 0.1; // 200 ms

// min points to predict to take max sim latency into account
const unsigned int min_points_to_predict = (unsigned int) ceil(max_sim_latency / time_interval);

const unsigned int prediction_horizon = 100; // timesteps = 2 secs
const unsigned int lane_size = 4; // m : size of a lane
const double time_to_change_lane = 2.f; // secs

// m/s : velocity delta with car in the back to slow down such that the ego car gets eventually behind that car
const double slow_down_margin = 5.f; 

// m/s : margin with target speed to cause acceleration/deceleration (and thus trajectory recalculation) of the ego car
const double acceleration_margin = 0.5;

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
	const float ideal_speed,
	const unsigned int current_lane):
	m_CurrentManoeuver(Manoeuver::FOLLOWING_LANE),
	m_NormalAcceleration(normal_acceleration),
	m_IdealSpeed(ideal_speed),
	m_TotalLanes(total_lanes),
	m_TargetLane(current_lane),
	m_TargetSpeed(0),
	m_TargetS(0),
	m_CarIdToSlowdownFor(-1)
{
}

Behaviour::CarsInLane Behaviour::vehicle_up_front_and_back(
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

vector<vector<CarPrediction>> Behaviour::bin_vehicles_by_lane(
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
		throw invalid_argument("Wrong point_in_time argument to bin_vehicles_by_lane: " 
			+ to_string(point_in_time) + " of " + to_string(close_by_car_predictions[0].m_PositionPredictions.size()));
	}

	for (const CarPrediction& car_prediction : close_by_car_predictions)
	{
		const int lane = convert_from_d_to_lane(car_prediction.m_PositionPredictions[point_in_time].m_d);

		if (lane > total_lanes && lane < 0)
		{
			cout << "Car prediction out of lanes, d coord: " << car_prediction.m_PositionPredictions[point_in_time].m_d << endl;
		}
		else
		{
			car_predictions_per_lane[lane].push_back(car_prediction);
		}
	}

	return car_predictions_per_lane;
}

CalculatedTrajectory Behaviour::consider_lane_change(
	const CarState& ego_car,
	const vector<CarsInLane>& cars_per_lane,
	float fastest_velocity_up_front,
	float lane_with_fastest_velocity,
	const CalculatedTrajectory& precalculated_trajectory,
	const vector<double>& map_waypoints_x,
	const vector<double>& map_waypoints_y,
	const vector<double>& map_waypoints_s)
{
	const unsigned int current_ego_lane = convert_from_d_to_lane(ego_car.m_d);

	cout << "Current target lane: " << m_TargetLane << endl;
	cout << "Considering another lane: " << lane_with_fastest_velocity << endl;
	cout << "Velocity there: " << fastest_velocity_up_front << endl;

	if (lane_with_fastest_velocity < current_ego_lane)
	{
		m_TargetLane = current_ego_lane - 1;
	}
	else
	{
		m_TargetLane = current_ego_lane + 1;
	}

	const CarsInLane& up_front_and_back_in_target_lane = cars_per_lane[m_TargetLane];
	const optional<CarPrediction>& optional_car_up_front_target_lane = up_front_and_back_in_target_lane.first;
	const optional<CarPrediction>& optional_car_behind_target_lane = up_front_and_back_in_target_lane.second;

	if (optional_car_up_front_target_lane && optional_car_behind_target_lane)
	{
		if (optional_car_up_front_target_lane->m_PositionPredictions[0].m_s > ego_car.m_s + safety_distance_front &&
			optional_car_behind_target_lane->m_PositionPredictions[0].m_s < ego_car.m_s - safety_distance_back)
		{
			cout << "Could change lanes" << endl;
			m_CurrentManoeuver = Manoeuver::CHANGE_LANE;
			m_TargetSpeed = optional_car_up_front_target_lane->m_SpeedPrediction;
			m_TargetS = ego_car.m_s + (m_TargetSpeed + ego_car.m_v) / 2 * time_to_change_lane;
			return create_trajectory_for_lane_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s,
				time_to_change_lane);
		}
		else
		{
			cout << "Too tight to change lanes, should change velocity" << endl;
			cout << "Current s of ego car: " << ego_car.m_s
				<< ", s of car upfront: "
				<< optional_car_up_front_target_lane->m_PositionPredictions[0].m_s
				<< ", s of car behind: "
				<< optional_car_behind_target_lane->m_PositionPredictions[0].m_s << endl;
			m_TargetLane = current_ego_lane;
			m_TargetSpeed = optional_car_behind_target_lane->m_SpeedPrediction - slow_down_margin;
			m_CurrentManoeuver = Manoeuver::SLOW_DOWN_FOR_LANE_CHANGE;
			m_CarIdToSlowdownFor = optional_car_behind_target_lane->m_CarId;
			return create_trajectory_for_velocity_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
	}
	else if (optional_car_up_front_target_lane)
	{
		if (optional_car_up_front_target_lane->m_PositionPredictions[0].m_s > ego_car.m_s + safety_distance_front)
		{
			cout << "Could change lanes" << endl;
			m_CurrentManoeuver = Manoeuver::CHANGE_LANE;
			m_TargetSpeed = optional_car_up_front_target_lane->m_SpeedPrediction;
			const float time_to_chance_lane = 2; // secs
			m_TargetS = ego_car.m_s + (m_TargetSpeed + ego_car.m_v) / 2 * time_to_change_lane;
			return create_trajectory_for_lane_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s,
				time_to_change_lane);
		}
		else
		{
			cout << "Too tight to change lanes" << endl;
			cout << "Current s of ego car: " << ego_car.m_s << ", s of car up front: "
				<< optional_car_up_front_target_lane->m_PositionPredictions[0].m_s << endl;
			m_TargetLane = current_ego_lane;
			m_TargetSpeed = optional_car_up_front_target_lane->m_SpeedPrediction - slow_down_margin;
			m_CurrentManoeuver = Manoeuver::SLOW_DOWN_FOR_LANE_CHANGE;
			m_CarIdToSlowdownFor = optional_car_up_front_target_lane->m_CarId;
			return create_trajectory_for_velocity_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
	}
	else if (optional_car_behind_target_lane)
	{
		if (optional_car_behind_target_lane->m_PositionPredictions[0].m_s < ego_car.m_s - safety_distance_back)
		{
			cout << "Could change lanes" << endl;
			m_CurrentManoeuver = Manoeuver::CHANGE_LANE;
			m_TargetSpeed = min(m_IdealSpeed, (float)ego_car.m_s + 5);
			m_TargetS = ego_car.m_s + (m_TargetSpeed + ego_car.m_v) / 2 * time_to_change_lane;
			return create_trajectory_for_lane_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s,
				time_to_change_lane);
		}
		else
		{
			cout << "Too tight to change lanes" << endl;
			cout << "Current s of ego car: " << ego_car.m_s << ", s of car behind: "
				<< optional_car_behind_target_lane->m_PositionPredictions[0].m_s << endl;
			m_TargetLane = current_ego_lane;
			m_TargetSpeed = optional_car_behind_target_lane->m_SpeedPrediction - slow_down_margin;
			m_CurrentManoeuver = Manoeuver::SLOW_DOWN_FOR_LANE_CHANGE;
			m_CarIdToSlowdownFor = optional_car_behind_target_lane->m_CarId;
			return create_trajectory_for_velocity_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
	}
	else
	{
		cout << "Could change lanes" << endl;
		m_CurrentManoeuver = Manoeuver::CHANGE_LANE;
		m_TargetSpeed = min(m_IdealSpeed, (float)ego_car.m_s + 5);
		m_TargetS = ego_car.m_s + (m_TargetSpeed + ego_car.m_v) / 2 * time_to_change_lane;
		return create_trajectory_for_lane_change(
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s,
			time_to_change_lane);
	}
}

void Behaviour::print_state()
{
	cout << "Current state: ";

	switch (m_CurrentManoeuver)
	{
	case Manoeuver::FOLLOWING_LANE:
	{
		cout << "Following lane";
		break;
	}
	case Manoeuver::CHANGE_LANE:
	{
		cout << "Change lane";
		break;
	}
	case Manoeuver::SLOW_DOWN_FOR_LANE_CHANGE:
	{
		cout << "Slow down for lane change";
		break;
	}
	}
	cout << endl;
}

Behaviour::CarsPerLaneInfo Behaviour::determine_cars_up_front_and_behind_for_each_lane(
	const CarState& ego_car,
	const PredictionCalculator& prediction_calculator,
	unsigned int future_point)
{
	// Bin all sensed vehicles per lane
	vector<vector<CarPrediction>> vehicle_prediction_bins_per_lane = bin_vehicles_by_lane(
		prediction_calculator.get_car_predictions(),
		future_point,
		m_TotalLanes);

	vector<CarsInLane> cars_per_lane;
	float fastest_velocity_up_front = 0;
	unsigned int lane_with_fastest_velocity = 0;

	// Calculate properties of each car up front and behind in each lane
	for (unsigned int i = 0; i < m_TotalLanes; ++i)
	{
		const CarsInLane& cars_in_current_lane = vehicle_up_front_and_back(
			ego_car,
			vehicle_prediction_bins_per_lane[i],
			future_point);
		cars_per_lane.emplace_back(cars_in_current_lane);

		if (cars_in_current_lane.first)
		{
			if (cars_in_current_lane.first->m_SpeedPrediction > fastest_velocity_up_front)
			{
				fastest_velocity_up_front = cars_in_current_lane.first->m_SpeedPrediction;
				lane_with_fastest_velocity = i;
			}
		}
		else
		{
			fastest_velocity_up_front = m_IdealSpeed;
			lane_with_fastest_velocity = i;
		}
	}

	return { cars_per_lane, fastest_velocity_up_front, lane_with_fastest_velocity };
}

CalculatedTrajectory Behaviour::following_lane_state(
	const CarsPerLaneInfo& cars_per_lane_info,
	const CarState& ego_car,
	const CalculatedTrajectory& precalculated_trajectory,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s)
{
	const vector<CarsInLane>& cars_per_lane = get<0>(cars_per_lane_info);
	const float fastest_velocity_up_front = get<1>(cars_per_lane_info);
	const unsigned int lane_with_fastest_velocity = get<2>(cars_per_lane_info);

	const unsigned int current_ego_lane = convert_from_d_to_lane(ego_car.m_d);

	const CarsInLane& up_front_and_back = cars_per_lane[current_ego_lane];

	// v_u = velocity of the car upfront in current lane if any, else m_TargetSpeed
	float v_u = m_IdealSpeed;

	const optional<CarPrediction>& optional_car_up_front = up_front_and_back.first;
	double space_in_front_of_ego_car = max_distance_to_change_lane + 1;

	if (optional_car_up_front)
	{
		v_u = optional_car_up_front->m_SpeedPrediction;
		space_in_front_of_ego_car = optional_car_up_front->m_PositionPredictions[0].m_s - ego_car.m_s;
	}

	cout << "Speed of car up front in lane if any: " << v_u << endl;
	cout << "space_in_front_of_ego_car: " << space_in_front_of_ego_car << endl;
	cout << "fastest_velocity_up_front: " << fastest_velocity_up_front << endl;
	cout << "lane_with_fastest_velocity: " << lane_with_fastest_velocity << endl;
	cout << "Target speed : " << m_TargetSpeed << endl;

	// Safety first !
	if (space_in_front_of_ego_car < safety_distance_front)
	{
		cout << "Should decelerate" << endl;
		m_TargetSpeed = v_u - acceleration_margin;
		return create_trajectory_for_velocity_change(
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}

	// If there is a faster lane and the car up front is close enough so that we should consider a lane change
	if (v_u <= fastest_velocity_up_front - min_speed_diff_to_change_lane && space_in_front_of_ego_car < max_distance_to_change_lane)
	{
		return consider_lane_change(
			ego_car,
			cars_per_lane,
			fastest_velocity_up_front,
			lane_with_fastest_velocity,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}
	else // There is no faster lane
	{
		if (space_in_front_of_ego_car >= 30)
		{
			if (m_TargetSpeed < m_IdealSpeed - acceleration_margin)
			{
				cout << "Should acccelerate, car up front is still far away. Current target speed: " << m_TargetSpeed << endl;
				m_TargetSpeed = m_IdealSpeed;
				return create_trajectory_for_velocity_change(
					ego_car,
					precalculated_trajectory,
					map_waypoints_x,
					map_waypoints_y,
					map_waypoints_s);
			}
			else
			{
				cout << "Keep same velocity" << endl;
				return extend_trajectory(
					ego_car,
					precalculated_trajectory,
					map_waypoints_x,
					map_waypoints_y,
					map_waypoints_s);
			}
		}

		if (m_TargetSpeed > v_u + acceleration_margin)
		{
			cout << "Should decelerate" << endl;
			m_TargetSpeed = v_u - acceleration_margin;
			return create_trajectory_for_velocity_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
		else if (m_TargetSpeed < v_u - acceleration_margin)
		{
			cout << "Should accelerate" << endl;

			m_TargetSpeed = v_u;
			return create_trajectory_for_velocity_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
		else
		{
			cout << "Keep same velocity" << endl;
			return extend_trajectory(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
	}
}

CalculatedTrajectory Behaviour::change_lane_state(
	const CarsPerLaneInfo& cars_per_lane_info,
	const CarState& ego_car,
	const CalculatedTrajectory& precalculated_trajectory,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s)
{
	const vector<CarsInLane>& cars_per_lane = get<0>(cars_per_lane_info);
	const float fastest_velocity_up_front = get<1>(cars_per_lane_info);
	const unsigned int lane_with_fastest_velocity = get<2>(cars_per_lane_info);

	const CarsInLane& up_front_and_back_in_target_lane = cars_per_lane[m_TargetLane];
	const optional<CarPrediction>& optional_car_up_front_target_lane = up_front_and_back_in_target_lane.first;

	// If velocity of car up front changed, recalculate trajectory
	if (optional_car_up_front_target_lane && optional_car_up_front_target_lane->m_SpeedPrediction < m_TargetSpeed - acceleration_margin)
	{
		cout << "Velocity of car up front changed" << endl;
		m_TargetSpeed = optional_car_up_front_target_lane->m_SpeedPrediction;
		const float time_to_chance_lane = 2.f; // secs
		const float dt = max(0.5, abs(ego_car.m_d - convert_from_middle_of_lane_to_d(m_TargetLane)) / lane_size * time_to_chance_lane);
		m_TargetS = ego_car.m_s + (m_TargetSpeed + ego_car.m_v) / 2 * dt;

		return create_trajectory_for_lane_change(
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s,
			dt);
	}
	else if (ego_car.m_s > m_TargetS)
	{
		cout << "Done manoeuver" << endl;
		m_CurrentManoeuver = Manoeuver::FOLLOWING_LANE;
		return extend_trajectory(
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}
	else
	{
		cout << "Keep on changing lanes" << endl;
		return extend_trajectory(
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}
}

CalculatedTrajectory Behaviour::slow_down_for_lane_change_state(
	const CarsPerLaneInfo& cars_per_lane_info,
	const CarState& ego_car,
	const PredictionCalculator& prediction_calculator,
	const CalculatedTrajectory& precalculated_trajectory,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s)
{
	const vector<CarsInLane>& cars_per_lane = get<0>(cars_per_lane_info);
	const float fastest_velocity_up_front = get<1>(cars_per_lane_info);
	const unsigned int lane_with_fastest_velocity = get<2>(cars_per_lane_info);

	const unsigned int current_ego_lane = convert_from_d_to_lane(ego_car.m_d);

	const CarsInLane& up_front_and_back = cars_per_lane[current_ego_lane];

	// v_u = velocity of the car upfront in current lane if any, else m_TargetSpeed
	float v_u = m_IdealSpeed;

	const optional<CarPrediction>& optional_car_up_front = up_front_and_back.first;

	// We look at the car up front for a longer distance, because we are slowing down due the slowness of this car
	if (optional_car_up_front && optional_car_up_front->m_PositionPredictions[0].m_s)
	{
		v_u = optional_car_up_front->m_SpeedPrediction;
	}

	cout << "Speed of car up front in lane if any: " << v_u << endl;

	// If there is still a faster lane
	if (v_u <= fastest_velocity_up_front - min_speed_diff_to_change_lane)
	{
		for (const CarPrediction& car_prediction : prediction_calculator.get_car_predictions())
		{
			if (car_prediction.m_CarId == m_CarIdToSlowdownFor)
			{
				if (ego_car.m_s < car_prediction.m_PositionPredictions[0].m_s - safety_distance_front)
				{
					cout << "Safety distance behind car to slow down for, back to following lane state" << endl;

					return consider_lane_change(
						ego_car,
						cars_per_lane,
						fastest_velocity_up_front,
						lane_with_fastest_velocity,
						precalculated_trajectory,
						map_waypoints_x,
						map_waypoints_y,
						map_waypoints_s);
				}
				else
				{
					cout << "ego s: " << ego_car.m_s << ", slow down car s: " << car_prediction.m_PositionPredictions[0].m_s << endl;
					cout << "Continue at slower speed" << endl;
					return extend_trajectory(
						ego_car,
						precalculated_trajectory,
						map_waypoints_x,
						map_waypoints_y,
						map_waypoints_s);
				}
			}
		}

		// Car to slow down for not found: retry in FOLLOWING_LANE state
		cout << "Car to slow down for not found: retry in FOLLOWING_LANE state" << endl;
		m_CurrentManoeuver = Manoeuver::FOLLOWING_LANE;

		return precalculated_trajectory;
	}
	else
	{
		cout << "No faster lane anymore, back to FOLLOWING_LANE state" << endl;
		m_CurrentManoeuver = Manoeuver::FOLLOWING_LANE;
		return precalculated_trajectory;

		if (m_TargetSpeed < v_u - acceleration_margin)
		{
			cout << "No faster lane anymore, should regain speed again" << endl;
			m_CurrentManoeuver = Manoeuver::FOLLOWING_LANE;
			m_TargetSpeed = v_u;
			return create_trajectory_for_velocity_change(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
		else
		{
			cout << "No faster lane anymore, continue at same speed" << endl;
			m_CurrentManoeuver = Manoeuver::FOLLOWING_LANE;
			return extend_trajectory(
				ego_car,
				precalculated_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);
		}
	}
}

CalculatedTrajectory Behaviour::calculate_new_behaviour(
	const CarState& ego_car,
	const PredictionCalculator& prediction_calculator,
	const CalculatedTrajectory& precalculated_trajectory,
	const vector<double>& map_waypoints_x,
	const vector<double>& map_waypoints_y,
	const vector<double>& map_waypoints_s)
{
	print_state();

	const unsigned int previous_size = precalculated_trajectory.m_path_x.size();
	const unsigned int future_point = min(previous_size, prediction_calculator.get_horizon()) - 1;
	cout << "Future point for prediction: " << future_point << endl;
	const CarsPerLaneInfo& cars_per_lane_info =
		determine_cars_up_front_and_behind_for_each_lane(
			ego_car,
			prediction_calculator,
			future_point);
	const vector<CarsInLane>& cars_per_lane = get<0>(cars_per_lane_info);
	const float fastest_velocity_up_front = get<1>(cars_per_lane_info);
	const unsigned int lane_with_fastest_velocity = get<2>(cars_per_lane_info);

	switch (m_CurrentManoeuver)
	{
	case Manoeuver::FOLLOWING_LANE:
	{
		return following_lane_state(
			cars_per_lane_info,
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}
	case Manoeuver::CHANGE_LANE:
	{
		return change_lane_state(
			cars_per_lane_info,
			ego_car,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}
	case Manoeuver::SLOW_DOWN_FOR_LANE_CHANGE:
	{
		return slow_down_for_lane_change_state(
			cars_per_lane_info,
			ego_car,
			prediction_calculator,
			precalculated_trajectory,
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);
	}
	}
}

CalculatedTrajectory Behaviour::create_trajectory_for_lane_change(
	const CarState& ego_car,
	const CalculatedTrajectory& precalculated_trajectory,
	const vector<double>& map_waypoints_x,
	const vector<double>& map_waypoints_y,
	const vector<double>& map_waypoints_s,
	const double trajectory_time)
{
	cout << "Trajectory time: " << trajectory_time << endl;

	const unsigned int points_to_reuse = min((unsigned int)precalculated_trajectory.m_path_x.size(), min_points_to_predict);

	vector<double> next_x_vals;
	vector<double> next_y_vals;

	// Already calculated path points
	for (unsigned int i = 0; i < points_to_reuse; ++i)
	{
		next_x_vals.push_back(precalculated_trajectory.m_path_x[i]);
		next_y_vals.push_back(precalculated_trajectory.m_path_y[i]);
	}

	const double x_i = precalculated_trajectory.m_path_x[points_to_reuse - 1];
	const double y_i = precalculated_trajectory.m_path_y[points_to_reuse - 1];
	
	double heading = 0;

	if (points_to_reuse > 1)
	{
		const double prev_x_i = precalculated_trajectory.m_path_x[points_to_reuse - 2];
		const double prev_y_i = precalculated_trajectory.m_path_y[points_to_reuse - 2];

		heading = atan2((y_i - prev_y_i), (x_i - prev_x_i));
	}

	cout << "Heading: " << heading << endl;

	cout << "Original x,y: " << x_i << " " << y_i << endl;
	const vector<double>& sd_i = getFrenet(x_i, y_i, heading, map_waypoints_x, map_waypoints_y);
	const double s_i = sd_i[0];
	const double d_i = sd_i[1];
	cout << "Start Frenet: " << s_i << " " << d_i << endl;
	//cout << "Recalculated x,y from Frenet: " << xy[0] << " " << xy[1] << endl;
	cout << "Ego car velocity: " << ego_car.m_v << endl;
	

	double v_x = 0;
	double v_y = 0;
	double v = 0;
	double a_x = 0;
	double a_y = 0;

	// Calculate start parameters for JMT at the end of the part of the trajectory to reuse
	if (points_to_reuse > 3)
	{
		const double prev_x_i = precalculated_trajectory.m_path_x[points_to_reuse - 2];
		const double prev_y_i = precalculated_trajectory.m_path_y[points_to_reuse - 2];

		v_x = (x_i - prev_x_i) / time_interval;
		v_y = (y_i - prev_y_i) / time_interval;
		v = sqrt(v_x * v_x + v_y * v_y);

		const double prev2_x_i = precalculated_trajectory.m_path_x[points_to_reuse - 3];
		const double prev2_y_i = precalculated_trajectory.m_path_y[points_to_reuse - 3];

		const double v_x_prev = (prev_x_i - prev2_x_i) / time_interval;
		const double v_y_prev = (prev_y_i - prev2_y_i) / time_interval;

		a_x = (v_x - v_x_prev) / time_interval;
		a_y = (v_y - v_y_prev) / time_interval;

		double prev_heading = atan2((prev_y_i - prev2_y_i), (prev_x_i - prev2_x_i));
		cout << "prev_heading: " << prev_heading << endl;
	}

	// Calculate end speed of JMT
	const float min_conf_speed = v - (trajectory_time / time_interval) * m_NormalAcceleration;
	const float max_conf_speed = v + (trajectory_time / time_interval) * m_NormalAcceleration;
	const double v_f = max(min_conf_speed, min(m_TargetSpeed, max_conf_speed));
	cout << "Target final speed: " << m_TargetSpeed << ", final speed adapted to max. acc/deceleration: " << v_f << endl;

	// Calculate end locations of JMT in Frenet coordinate system 
	const double ds_f = max(min_conf_speed, min(m_TargetSpeed, max_conf_speed));
	cout << "Target ds_f: " << ds_f << endl;

	double s_f = s_i + trajectory_time * (ds_f + v) / 2.f;
	const double d_f = convert_from_middle_of_lane_to_d(m_TargetLane);

	// Max distance to cover in JMT, otherwise the JMT might exceed speed limits somewhere in the middle
	double max_dist = trajectory_time * (v + v_f) / 2;

	// Now correct for any discontinuities / miscalculations in (s,d) coordinate systems
	const vector<double>& point_f = getXY(s_f, d_f, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	cout << "Initial destination point: " << point_f[0] << " " << point_f[1] << endl;

	double dist = sqrt(pow(point_f[1] - y_i, 2) + pow(point_f[0] - x_i, 2));

	while (dist < max_dist - 0.5)
	{
		s_f += 1;
		cout << "Increasing s_f to get to max. dist; max_dist: " << max_dist << ", dist: " << dist << ", s_f: " << s_f << endl;
		const vector<double>& new_point_f = getXY(s_f, d_f, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		cout << "New destination point: " << new_point_f[0] << " " << new_point_f[1] << endl;
		dist = sqrt(pow(new_point_f[1] - y_i, 2) + pow(new_point_f[0] - x_i, 2));
	}
	while (dist > max_dist + 0.5)
	{
		s_f -= 1;
		cout << "Decreasing s_f to get to max. dist; max_dist: " << max_dist << ", dist: " << dist << ", s_f: " << s_f << endl;
		const vector<double>& new_point_f = getXY(s_f, d_f, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		cout << "New destination point: " << new_point_f[0] << " " << new_point_f[1] << endl;
		dist = sqrt(pow(new_point_f[1] - y_i, 2) + pow(new_point_f[0] - x_i, 2));
	}

	// We also need the heading at the end point for good estimates of v_x, v_y 
	const double s_f_next = s_f + time_interval * ds_f;

	const vector<double>& final_point_f = getXY(s_f, d_f, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	const vector<double>& next_point_f = getXY(s_f_next, d_f, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	const double heading_f = atan2(next_point_f[1] - final_point_f[1], next_point_f[0] - final_point_f[0]);
	cout << "Final heading: " << heading_f << endl;

	cout << "Target lane: " << m_TargetLane << endl;
	cout << "d_f: " << d_f << endl;

	// Calculate the v_x, v_y components from v_f and the calculated heading
	const double v_f_x = v_f * cos(heading_f);
	const double v_f_y = v_f * sin(heading_f);

	const vector<double> start_x = { x_i, v_x, a_x };
	const vector<double> end_x = { final_point_f[0], v_f_x, 0 };

	const vector<double> start_y = { y_i, v_y, a_y};
	const vector<double> end_y = { final_point_f[1], v_f_y, 0 };

	const vector<double>& x_traj = JMT(start_x, end_x, trajectory_time);
	const vector<double>& y_traj = JMT(start_y, end_y, trajectory_time);

	const unsigned int extra_points = trajectory_time / time_interval;
	cout << "Extra points: " << extra_points << endl;

	float t = 0;

	for (unsigned int i = 0; i < extra_points; ++i)
	{
		t += time_interval;

		double x = poly_eval(x_traj, t);
		double y = poly_eval(y_traj, t);

		next_x_vals.push_back(x);
		next_y_vals.push_back(y);
	}

	cout << "Calculated trajectory for lane change: " << endl;
	for (unsigned int i = 0; i < next_x_vals.size(); ++i)
	{
		if (i == 0)
		{
			cout
				<< next_x_vals[i] << " " << next_y_vals[i] << " "
				<< endl;
		}
		else
		{
			const double v_x = (next_x_vals[i] - next_x_vals[i - 1]) / time_interval;
			const double v_y = (next_y_vals[i] - next_y_vals[i - 1]) / time_interval;
			const double v = sqrt(pow(v_x, 2) + pow(v_y, 2));
			cout
				<< next_x_vals[i] << " " << next_y_vals[i] << " "
				<< v_x << " " << v_y << " " << v
				<< endl;
		}
	}

	return { next_x_vals, next_y_vals };
}

CalculatedTrajectory Behaviour::create_trajectory_for_velocity_change(
	const CarState& ego_car,
	const CalculatedTrajectory& precalculated_trajectory,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s)
{
	const unsigned int points_to_reuse = min((unsigned int)precalculated_trajectory.m_path_x.size(), min_points_to_predict);

	vector<double> next_x_vals;
	vector<double> next_y_vals;
	vector<double> deltas;

	// Already calculated path points
	for (unsigned int i = 0; i < points_to_reuse; ++i)
	{
		next_x_vals.push_back(precalculated_trajectory.m_path_x[i]);
		next_y_vals.push_back(precalculated_trajectory.m_path_y[i]);
	}

	const double x_i = precalculated_trajectory.m_path_x[points_to_reuse - 1];
	const double y_i = precalculated_trajectory.m_path_y[points_to_reuse - 1];

	float velocity = ego_car.m_v;

	double heading = 0;

	if (points_to_reuse > 1)
	{
		const double prev_x_i = precalculated_trajectory.m_path_x[points_to_reuse - 2];
		const double prev_y_i = precalculated_trajectory.m_path_y[points_to_reuse - 2];

		heading = atan2((y_i - prev_y_i), (x_i - prev_x_i));

		velocity = sqrt((x_i - prev_x_i) * (x_i - prev_x_i) + (y_i - prev_y_i) * (y_i - prev_y_i)) / time_interval;
	}

	const vector<double> last_point = getFrenet(x_i, y_i, heading, map_waypoints_x, map_waypoints_y);
	const double last_point_s = last_point[0];
	const double last_point_d = last_point[1];

	cout << "Last point: " << last_point_s << " " << last_point_d << endl;
	cout << ego_car.m_s << " " << ego_car.m_d << endl;

	double s_i = last_point_s;

	cout << "Target speed: " << m_TargetSpeed << endl;
	while (velocity != m_TargetSpeed)
	{
		if (velocity < m_TargetSpeed)
		{
			velocity += m_NormalAcceleration;
			velocity = min(velocity, m_TargetSpeed);
		}
		else
		{
			velocity -= m_NormalAcceleration;
			velocity = max(velocity, m_TargetSpeed);
		}

		const float delta = velocity * time_interval;
		deltas.push_back(delta);
		s_i += delta;
	}

	cout << "# predicted points for velocity change " << next_x_vals.size() << endl;

	for (unsigned int i = deltas.size(); i < prediction_horizon; ++i)
	{
		const float delta = velocity * time_interval;
		deltas.push_back(delta);
		s_i += delta;
	}

	cout << "Current ego car location: " << ego_car.m_x << " " << ego_car.m_y << endl;

	const CalculatedTrajectory& smoothened_path = extend_trajectory_with_lane_spline(
		ego_car, 
		CalculatedTrajectory{ next_x_vals, next_y_vals }, 
		deltas,
		points_to_reuse,
		map_waypoints_x,
		map_waypoints_y,
		map_waypoints_s);

	for (unsigned int i = 0; i < smoothened_path.m_path_x.size(); ++i)
	{
		next_x_vals.push_back(smoothened_path.m_path_x[i]);
		next_y_vals.push_back(smoothened_path.m_path_y[i]);
	}

	cout << "create_trajectory_for_velocity_change Trajectory: " << endl;
	for (unsigned int i = 0; i < next_x_vals.size(); ++i)
	{
		cout 
			<< next_x_vals[i] << " " << next_y_vals[i] << " " 
			<< endl;
	}

	return { next_x_vals, next_y_vals };
}

CalculatedTrajectory Behaviour::extend_trajectory(
	const CarState& ego_car,
	const CalculatedTrajectory& precalculated_trajectory,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s)
{
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	vector<double> deltas;

	// Already calculated path points
	for (unsigned int i = 0; i < precalculated_trajectory.m_path_x.size(); ++i)
	{
		next_x_vals.push_back(precalculated_trajectory.m_path_x[i]);
		next_y_vals.push_back(precalculated_trajectory.m_path_y[i]);
	}

	const double x_i = precalculated_trajectory.m_path_x[precalculated_trajectory.m_path_x.size() - 1];
	const double y_i = precalculated_trajectory.m_path_y[precalculated_trajectory.m_path_x.size() - 1];

	float velocity = ego_car.m_v;

	double heading = 0;

	if (precalculated_trajectory.m_path_x.size() > 1)
	{
		const double prev_x_i = precalculated_trajectory.m_path_x[precalculated_trajectory.m_path_x.size() - 2];
		const double prev_y_i = precalculated_trajectory.m_path_y[precalculated_trajectory.m_path_x.size() - 2];

		heading = atan2((y_i - prev_y_i), (x_i - prev_x_i));

		velocity = sqrt((x_i - prev_x_i) * (x_i - prev_x_i) + (y_i - prev_y_i) * (y_i - prev_y_i)) / time_interval;
	}

	cout << "Velocity at end of calculated trajectory: " << velocity << endl;

	const vector<double> last_point = getFrenet(x_i, y_i, heading, map_waypoints_x, map_waypoints_y);
	cout << "Last point: " << x_i << " " << y_i << endl;
	const double last_point_s = last_point[0];
	const double last_point_d = last_point[1];
	const vector<double>& xy_check = getXY(last_point_s, last_point_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	cout << "Last point: " << xy_check[0] << " " << xy_check[1] << endl;

	double s_i = last_point_s;
	for (unsigned int i = precalculated_trajectory.m_path_x.size(); i < prediction_horizon; ++i)
	{
		const float delta = m_TargetSpeed * time_interval;
		deltas.push_back(delta);
		cout << "Delta: " << delta << endl;
		s_i += velocity * time_interval;
	}

	if (deltas.size() > 2)
	{
		const CalculatedTrajectory& smoothened_path = extend_trajectory_with_lane_spline(
			ego_car,
			CalculatedTrajectory{ next_x_vals, next_y_vals },
			deltas,
			precalculated_trajectory.m_path_x.size(),
			map_waypoints_x,
			map_waypoints_y,
			map_waypoints_s);

		for (unsigned int i = 0; i < smoothened_path.m_path_x.size(); ++i)
		{
			next_x_vals.push_back(smoothened_path.m_path_x[i]);
			next_y_vals.push_back(smoothened_path.m_path_y[i]);
		}
	}

	cout << "Current ego car location: " << ego_car.m_x << " " << ego_car.m_y << endl;

	cout << "extend_trajectory Trajectory: " << endl;
	for (unsigned int i = 0; i < next_x_vals.size(); ++i)
	{
		cout << next_x_vals[i] << " " << next_y_vals[i] << endl;
	}

	return { next_x_vals, next_y_vals };
}

CalculatedTrajectory Behaviour::extend_trajectory_with_lane_spline(
	const CarState& ego_car,
	const CalculatedTrajectory& calculated_trajectory,
	const vector<double>& deltas,
	const unsigned int points_to_reuse,
	const std::vector<double>& map_waypoints_x,
	const std::vector<double>& map_waypoints_y,
	const std::vector<double>& map_waypoints_s)
{
	vector<double> pts_x;
	vector<double> pts_y;

	double ref_x = ego_car.m_x;
	double ref_y = ego_car.m_y;
	double ref_yaw = ego_car.m_yaw;

	if (points_to_reuse < 2)
	{
		double prev_car_x = ego_car.m_x - cos(ref_yaw);
		double prev_car_y = ego_car.m_y - sin(ref_yaw);

		pts_x.push_back(prev_car_x);
		pts_x.push_back(ego_car.m_x);

		pts_y.push_back(prev_car_y);
		pts_y.push_back(ego_car.m_y);
	}
	else
	{
		ref_x = calculated_trajectory.m_path_x[points_to_reuse - 1];
		ref_y = calculated_trajectory.m_path_y[points_to_reuse - 1];

		const double prev_car_x = calculated_trajectory.m_path_x[points_to_reuse - 2];
		const double prev_car_y = calculated_trajectory.m_path_y[points_to_reuse - 2];
		ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);

		pts_x.push_back(prev_car_x);
		pts_x.push_back(ref_x);
		pts_y.push_back(prev_car_y);
		pts_y.push_back(ref_y);
	}

	return calculate_spline_trajectory_in_lane(
		pts_x, 
		pts_y, 
		m_TargetLane, 
		deltas, 
		map_waypoints_x, 
		map_waypoints_y, 
		map_waypoints_s);
}