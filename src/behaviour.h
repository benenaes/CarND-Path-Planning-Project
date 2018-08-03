#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include "car_state.h"
#include "predict.h"

#include <vector>
#include <utility>
#include <optional>

struct CalculatedTrajectory
{
	std::vector<double> m_previous_path_x;
	std::vector<double> m_previous_path_y;
};

class Behaviour
{
public:
	using CarsInLane = std::pair<std::optional<CarPrediction>, std::optional<CarPrediction>>;

	enum class Manoeuver
	{
		FOLLOWING_LANE,
		CHANGE_LANE,
		SLOW_DOWN_FOR_LANE_CHANGE
	};

	Behaviour(
		const unsigned int total_lanes, 
		const float normal_acceleration,
		const float ideal_speed,
		const unsigned int current_lane);

	CalculatedTrajectory calculate_new_behaviour(
		const CarState& ego_car, 
		const PredictionCalculator& prediction_calculator,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

private:
	CalculatedTrajectory create_trajectory_for_lane_change(
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s,
		const double trajectory_time);

	CalculatedTrajectory create_trajectory_for_velocity_change(
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	CalculatedTrajectory extend_trajectory(
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	CalculatedTrajectory consider_lane_change(
		const CarState& ego_car,
		const std::vector<CarsInLane>& cars_per_lane,
		float fastest_velocity_up_front,
		float lane_with_fastest_velocity,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	CalculatedTrajectory apply_spline(
		const CarState& ego_car,
		const CalculatedTrajectory& calculated_trajectory,
		const std::vector<double>& deltas,
		const unsigned int points_to_reuse,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	Manoeuver m_CurrentManoeuver;
	const float m_NormalAcceleration;	// meter per second²
	const float m_IdealSpeed;	// meter per second
	unsigned int m_TargetLane;
	float m_TargetSpeed; // meter per second
	float m_TargetS;
	int m_CarIdToSlowdownFor;
	const unsigned int m_TotalLanes;
};

#endif