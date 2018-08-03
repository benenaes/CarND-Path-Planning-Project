#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include "car_state.h"
#include "predict.h"
#include "trajectory_generation.h"

#include <vector>
#include <utility>
#include <optional>
#include <tuple>

class Behaviour
{
public:
	using CarsInLane = std::pair<std::optional<CarPrediction>, std::optional<CarPrediction>>;
	using CarsPerLaneInfo = std::tuple<std::vector<Behaviour::CarsInLane>, float, unsigned int>;

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

	/*
	Calculate a new or extended trajectory based on 
	the current behaviour state, the current ego car and predictions from other cars
	*/
	CalculatedTrajectory calculate_new_behaviour(
		const CarState& ego_car, 
		const PredictionCalculator& prediction_calculator,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

private:
	/*
	Make decisions for the behaviour of the ego car when in the FOLLOWING_LANE state
	*/
	CalculatedTrajectory following_lane_state(
		const CarsPerLaneInfo& cars_per_lane_info,
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Make decisions for the behaviour of the ego car when in the CHANGE_LANE state
	*/
	CalculatedTrajectory change_lane_state(
		const CarsPerLaneInfo& cars_per_lane_info,
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Make decisions for the behaviour of the ego car when in the SLOW_DOWN_FOR_LANE_CHANGE state
	*/
	CalculatedTrajectory slow_down_for_lane_change_state(
		const CarsPerLaneInfo& cars_per_lane_info,
		const CarState& ego_car,
		const PredictionCalculator& prediction_calculator,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Create a trajectory for a lange change with some points of the precalculated trajectory prepended for smooth driving
	(JMT based)
	*/
	CalculatedTrajectory create_trajectory_for_lane_change(
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s,
		const double trajectory_time);

	/*
	Create a trajectory for a velocity change with some points of the precalculated trajectory prepended for smooth driving
	(spline based)
	*/
	CalculatedTrajectory create_trajectory_for_velocity_change(
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Extend an existing trajectory keeping up the velocity and the lane at the end of the precalculated trajectory
	(spline based)
	*/
	CalculatedTrajectory extend_trajectory(
		const CarState& ego_car,
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Determine if a lane change is possible and if so calculate a trajectory (JMT based)
	*/
	CalculatedTrajectory consider_lane_change(
		const CarState& ego_car,
		const std::vector<CarsInLane>& cars_per_lane,
		float fastest_velocity_up_front,	// fastest velocity of a car up front in any lane
		float lane_with_fastest_velocity,	// the lane where this car is driving
		const CalculatedTrajectory& precalculated_trajectory,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Prepare the calculation of a cubic spline by choosing points from a precalculated trajectory
	*/
	CalculatedTrajectory extend_trajectory_with_lane_spline(
		const CarState& ego_car,
		const CalculatedTrajectory& calculated_trajectory,
		const std::vector<double>& deltas,	// deltas to apply (can be different values in case of acc/decelerations)
		const unsigned int points_to_reuse,
		const std::vector<double>& map_waypoints_x,
		const std::vector<double>& map_waypoints_y,
		const std::vector<double>& map_waypoints_s);

	/*
	Print the current state the behaviour mechanism is in
	*/
	void print_state();

	/*
	Calculate which vehicle are up front and behind the ego car for the lane of the ego car,
	given car state predictions and a given point in time
	*/
	CarsInLane vehicle_up_front_and_back(
		const CarState& ego_car,
		const std::vector<CarPrediction>& cars_of_particular_lane,
		unsigned int point_in_time);

	/*
	Bin all car predictions per lane at a given point time
	*/
	std::vector<std::vector<CarPrediction>> bin_vehicles_by_lane(
		const std::vector<CarPrediction>& close_by_car_predictions,
		unsigned int point_in_time,
		unsigned int total_lanes);

	/*
	Calculate which vehicle are up front and behind the ego car for all lanes,
	given car state predictions and a given point in time
	*/
	CarsPerLaneInfo determine_cars_up_front_and_behind_for_each_lane(
		const CarState& ego_car,
		const PredictionCalculator& prediction_calculator,
		unsigned int future_point);

	Manoeuver m_CurrentManoeuver;		// Current state of the behaviour mechanism
	const float m_NormalAcceleration;	// meter per second²
	const float m_IdealSpeed;	// meter per second
	unsigned int m_TargetLane;	// Target lane
	float m_TargetSpeed; // meter per second
	float m_TargetS;	// target s for the end of a lane change
	int m_CarIdToSlowdownFor;	// ID of the car to slow down for if in the SLOW_DOWN_FOR_LANE_CHANGE state
	const unsigned int m_TotalLanes; // number of lanes
};

#endif