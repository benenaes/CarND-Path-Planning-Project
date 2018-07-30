#ifndef BEHAVIOUR_HPP
#define BEHAVIOUR_HPP

#include "car_state.h"
#include "predict.h"

struct PrecalculatedTrajectory
{
	const std::vector<double> m_previous_path_x;
	const std::vector<double> m_previous_path_y;

	double m_end_path_s;
	double m_end_path_d;
};

class Behaviour
{
public:
	enum class Manoeuver
	{
		FOLLOWING_LANE,
		CHANGE_LEFT,
		CHANGE_RIGHT
	};

	Behaviour(
		unsigned int total_lanes, 
		float normal_acceleration,
		float target_speed);

	void calculate_new_behaviour(
		const CarState& ego_car, 
		const PredictionCalculator& prediction_calculator,
		const PrecalculatedTrajectory& precalculated_trajectory);

private:
	Manoeuver m_CurrentManoeuver;
	const float m_NormalAcceleration;
	const float m_TargetSpeed;
	unsigned int m_TargetLane;
	float m_TargetS;
	const unsigned int m_TotalLanes;
};

#endif