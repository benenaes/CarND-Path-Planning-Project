#ifndef PREDICT_HPP
#define PREDICT_HPP

#include <vector>

#include "car_state.h"

struct FrenetCoordinate
{
	float m_s;
	float m_d;

	FrenetCoordinate(float s, float d): m_s(s), m_d(d) {}
};

struct CarPrediction
{
	std::vector<FrenetCoordinate> m_PositionPredictions;
	float m_SpeedPrediction;

	CarPrediction(const std::vector<FrenetCoordinate>& position_predictions, float speed_prediction) :
		m_PositionPredictions(position_predictions), m_SpeedPrediction(speed_prediction) {}
};

class PredictionCalculator
{
public:
	PredictionCalculator(
		const std::vector<CarState>& car_states, const CarState& ego_car, unsigned int horizon, double time_delta, double max_distance_to_predict);

	unsigned int get_predicted_cars() const;

	std::vector<CarPrediction> get_car_predictions() const;

	FrenetCoordinate get_frenet_coord_of_car(unsigned int car_index, unsigned int point_in_time) const;

private:
	std::vector<CarPrediction> m_CarPredictions;
};

#endif
