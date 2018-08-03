#include <stdexcept>

#include "predict.h"
#include "conversion.h"

using namespace std;

PredictionCalculator::PredictionCalculator(
	const vector<CarState>& car_states, 
	const CarState& ego_car, 
	unsigned int horizon, 
	double time_delta, 
	double max_distance_to_predict):
	m_Horizon(horizon)
{
	for (unsigned int i = 0; i < car_states.size(); ++i)
	{
		const CarState& other_car = car_states[i];

		if (ego_car.m_s - max_distance_to_predict <= other_car.m_s &&
			ego_car.m_s + max_distance_to_predict >= other_car.m_s)
		{
			const double s = other_car.m_s;
			const double d = other_car.m_d;	// cars stay in the same lane in the simulation
			const double v = other_car.m_v;

			vector<FrenetCoordinate> position_predictions;

			for (unsigned int j = 0; j < horizon; ++j) 
			{
				position_predictions.emplace_back(s + time_delta * j * v, d);
			}

			m_CarPredictions.emplace_back(position_predictions, v, other_car.m_id);
		}
	}
}

unsigned int PredictionCalculator::get_predicted_cars() const
{
	return m_CarPredictions.size();
}

std::vector<CarPrediction> PredictionCalculator::get_car_predictions() const
{
	return m_CarPredictions;
}

FrenetCoordinate PredictionCalculator::get_frenet_coord_of_car(unsigned int car_index, unsigned int point_in_time) const
{
	if (car_index >= m_CarPredictions.size() || point_in_time >= m_CarPredictions[0].m_PositionPredictions.size())
	{
		throw invalid_argument("Wrong arguments to get_s_of_car");
	}

	return m_CarPredictions[car_index].m_PositionPredictions[point_in_time];
}

unsigned int PredictionCalculator::get_horizon() const
{
	return m_Horizon;
}