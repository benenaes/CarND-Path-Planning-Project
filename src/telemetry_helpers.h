#ifndef TELEMETRY_HELPERS
#define TELEMETRY_HELPERS

#include <string>
#include <vector>

#include "json.hpp"

#include "car_state.h"
#include "behaviour.h"

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string& s);

CarState ego_car_state_from_telemetry(const nlohmann::basic_json<>& json);
std::vector<CarState> car_states_from_sensor_fusion(const std::vector<std::vector<double>>& sensor_fusion);

PrecalculatedTrajectory precalculated_trajectory_from_telemetry(const nlohmann::basic_json<>& json);

#endif