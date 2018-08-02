#include "conversion.h"

#include "telemetry_helpers.h"

using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string& s) 
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

CarState ego_car_state_from_telemetry(const nlohmann::basic_json<>& j)
{
	const double car_yaw = j[1]["yaw"];
	const double ref_yaw = deg2rad(car_yaw);
	const double car_speed = mph_to_mps(j[1]["speed"]);
	const double v_x = car_speed * cos(car_yaw);
	const double v_y = car_speed * sin(car_yaw);
	return CarState(-1, j[1]["x"], j[1]["y"], v_x, v_y, j[1]["s"], j[1]["d"]);
}

vector<CarState> car_states_from_sensor_fusion(const std::vector<std::vector<double>>& sensor_fusion)
{
	vector<CarState> car_states;
	for (unsigned int i = 0; i < sensor_fusion.size(); ++i)
	{
		const double d = sensor_fusion[i][6];
		const double v_x = sensor_fusion[i][3];
		const double v_y = sensor_fusion[i][4];
		const double s = sensor_fusion[i][5];
		const double x = sensor_fusion[i][1];
		const double y = sensor_fusion[i][2];
		const int id = sensor_fusion[i][0];
		car_states.emplace_back(id, x, y, v_x, v_y, s, d);
	}

	return car_states;
}

CalculatedTrajectory precalculated_trajectory_from_telemetry(const nlohmann::basic_json<>& j)
{
	CalculatedTrajectory trajectory = {j[1]["previous_path_x"], j[1]["previous_path_y"]};

	return trajectory;
}