#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "car_state.h"
#include "predict.h"
#include "conversion.h"
#include "telemetry_helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

void load_map_waypoints(
	vector<double>& map_waypoints_x,
	vector<double>& map_waypoints_y,
	vector<double>& map_waypoints_s,
	vector<double>& map_waypoints_dx,
	vector<double>& map_waypoints_dy)
{
	// Waypoint map to read from
	const string map_file_ = "../data/highway_map.csv";
	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}
}

int main() 
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  load_map_waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

  std::chrono::time_point<std::chrono::system_clock> prev_behaviour_time;

  //prev_behaviour_time = std::chrono::system_clock::now() - prev_behaviour_time;

  double ref_vel = 0.0;
  unsigned int lane = 1;

  Behaviour behaviour(
	  3,		// total lanes
	  mph_to_mps(.224),		// normal acceleration
	  mph_to_mps(48), // ideal speed
	  1);		// current lane	

  h.onMessage([
		&map_waypoints_x,
		&map_waypoints_y,
		&map_waypoints_s,
		&map_waypoints_dx,
		&map_waypoints_dy,
		&ref_vel,
		&lane,
		&behaviour]
		  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
			const CarState ego_car_state = ego_car_state_from_telemetry(j);

          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

			CalculatedTrajectory previous_trajectory = precalculated_trajectory_from_telemetry(j);

			if (previous_trajectory.m_path_x.size() == 0)
			{
				previous_trajectory.m_path_x.push_back(ego_car_state.m_x);
				previous_trajectory.m_path_y.push_back(ego_car_state.m_y);
			}

			//cout << "Precalculated trajectory: " << endl;

			// Already calculated path points
			for (unsigned int i = 0; i < previous_trajectory.m_path_x.size(); ++i)
			{
				cout << previous_trajectory.m_path_x[i] << " " << previous_trajectory.m_path_y[i] << endl;
			}

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
			const vector<CarState> car_states = car_states_from_sensor_fusion(j[1]["sensor_fusion"]);

			const double time_interval = 0.02; // 20 ms
			const unsigned int max_distance_for_prediction = 80; // 80 m
			const unsigned int prediction_horizon = 50; // timesteps = 1 sec
			PredictionCalculator prediction_calculator(car_states, ego_car_state, prediction_horizon, time_interval, max_distance_for_prediction);

			const CalculatedTrajectory& calculated_trajectory = behaviour.calculate_new_behaviour(
				ego_car_state, 
				prediction_calculator, 
				previous_trajectory,
				map_waypoints_x,
				map_waypoints_y,
				map_waypoints_s);

			json msgJson;

          	msgJson["next_x"] = calculated_trajectory.m_path_x;
          	msgJson["next_y"] = calculated_trajectory.m_path_y;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
