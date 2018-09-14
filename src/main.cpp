#include "planner.h"
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

using namespace std;

// for convenience
using json = nlohmann::json;

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

std::map<BhState, int> lane_direction = {{BhState::LCL, -1}, {BhState::LCR, 1}, {BhState::KL, 0}};


void printState(BhState next_state);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  int lane = 1; //starts from zero, 0 left lane
  double target_velocity = 0.0;
  BhState current_state = BhState::KL; //start with keep lane


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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&target_velocity,&current_state](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
//        std::cout << s << std::endl;
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = deg2rad(j[1]["yaw"]);
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          cout << "Sensor fusion: ";
          for (int k = 0; k <sensor_fusion.size(); ++k) {
            vector<double> d = sensor_fusion[k];
            double vx = d[3];
            double vy = d[4];

            cout << "{" << d[0] << ": s=" << (d[5]-car_s) << ", l=" << (int)(d[6]/LANE_WIDTH) << ", v=" << sqrt(vx*vx+vy*vy) << "}";
          }
          cout << endl;

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<double> prev_x;
          vector<double> prev_y;
          for (int i = 0; i < previous_path_x.size(); ++i) {
            prev_x.push_back(previous_path_x[i]);
            prev_y.push_back(previous_path_y[i]);
          }

          //NEW:

          // check for cars in front
          bool drone_too_close = false;
          double close_drone_velocity = 0.0;
          double close_drone_distance = 1000.0;
          double safe_distance = 30.0;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            auto drone = sensor_fusion[i];
            double drone_d = drone[6];

            if (drone_d > lane * LANE_WIDTH && drone_d < LANE_WIDTH * (lane + 1)) {
              //this car is in our lane
              double drone_s = (double)drone[5];
              double drone_dist = drone_s - car_s;
              if (car_s < max_s - 2*safe_distance && drone_s < 2*safe_distance) {
                drone_dist = drone_s + max_s - car_s;
              }

              if (drone_s > car_s && drone_dist < safe_distance && drone_dist < close_drone_distance) {
                drone_too_close = true;
                close_drone_distance = drone_dist;
                double drone_vx = drone[3];
                double drone_vy = drone[4];
                close_drone_velocity = sqrt(drone_vx*drone_vx + drone_vy*drone_vy);
                std::cout << "Drone too close " << drone[0] << ", dist=" << drone_dist
                          << ", drone_s=" << (double)drone[5] << ", drone_d=" << drone_d << ", car_s=" << car_s
                          << ", drone_v=" << close_drone_velocity
                          << ", drone_lane=" << (int)(drone_d/LANE_WIDTH) << ", car_lane=" << lane << std::endl;
              }
            }
          }

          BhState next_state = BhState::KL;

          if (drone_too_close && (close_drone_distance < safe_distance - 10 || target_velocity > close_drone_velocity)) {
            target_velocity -= .224;
            std::cout << "Dec velocity to: " << target_velocity << ", drone_v=" << close_drone_velocity << std::endl;

//            if (lane > 0) { //TODO
//              next_state = BhState::LCL;
//            }
          } else if (target_velocity < IDEAL_VELOCITY) {
            target_velocity += .224;
            std::cout << "Inc velocity to: " << target_velocity << std::endl;
          }

          // simple behavior planner here to define strategy of the next steps

          cout << "Current state=";
          printState(current_state);
          cout << ", lane=" << lane << ", costs: ";

          vector<BhState> next_states = getNextStates(current_state, lane);
          vector<double> cost;
          for (auto bh_state : next_states) {
            int new_lane = lane + lane_direction[bh_state];
            // generate trajectory
            // calculate cost of the trajectory
            double lateral_disp_cost = (new_lane * LANE_WIDTH + 0.5 * LANE_WIDTH - car_d) / LANE_WIDTH;
            lateral_disp_cost = tanh(0.3*lateral_disp_cost*lateral_disp_cost*lateral_disp_cost);

            double c = costSpeed(new_lane, car_s, sensor_fusion) + 2 * lateral_disp_cost
                       + 1.5 * costDistance(new_lane, car_s, sensor_fusion);
            cost.push_back(c);
            cout << "";
            printState(bh_state);
            cout << "=" << c << ", ";
          }
          cout << endl;

          long state_index = std::distance(cost.begin(), std::min_element(cost.begin(), cost.end()));
          next_state = next_states[state_index];
          cout << "Next state: ";
          printState(next_state);
          cout << endl;

          lane += lane_direction[next_state]; //execute lane change on state change
          current_state = next_state;

          auto next = planTrajectory(car_x, car_y, car_yaw, lane, target_velocity, prev_x, prev_y, map_waypoints_x, map_waypoints_y,
                                     map_waypoints_s);


          msgJson["next_x"] = next[0];
          msgJson["next_y"] = next[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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

void printState(BhState next_state) {
  switch (next_state) {
    case BhState::KL: cout << "KL"; break;
    case BhState::LCL: cout << "LCL"; break;
    case BhState::LCR: cout << "LCR"; break;
  }
}


