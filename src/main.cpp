#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "libs/spline.h"
#include "vehicle.h"
#include "trajectory.h"
#include "helper_functions.h"
#include "map.h"
#include "FSM.h"


using namespace std;
using namespace pathplanner;
using namespace std::chrono;
using namespace helpers;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
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


int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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
    Map::add_waypoints(x, y, s, d_x, d_y);
  }

  //int lane = 1;
  //double ref_vel = 0.0;//mps
  Trajectory trajectory = Trajectory();
  Vehicle ego_car = Vehicle(-1);
  FSM fsm = FSM(ego_car);
  map<int, Vehicle*> vehicles;

  milliseconds ms = duration_cast<milliseconds>(
    system_clock::now().time_since_epoch()
  );
  h.onMessage([&trajectory, &max_s, &ms, &vehicles, &fsm](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
    uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    milliseconds new_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        double diff = (double)(new_time - ms).count() / 1000;
        //cout << "time diff is : " << diff << endl;

        string event = j[0].get<string>();

        if (event == "telemetry") {
          ms = new_time;
          // j[1] is the data JSON object

          // Main car's localization Data
          double original_x = j[1]["x"];
          double original_y = j[1]["y"];
          double original_s = j[1]["s"];
          double original_d = j[1]["d"];
          double original_yaw = j[1]["yaw"];
          double original_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          trajectory.set_previous_path(previous_path_x, previous_path_y);

          json msgJson;
          double TIME_INTERVAL = 0.02;
          //cout << j << endl;
          double car_s = original_s;

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            /*double ref_x = previous_path_x[1];
            double ref_y = previous_path_y[1];

            double ref_x_prev = previous_path_x[0];
            double ref_y_prev = previous_path_y[0];
            original_vx = (ref_x - car_x) / TIME_INTERVAL;
            original_vy = (ref_y - car_y) / TIME_INTERVAL;
            original_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            Frenet frenet = Map::getFrenet(ref_x_prev, ref_y_prev, original_yaw);
            original_s = frenet.s;
            original_d = frenet.d;*/

            car_s = end_path_s;
          }
          //cout << "ego car: " << endl;
          //ego_car.display();
          // [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, 
          // car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, 
          // car's d position in frenet coordinates.
          map<int, vector<prediction>> predictions;
          for (auto data: sensor_fusion) {
            // [id, x, y, dx, dy, s, d]
            Vehicle* vehicle = NULL;
            if (((double)data[5] < max_s) && ((double)data[6] > 0)) {// check if car is visible
              if (vehicles.find(data[0]) == vehicles.end()) {

                vehicle = new Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
                vehicles[data[0]] = vehicle;
              }
              else {
                vehicle = vehicles[data[0]];
                (*vehicle).update_yaw(data[1], data[2], data[3], data[4], data[5], data[6], diff);
                if ((*vehicle).shouldPredict()) {
                  vector<prediction> car_preds = (*vehicle).generate_predictions();
                  predictions[(*vehicle).id] = car_preds;
                }
              }
              //(*vehicle).display();
            }
            else {
              auto it = vehicles.find(data[0]);
              if (it != vehicles.end()) {
                cout << " remove vehicle: " << data[0] << endl;
                delete (*it).second;
                vehicles.erase((int)data[0]);
              }
            }
          }
          //cout << "update ego car " << car_speed << endl;
          fsm.ego_car.update_params(original_x, original_y, original_yaw, original_s, original_d, original_speed, diff);
          fsm.car_s = car_s;
          fsm.update_state(predictions);
          fsm.realize_state(predictions);
          //cout << "state: " << ego_car.state << " ref_vel: " << ego_car.get_velocity() << " lane: " << ego_car.lane << endl;

          trajectory.generate_trajectory(car_s, original_x, original_y, original_yaw, fsm.ego_car.lane, fsm.get_expected_velocity());

          msgJson["next_x"] = trajectory.next_x_vals;
          msgJson["next_y"] = trajectory.next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      }
      else {
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
    }
    else {
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
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
