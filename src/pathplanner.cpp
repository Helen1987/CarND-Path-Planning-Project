#include <chrono>

#include "json.hpp"
#include "vehicle.h"
#include "map.h"
#include "FSM.h"
#include "trajectory.h"

#include "pathplanner.h"

namespace pathplanner {
  using namespace std;
  using json = nlohmann::json;
  using namespace std::chrono;

  PathPlanner::PathPlanner() {
    ms = duration_cast<milliseconds>(
      system_clock::now().time_since_epoch()
    );
    
  }

  double PathPlanner::get_time_step() {
    milliseconds new_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    diff = (double)(new_time - ms).count() / 1000;
    ms = new_time;
    return diff;
  }

  PathPlanner::~PathPlanner() {}

  // [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, 
  // car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, 
  // car's d position in frenet coordinates.
  void PathPlanner::update_vehicle_state(json sensor_fusion){
    double diff = get_time_step();

    predictions.clear();
    for (auto data : sensor_fusion) {
      // [id, x, y, dx, dy, s, d]
      Vehicle* vehicle = NULL;
      if (((double)data[5] < Map::MAX_S) && ((double)data[6] > 0)) {// check if car is visible
        if (vehicles.find(data[0]) == vehicles.end()) {

          vehicle = new Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
          vehicles[data[0]] = vehicle;
        }
        else {
          vehicle = vehicles[data[0]];
          (*vehicle).update_yaw(data[1], data[2], data[3], data[4], data[5], data[6], diff);
          if ((*vehicle).shouldPredict()) {
            vector<prediction> car_preds = (*vehicle).generate_predictions(fsm.PREDICTION_INTERVAL);
            predictions[(*vehicle).id] = car_preds;
          }
        }
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
  }

  void PathPlanner::update_ego_car_state(double car_s, double x, double y, double yaw, double s, double d, double speed) {
    original_yaw = yaw;
    fsm.car_s = car_s;
    fsm.ego_car.update_params(x, y, yaw, s, d, speed, diff);
  }

  void PathPlanner::generate_trajectory(vector<double> previous_path_x, vector<double> previous_path_y) {
    fsm.update_state(predictions);
    fsm.realize_state(predictions);
    trajectory.set_previous_path(previous_path_x, previous_path_y);
    trajectory.generate_trajectory(fsm.car_s, ego_car.x, ego_car.y, original_yaw, ego_car.lane, fsm.get_expected_velocity());
  }

  vector<double> PathPlanner::get_x_values() {
    return trajectory.next_x_vals;
  }

  vector<double> PathPlanner::get_y_values() {
    return trajectory.next_y_vals;
  }
}