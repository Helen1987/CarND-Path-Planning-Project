#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <chrono>
#include "json.hpp"

#include <iostream>
#include <math.h>
#include <map>
#include <vector>

#include "vehicle.h"
#include "FSM.h"
#include "trajectory.h"

namespace pathplanner {
  using namespace std;
  using namespace std::chrono;
  using json = nlohmann::json;

  class PathPlanner {

  public:

    PathPlanner();
    virtual ~PathPlanner();

    void update_vehicle_state(json sensor_fusion);
    void update_ego_car_state(double car_s, double x, double y, double yaw, double s, double d, double speed);
    void generate_trajectory(vector<double> previous_path_x, vector<double> previous_path_y, double car_s, double original_x, double original_y, double original_yaw);
    vector<double> get_x_values();
    vector<double> get_y_values();

  private:
    double diff;
    milliseconds ms;
    map<int, Vehicle*> vehicles;
    map<int, vector<prediction>> predictions;
    Vehicle ego_car = Vehicle(-1);
    FSM fsm = FSM(ego_car);
    Trajectory trajectory = Trajectory();

    double get_time_step();
  };
}

#endif