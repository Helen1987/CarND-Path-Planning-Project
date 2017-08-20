#include "estimator.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>

namespace pathplanner {
  using namespace std;

  double Estimator::change_lane_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>> predictions, TrajectoryData data) const {
    /*
    Penalizes lane changes
    */
    if (data.proposed_lane != data.current_lane)
      return COMFORT;

    return 0;
  }

  double Estimator::inefficiency_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>> predictions, TrajectoryData data) const {
    double speed = data.avg_speed;
    double target_speed = MAX_SPEED;
    double diff = target_speed - speed;
    double pct = diff / target_speed;
    double multiplier = pow(pct, 2);
    return multiplier * EFFICIENCY;
  }

  double Estimator::collision_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>> predictions, TrajectoryData data) const {
    if (data.collides.hasCollision) {
      double time_til_collision = data.collides.step*PREDICTION_INTERVAL;
      double exponent = time_til_collision*time_til_collision;
      double mult = exp(-exponent);

      return mult * COLLISION;
    }
    return 0;
  }

  double Estimator::free_line_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>> predictions, TrajectoryData data) const {
    double closest = data.prop_closest_approach;
    //cout << "prop closest " << closest << endl;
    if (closest > 65) {
      return 0.0;
    }
    double multiplier = (MAX_DISTANCE - closest) / MAX_DISTANCE;
    return multiplier * EFFICIENCY;
  }

  double Estimator::buffer_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>> predictions, TrajectoryData data) const {
    double closest = data.actual_closest_approach;
    //cout << "actual closest " << closest << endl;
    if (closest < 2) {
      return 10 * DANGER;
    }

    //double timesteps_away = closest / (data.avg_speed*PREDICTION_INTERVAL);
    if (closest > DESIRED_BUFFER) {
      return 0.0;
    }

    double multiplier = 1.0 - pow((closest / DESIRED_BUFFER), 2);
    return multiplier * DANGER;
    return 0;
  }

  double Estimator::calculate_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
      map<int, vector<Vehicle::prediction>>predictions, string state, bool verbose/*=false*/) {
    TrajectoryData trajectory_data = get_helper_data(trajectory, predictions, state, vehicle.lane);

    double cost = 0.0;
    for (auto cf : delegates) {
      double new_cost = cf(*this, trajectory, predictions, trajectory_data);
      cost += new_cost;
    }
    if (verbose) {
      cout << "has cost " << cost << " for state " << state << endl;
    }
    return cost;
  }


  Estimator::TrajectoryData Estimator::get_helper_data(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>>predictions, string checkstate, int lane) {
    TrajectoryData data = TrajectoryData();

    vector<Vehicle::snapshot> t = trajectory;
    Vehicle::snapshot current_snapshot = t[0];
    Vehicle::snapshot first = t[1];
    Vehicle::snapshot last = t[t.size() - 1];

    double dt = trajectory.size()*PREDICTION_INTERVAL;
    data.current_lane = lane;
    data.proposed_lane = first.proposed_lane;
    data.avg_speed = (last.get_speed()*dt - current_snapshot.get_speed()) / dt; // (v2*dt-v1*1)/dt

    // initialize a bunch of variables
    //vector<double> accels = {};
    data.prop_closest_approach = MAX_DISTANCE;
    data.actual_closest_approach = MAX_DISTANCE;

    data.collides = collision();
    data.collides.hasCollision = false;
    bool checkCollisions = data.current_lane != data.proposed_lane;
    //Vehicle::snapshot last_snap = trajectory[0];
    map<int, vector<Vehicle::prediction>> cars_in_proposed_lane = filter_predictions_by_lane(predictions, data.proposed_lane);
    map<int, vector<Vehicle::prediction>> cars_in_actual_lane = filter_predictions_by_lane(predictions, data.current_lane);

    //cout << checkstate << " state esimation" << endl;
    //cout << "collides in lane: " << data.proposed_lane << endl;
    for (int i = 0; i < PLANNING_HORIZON; ++i) {
      Vehicle::snapshot snap = trajectory[i];

      for (auto pair : cars_in_proposed_lane) {
        Vehicle::prediction state = pair.second[i];
        if (checkCollisions) {
          bool vehicle_collides = check_collision(snap, state, checkstate);
          if (vehicle_collides) {
            data.collides.hasCollision = true;
            data.collides.step = i;
          }
        }
        double dist = state.s - snap.s;
        if (dist > 0 && dist < data.prop_closest_approach) {
          data.prop_closest_approach = dist;
        }
      }
    }
    //cout << "collides in lane: " << data.current_lane << endl;
    for (int i = 0; i < PLANNING_HORIZON; ++i) {
      Vehicle::snapshot snap = trajectory[i];
      //accels.push_back(snap.get_acceleration());

      for (auto pair : cars_in_actual_lane) {
        Vehicle::prediction state = pair.second[i];
        // do not check collisions on actual line
        double dist = state.s - snap.s;
        if (dist > 0 && dist < data.actual_closest_approach) {
          data.actual_closest_approach = dist;
        }
        //last_snap = snap;
      }
    }

    return data;
  }


  bool Estimator::check_collision(Vehicle::snapshot snap, Vehicle::prediction s_now, string checkstate) {
    double s = snap.s;
    double v = snap.get_speed();

    double collide_car_v = s_now.get_velocity();
    //double predicted_distance = PREDICTION_INTERVAL*(s + v - s_now.s - collide_car_v);
    
    //double v_target = (s_now - s_previous) / PREDICTION_INTERVAL;
    if (s_now.s > s && s_now.s - s > MANOEUVRE && (collide_car_v - v) > 5) {
      return false;
    }
    else if (s > s_now.s && s - s_now.s > MANOEUVRE && (v - collide_car_v) > 5) {
      return false;
    }

    return true;
    throw invalid_argument("Incorrect s coordinate for predicted trajectory");
  }

  map<int, vector<Vehicle::prediction>> Estimator::filter_predictions_by_lane(
    map<int, vector<Vehicle::prediction>> predictions, int lane) {
    map<int, vector<Vehicle::prediction>> filtered = {};
    for (auto pair: predictions) {
      //pair.second[0].display();
      if (pair.second[0].is_in_lane(lane)) {
        //cout << "in lane: " << lane << endl;
        filtered[pair.first] = pair.second;
      }
      else {
        //cout << "not in lane: " << lane << endl;
      }
    }
    return filtered;
  }

}