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

  double Estimator::buffer_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>> predictions, TrajectoryData data) const {
    double closest = data.closest_approach;
    if (closest < 1) {
      return 10 * DANGER;
    }

    double timesteps_away = closest / (data.avg_speed*INTERVAL);
    if (timesteps_away > DESIRED_BUFFER) {
      return 0.0;
    }

    double multiplier = 1.0 - pow((timesteps_away / DESIRED_BUFFER), 2);
    return multiplier * DANGER;
    return 0;
  }

  double Estimator::calculate_cost(Vehicle vehicle, vector<Vehicle::snapshot> trajectory,
      map<int, vector<Vehicle::prediction>>predictions, string state, bool verbose/*=false*/) {
    TrajectoryData trajectory_data = get_helper_data(trajectory, predictions);
    trajectory_data.current_lane = vehicle.lane;

    double cost = 0.0;
    for (auto cf : delegates) {
      double new_cost = cf(*this, trajectory, predictions, trajectory_data);

      //cout << "has cost " << new_cost << " for lane " << trajectory[trajectory.size()-1].lane << endl;

      cost += new_cost;
    }
    if (state == "PLCR" || state == "PLCL")
    {
      //cout << "has cost " << cost << " for state " << state << endl;
    }
    return cost;
    /*if (state == "KL")
      return 0;
    return 1000;//cost*/
  }


  Estimator::TrajectoryData Estimator::get_helper_data(vector<Vehicle::snapshot> trajectory,
    map<int, vector<Vehicle::prediction>>predictions) {
    TrajectoryData data = TrajectoryData();

    vector<Vehicle::snapshot> t = trajectory;
    Vehicle::snapshot current_snapshot = t[0];
    Vehicle::snapshot first = t[1];
    Vehicle::snapshot last = t[t.size() - 1];

    //end_distance_to_goal = vehicle.goal_s - last.s
    //end_lanes_from_goal = abs(vehicle.goal_lane - last.lane)
    double dt = trajectory.size()*PREDICTION_INTERVAL;
    data.proposed_lane = first.lane;
    data.avg_speed = (last.get_speed()*dt - current_snapshot.get_speed()) / dt; // (v2*dt-v1*1)/dt

    // initialize a bunch of variables
    //vector<double> accels = {};
    data.closest_approach = 999999;

    data.collides = collision();
    data.collides.hasCollision = false;
    //Vehicle::snapshot last_snap = trajectory[0];
    map<int, vector<Vehicle::prediction>> filtered = filter_predictions_by_lane(predictions, data.proposed_lane);

    for (int i = 1; i < PLANNING_HORIZON + 1; ++i) {
      Vehicle::snapshot snap = trajectory[i];
      //accels.push_back(snap.get_acceleration());

      for (auto pair : filtered) {
        Vehicle::prediction state = pair.second[i];
        Vehicle::prediction last_state = pair.second[i - 1];
        bool vehicle_collides = check_collision(snap, last_state.s, state.s);
        if (vehicle_collides) {
          data.collides.hasCollision = true;
          data.collides.step = i;

          double dist = abs(state.s - snap.s);
          if (dist < data.closest_approach) {
            data.closest_approach = dist;
          }
        }
        //last_snap = snap;
      }
    }
    /*auto smallest = min_element(accels.begin(), accels.end());
    data.max_acceleration = *smallest;
    vector<double> rms_accels = {};
    rms_accels.resize(accels.size());
    transform(accels.begin(), accels.end(), rms_accels.begin(), [](double acc) {
      return acc*acc;
    });
    int num_accels = rms_accels.size();
    data.rms_acceleration = accumulate(rms_accels.begin(), rms_accels.end(), 0) / num_accels;*/

    return data;
  }


  bool Estimator::check_collision(Vehicle::snapshot snap, double s_previous, double s_now) {
    double s = snap.s;
    double v = snap.get_speed();
    double v_target = (s_now - s_previous) / PREDICTION_INTERVAL;
    if ((s_previous - s) < 1) {
      if (v_target > v) {
        return false;
      }
      else {
        return true;
      }
    }

    if (s_previous < s) {
      if (s_now >= s) {
        return true;
      }
      else {
        return false;
      }
    }

    if (s_previous > s) {
      if (s_now <= s) {
        return true;
      }
      else {
        return false;
      }
    }

    throw invalid_argument("Incorrect s coordinate for predicted trajectory");
  }

  map<int, vector<Vehicle::prediction>> Estimator::filter_predictions_by_lane(
    map<int, vector<Vehicle::prediction>> predictions, int lane) {
    map<int, vector<Vehicle::prediction>> filtered = {};
    for (auto pair: predictions) {
      if (pair.second[0].is_in_lane(lane)) {
        filtered[pair.first] = pair.second;
      }
    }
    return filtered;
  }

}