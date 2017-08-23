#include "estimator.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>
#include <functional>
#include <numeric>

namespace pathplanner {
  using namespace std;

  double Estimator::change_lane_cost(vector<snapshot> trajectory,
    map<int, vector<prediction>> predictions, TrajectoryData data) const {
    /*
    Penalizes lane changes
    */
    if (data.proposed_lane != data.current_lane)
      return COMFORT;

    return 0;
  }

  double Estimator::inefficiency_cost(vector<snapshot> trajectory,
    map<int, vector<prediction>> predictions, TrajectoryData data) const {
    double speed = data.avg_speed;
    double target_speed = MAX_SPEED;
    double diff = target_speed - speed;
    double pct = diff / target_speed;
    double multiplier = pow(pct, 2);
    return multiplier * EFFICIENCY;
  }

  double Estimator::collision_cost(vector<snapshot> trajectory,
    map<int, vector<prediction>> predictions, TrajectoryData data) const {
    if (data.collides.hasCollision) {
      double time_til_collision = 0;
      double exponent = time_til_collision*time_til_collision;
      double mult = exp(-exponent);
      //cout << " collision: " << mult * COLLISION << " on step: " << data.collides.step << endl;
      return mult * COLLISION;
    }
    return 0;
  }

  double Estimator::free_line_cost(vector<snapshot> trajectory,
    map<int, vector<prediction>> predictions, TrajectoryData data) const {
    double closest = data.prop_closest_approach;
    //cout << "prop closest " << closest << endl;
    if (closest > 85) {
      return 0.0;
    }
    double multiplier = (MAX_DISTANCE - closest) / MAX_DISTANCE;
    return 13 * multiplier * EFFICIENCY;
  }

  double Estimator::buffer_cost(vector<snapshot> trajectory,
    map<int, vector<prediction>> predictions, TrajectoryData data) const {
    double closest = data.actual_closest_approach;
    //cout << "actual closest " << closest << endl;
    if (closest < 25) {
      return 3 * DANGER;
    }

    //double timesteps_away = closest / (data.avg_speed*PREDICTION_INTERVAL);
    if (closest > DESIRED_BUFFER) {
      return 0.0;
    }

    double multiplier = 1.0 - pow((closest / DESIRED_BUFFER), 2);
    return multiplier * DANGER;
  }

  double Estimator::calculate_cost(vector<snapshot> trajectory,
      map<int, vector<prediction>>predictions, CarState state, bool verbose/*=false*/) {
    TrajectoryData trajectory_data = get_helper_data(trajectory, predictions, state);

    double cost = 0.0;
    for (auto cf : delegates) {
      double new_cost = cf(*this, trajectory, predictions, trajectory_data);
      cost += new_cost;
    }
    if (verbose) {
      cout << "has cost " << cost << " for state " << (int)state << endl;
    }
    return cost;
  }


  Estimator::TrajectoryData Estimator::get_helper_data(vector<snapshot> trajectory,
    map<int, vector<prediction>>predictions, CarState checkstate) {
    TrajectoryData data = TrajectoryData();

    vector<snapshot> t = trajectory;
    // actual state
    snapshot current_snapshot = t[0];
    snapshot first = t[1];
    snapshot last = t[t.size() - 1];

    double dt = trajectory.size()*PREDICTION_INTERVAL;
    data.current_lane = current_snapshot.lane;
    data.proposed_lane = last.lane;
    data.avg_speed = (last.get_speed()*dt - current_snapshot.get_speed()) / dt; // (v2*dt-v1*1)/dt

    // initialize a bunch of variables
    //vector<double> accels = {};
    data.prop_closest_approach = MAX_DISTANCE;
    data.actual_closest_approach = MAX_DISTANCE;

    data.collides = collision();
    data.collides.hasCollision = false;
    bool checkCollisions = data.current_lane != data.proposed_lane;
    //Vehicle::snapshot last_snap = trajectory[0];
    map<int, vector<prediction>> cars_in_proposed_lane = filter_predictions_by_lane(predictions, data.proposed_lane);
    map<int, vector<prediction>> cars_in_actual_lane = filter_predictions_by_lane(predictions, data.current_lane);

    //cout << checkstate << " state esimation" << endl;
    //cout << "collides in lane: " << data.proposed_lane << " size " << cars_in_proposed_lane.size() << endl;
    for (int i = 0; i < PLANNING_HORIZON; ++i) {
      snapshot snap = trajectory[i];

      for (auto pair : cars_in_proposed_lane) {
        prediction state = pair.second[i];
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
    //cout << "collides in lane: " << data.current_lane << " size " << cars_in_actual_lane.size() << endl;
    for (int i = 0; i < PLANNING_HORIZON; ++i) {
      snapshot snap = trajectory[i];
      //accels.push_back(snap.get_acceleration());

      for (auto pair : cars_in_actual_lane) {
        prediction state = pair.second[i];
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


  bool Estimator::check_collision(snapshot snap, prediction s_now, CarState checkstate) {
    double s = snap.s;
    // TOOD: get car's speed in moment as original_s
    double v = snap.get_speed();

    double collide_car_v = s_now.get_velocity();
    //cout << " lane " << snap.lane << " s " << s << " v " << v << " or_s " << snap.original_s << " col_v " << collide_car_v << endl;
    if (snap.s-MANOEUVRE <= s_now.s && s_now.s <= car_s + MANOEUVRE) {
      return true;
    }
    if (snap.s > s_now.s) {
      if (snap.s - s_now.s > 2*MANOEUVRE && v > collide_car_v && snap.get_speed() > collide_car_v) {
        double predicted_distance = snap.s - s_now.s + PREDICTION_INTERVAL*(snap.get_speed() - collide_car_v);
        if (predicted_distance > 3*MANOEUVRE) {
          return false;
        }
      }
    }
    else {
      if (s_now.s > s && s_now.s - s > 3 * MANOEUVRE) {
        return false;
      }
    }
    //cout << "4 clause" << " s " << s << " v " << v << " or_s " << snap.original_s << " col_v " << collide_car_v << endl;
    //s_now.display();

    return true;
    throw invalid_argument("Incorrect s coordinate for predicted trajectory");
  }

  map<int, vector<prediction>> Estimator::filter_predictions_by_lane(
    map<int, vector<prediction>> predictions, int lane) {
    map<int, vector<prediction>> filtered = {};
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