#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include <tuple>
#include <algorithm>

namespace pathplanner {

  Vehicle::Vehicle(int id, double x, double y, double dx, double dy, double s, double d) {

    this->id = id;
    this->x = x;
    this->y = y;
    this->dx = dx;
    this->dy = dy;
    this->s = s;
    this->d = d;
    this->ddx = 0;
    this->ddy = 0;
    state = "CS";
    max_acceleration = 10;
  }

  Vehicle::Vehicle(int id) {
    this->id = id;
    state = "CS";
    max_acceleration = 10;
  }

  Vehicle::~Vehicle() {}

  // TODO - Implement this method.
  void Vehicle::update_state(map<int, vector<Vehicle::prediction>> predictions, int lanes_available) {
    /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
    - The vehicle will attempt to drive its target speed, unless there is
    traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
    - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
    behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
    - The vehicle will find the nearest vehicle in the adjacent lane which is
    BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
    3 : [
    {"s" : 4, "lane": 0},
    {"s" : 6, "lane": 0},
    {"s" : 8, "lane": 0},
    {"s" : 10, "lane": 0},
    ]
    }

    */
    state = get_next_state(predictions, lanes_available);
  }

  string Vehicle::get_next_state(map<int, vector<prediction>> predictions, int lanes_available) {
    vector<string> states;
    if (state == "PLCR") {
      states = vector<string>{ "PLCR", "LCR", "KL" };
    }
    else if (state == "PLCL") {
      states = vector<string>{ "PLCL", "LCL", "KL" };
    }
    else {
      states = vector<string>{ "KL" };
      if (this->d > LANE_WIDTH) {
        states.push_back("PLCL");
      }
      if (this->d < (lanes_available - 1)*LANE_WIDTH) {
        states.push_back("PLCL");
      }
    }

    auto costs = vector<Vehicle::estimate>();
    for (auto state : states) {
      Vehicle::estimate estimate = Vehicle::estimate();
      estimate.state = state;
      auto trajectory = trajectory_for_state(state, predictions);
      estimate.cost = calculate_cost(trajectory, predictions, state);
      costs.push_back(estimate);
    }

    auto best = min_element(std::begin(costs), std::end(costs),
      [](Vehicle::estimate est1, Vehicle::estimate est2) {
      return est1.cost < est2.cost;
    });
    cout << "best estimate: " << (*best).cost << " in state " << (*best).state << endl;
    return (*best).state;
  }

  vector<Vehicle::snapshot> Vehicle::trajectory_for_state(string state, map<int, vector<Vehicle::prediction>> predictions,
    int horizon /*=5*/) {
    // remember current state
    auto initial_snapshot = this->get_snapshot();

    // pretend to be in new proposed state
    this->state = state;
    vector<snapshot> trajectory = { initial_snapshot };
    for (int i = 0; i < horizon; ++i) {
      restore_state_from_snapshot(initial_snapshot);
      this->state = state;
      realize_state(predictions);
      //assert 0 <= self.lane < self.lanes_available, "{}".format(self.lane)
      increment();
      trajectory.push_back(this->get_snapshot());

      // need to remove first prediction for each vehicle.
      for (auto pair : predictions) {
        auto vehicle_pred = pair.second;
        vehicle_pred.erase(vehicle_pred.begin());
      }
    }

    // restore state from snapshot
    restore_state_from_snapshot(initial_snapshot);
    return trajectory;
  }

  Vehicle::snapshot Vehicle::get_snapshot() {
    Vehicle::snapshot snapshot_temp;
    snapshot_temp.x = this->x;
    snapshot_temp.y = this->y;
    snapshot_temp.dx = this->dy;
    snapshot_temp.dy = this->dy;
    snapshot_temp.s = this->s;
    snapshot_temp.d = this->d;
    snapshot_temp.ddx = this->ddx;
    snapshot_temp.ddy = this->ddy;
    snapshot_temp.state = this->state;

    return snapshot_temp;
  }

  double Vehicle::calculate_cost(vector<Vehicle::snapshot> trajectory,
    map<int, vector<prediction>>predictions, string state, bool verbose/*=false*/) {
    /*trajectory_data = get_helper_data(vehicle, trajectory, predictions)
    cost = 0.0
    for cf in [
    distance_from_goal_lane,
    inefficiency_cost,
    collision_cost,
    buffer_cost,
    change_lane_cost]:
    new_cost = cf(vehicle, trajectory, predictions, trajectory_data)
    if DEBUG or verbose:
    print "{} has cost {} for lane {}".format(cf.__name__, new_cost, trajectory[-1].lane)
    # pdb.set_trace()
    cost += new_cost*/
    if (state == "KL")
      return 0;
    return 1000;//cost
  }

  void Vehicle::restore_state_from_snapshot(Vehicle::snapshot snapshot) {
    //s = snapshot
    this->s = snapshot.s;
    this->d = snapshot.d;
    this->dx = snapshot.dx;
    this->dy = snapshot.dy;
    this->ddx = snapshot.ddx;
    this->ddy = snapshot.ddy;
    this->state = snapshot.state;
  }

  //void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    //target_speed = road_data[0];
    //lanes_available = road_data[1];
    //goal_s = road_data[2];
    //goal_lane = road_data[3];
    //max_acceleration = road_data[4];
  //}

  string Vehicle::display() {

    cout << "s:    " << this->s << endl;
    cout << "d: " << this->d << endl;
    cout << "vx:    " << this->dx << endl;
    cout << "vy:    " << this->dy << endl;
    cout << "ax:    " << this->ddx << endl;
    cout << "ay:    " << this->ddy << endl;
  }

  void Vehicle::update_params(double x, double y, double v, double yaw, double s, double d) {
    this->x = x;
    this->y = y;
    this->dx = v*cos(yaw);
    this->dy = v*sin(yaw);
    this->s = s;
    this->d = d;
  }

  void Vehicle::increment(int dt /*=1*/) {

    double t = TIME_INTERVAL*dt;
    this->s += this->dy * t + this->ddy*t*t/2;
    this->d += this->dx * t + this->ddx*t*t/2;
    this->dx += this->ddx * t;
    this->dy += this->ddy * t;
  }

  Vehicle::prediction Vehicle::state_at(int count) {

    /*
    Predicts state of vehicle in t seconds (assuming 0 acceleration)
    */
    prediction pred;
    double t = count*TIME_INTERVAL;
    pred.s = this->s + this->dy * t + this->ddy * t * t / 2;
    pred.vy = this->dy + this->ddy * t;
    pred.d = this->d + this->dx * t + this->ddx * t * t / 2;
    pred.vx = this->dx + this->ddx * t;
    return pred;
  }

  bool Vehicle::collides_with(Vehicle other, int at_time) {

    /*
    Simple collision detection.
    */
    prediction check1 = state_at(at_time);
    prediction check2 = other.state_at(at_time);
    return (abs(check1.d - check2.d) < LANE_WIDTH) && (abs(check1.s - check2.s) <= SAFE_DISTANCE);
  }

  Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;

    for (int t = 0; t < timesteps + 1; t++)
    {
      if (collides_with(other, t))
      {
        collider_temp.collision = true;
        collider_temp.time = t;
        return collider_temp;
      }
    }

    return collider_temp;
  }

  void Vehicle::realize_state(map<int, vector<prediction>> predictions) {

    /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if (state.compare("CS") == 0)
    {
      realize_constant_speed();
    }
    else if (state.compare("KL") == 0)
    {
      realize_keep_lane(predictions);
    }
    else if (state.compare("LCL") == 0)
    {
      realize_lane_change(predictions, "L");
    }
    else if (state.compare("LCR") == 0)
    {
      realize_lane_change(predictions, "R");
    }
    else if (state.compare("PLCL") == 0)
    {
      realize_prep_lane_change(predictions, "L");
    }
    else if (state.compare("PLCR") == 0)
    {
      realize_prep_lane_change(predictions, "R");
    }
    cout << "state: " << state << " ref_vel: " << ref_vel << " lane: " << lane << endl;
  }

  void Vehicle::realize_constant_speed() {
    ddx = 0;
    ddy = 0;
  }

  void Vehicle::_update_ref_speed_for_lane(map<int, vector<prediction>> predictions, int lane, int s) {
    bool too_close = false;
    for (auto pair : predictions) {
      vector<prediction> preds = pair.second;
      for (prediction pred : preds) {
        if (pred.d < (LANE_WIDTH * (lane + 1)) && pred.d >(LANE_WIDTH * lane)) {

          //double check_speed = sqrt(pred.vx*pred.vx + pred.vy*pred.vy);

          if ((pred.s > s) && (pred.s - s) < SAFE_DISTANCE) {
            too_close = true;
          }
        }
      }
    }
    if (too_close) {
      ref_vel -= SPEED_INCREMENT;
    }
    else if (ref_vel < MAX_SPEED) {
      ref_vel += SPEED_INCREMENT;
    }
  }

  void Vehicle::realize_keep_lane(map<int, vector<prediction>> predictions) {
    _update_ref_speed_for_lane(predictions, this->lane, this->s);
  }

  void Vehicle::realize_lane_change(map<int, vector<prediction>> predictions, string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
      delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    _update_ref_speed_for_lane(predictions, lane, s);
  }

  void Vehicle::realize_prep_lane_change(map<int, vector<prediction> > predictions, string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
      delta = 1;
    }
    int lane = this->lane + delta;

    vector<vector<prediction>> at_behind;
    for (auto pair : predictions) {
      int v_id = pair.first;
      vector<prediction> v = pair.second;
      if ((abs(v[0].d - lane) < LANE_WIDTH) && (v[0].s < this->s)) {
        at_behind.push_back(v);
      }
    }
    if (at_behind.size() > 0)
    {
      int max_s = -1000;
      vector<prediction> nearest_behind = {};
      for (auto pred: at_behind) {
        if ((pred[0].s) > max_s)
        {
          max_s = pred[0].s;
          nearest_behind = pred;
        }
      }
      double target_vel = (nearest_behind[1].s - nearest_behind[0].s)/TIME_INTERVAL;
      int delta_v = get_velocity() - target_vel;
      int delta_s = this->s - nearest_behind[0].s;
      if (abs(delta_v) > 0.01)
      {
        int time = -2 * delta_s / delta_v;
        int velocity;
        if (time == 0) {
          velocity = ref_vel;
        }
        else {
          velocity = ref_vel + SPEED_INCREMENT;
        }
        if (velocity > MAX_SPEED) {
          velocity = MAX_SPEED;
        }
        ref_vel = velocity;
      }
    }
  }

  vector<Vehicle::prediction> Vehicle::generate_predictions(int horizon = 10) {

    vector<prediction> predictions;
    for (int i = 0; i < horizon; i++)
    {
      predictions.push_back(state_at(i));
    }
    return predictions;
  }
}